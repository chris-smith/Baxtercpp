#include "searchgeometry.h"
#include "rosCamWrap.h"
#include "baxterLimb.h"

/*------------------------------------------
 * Positive x direction --> Away from front plate
 * Positive y direction --> Towards Baxter's left side
 * Positive z direction --> Vertical up 
 * 
 * Postive Pitch --> Right hand rule x-axis
 * Positive Roll --> Right hand rule y-axis
 * Positive Yaw --> Right hand rule z-axis
 * 
 * -----------------------------------------*/

#define width_ratio 0.832       //width camera sees given a range [m/m]
#define height_ratio 0.523      //height camera sees given a range [m/m]

#ifndef SEARCH_CONTROL
#define SEARCH_CONTROL

class SearchControl
{
    // The purpose of this class is to implement a search control structure
    // So individual variables can be set up (limbs and cameras) before hand
    // Search Geometry should be set and checked to ensure valid choices are made
    
public:
    SearchControl(BaxterLimb*, BaxterLimb*, BaxterCamera*, PRYPose); //cam hand, other hand, cam
    
    //Use this to define geometry
    SearchGeometry geometry;
    
    double min_area;       // smallest area to be considered an object. default 100
    Point allowable_error;
    
    //main functions
    void search();         //main loop that searches for pieces, adjusts _state as necessary
        // typically, search would be called in a loop. At that point, you should
        // not perform distinct actions on the limbs/cameras you passed when
        // declaring an instance of this class.
    
private:
    SearchControl(); //no access to default constructor
    
    BaxterLimb* _cam_hand;
    BaxterLimb* _manip_hand;
    BaxterCamera* _search_cam;
    cv::Mat whole_scene;
    
    //State trackers
    uchar _state;            //  {0: searching for piece, 1: found piece & moving over,
                            //      2: verifying piece, 3: grabbing piece, 4: moving over dropoff
                            //      & depositing piece}
    bool _search_initialized; //set true when search starts. set false when search stops

    //Search Functions
    void _init_search();                                 //  starts timer, sets _search_initialized
    void _search_timer_callback(const ros::TimerEvent&); //  _search_loop_timer callback. calls search()
    void _search();                                      //  _state == 0
    void _move_to_piece();                               //  _state == 1
    void _verify_piece();                                //  _state == 2
    void _grab_piece();                                  //  _state == 3
    void _deposit_piece();                               //  _state == 4
    
    //Called by Search Functions
    bool _object();                                      //  looks for an object in _search_cam
    bool _roi_object();                                  //  looks for an object in subset of the image
    void _reset_loc();                                   //  sets _obj_loc to (-1,-1)
    void _reset_roi();                                   //  sets _roi to (-1,-1)
    void _set_roi(cv::Rect, double);                     //  set _roi based on _obj_loc and area of obj
    cv::Point _get_centroid(std::vector< cv::Point >);   //  returns centroid of a set of points, calculated as mean of points
    void _transform(Point &, double);                    //  transforms dimensionless error to real distance. accounts for hand rotation
    cv::Rect _get_bounds(std::vector<cv::Point>);        //  returns a bounding rectangle for the contour
    std::vector<cv::Point> _corners(cv::Rect);
    std::vector<cv::Point> _increase_bounds(std::vector<cv::Point>, cv::Point);
    cv::Rect _restrict(std::vector<cv::Point>, double,double);     //  returns rectangle enclosed in size of image
    
    
    //Search variables
    cv::Point2d _obj_loc;                                //  location of found object in _object()
    cv::Rect _roi;                                       //   subset of image where object may be found
    double scene_height;
    double scene_width;
};

SearchControl::SearchControl()
{
    ROS_ERROR("This object can not be declared without passing it arguments. \
        The default constructor on this object is private. \
        You should never see this error message.");
}

SearchControl::SearchControl(BaxterLimb* a, BaxterLimb* b, BaxterCamera* cam, PRYPose home)
{
    _cam_hand = a;
    _manip_hand = b;
    _search_cam = cam;
    _state = 0;
    _reset_loc();
    min_area = 100;
    geometry.home = home;
    scene_height = _search_cam->height();
    scene_width = _search_cam->width();
}

void SearchControl::search()
{
    if(!_search_initialized)
        _init_search();
    
    std::cout<<"state: ";
    switch(_state)
    {
        case 0:
            //  Searching, move around in bounding box
            //  obj found ? _state = 1 : _state = _state
            _search();
            std::cout<<"0";
            break;
        case 1:
            //  Found something, move over
            //  obj in bounds ? move over, _state = 2 : state = 0
            _move_to_piece();
            std::cout<<"1";
            break;
        case 2:
            //  Verify piece
            //  known piece ? _state = 3 : _state = 0
            _verify_piece();
            std::cout<<"2";
            break;
        case 3:
            //  Grab piece
            //  piece grabbed ? _state = 4 : _state = 3
            _grab_piece();
            std::cout<<"3";
            break;
        case 4:
            //  Move over deposit box, deposit piece
            //  move over, piece released -> _state = 0
            _deposit_piece();
            std::cout<<"4";
            break;
        default:
            //Shouldn't happen
            ROS_ERROR("_state has been set to an unknown value");
            _state = 0;  //should this happen??
            break;
    }
    std::cout<<std::endl;
    ros::spinOnce();
}

void SearchControl::_init_search()
{
    _search_initialized = true;
    _state = 0;
    allowable_error = (Point){10,10,0};
    PRYPose pos = geometry.home;
    if (geometry.table_height == 0 || geometry.height_offset == 0)
        pos.point.z = .15;
    else
        pos.point.z = geometry.table_height + geometry.height_offset;
    std::cout<<"height: "<<pos.point.z<<"\n";
    std::cout<<_search_cam->height()<<" "<<_search_cam->width();    std::cout<<"roll: " << pos.pry.roll <<"\npitch: "<<pos.pry.pitch<<"\nyaw: "<<pos.pry.yaw<<"\n";
    Pose pos2;
    pos2.point = pos.point;
    pos2.quaternion = toQuat(pos.pry);
    /*std::cout<<"w: "<<pos2.quaternion.w<<"\nx: "<<pos2.quaternion.x<<"\n";
    std::cout<<"y: "<<pos2.quaternion.y<<"\nz: "<<pos2.quaternion.z<<"\n";
    pos.pry = toPRY(pos2.quaternion);
    std::cout<<"roll: " << pos.pry.roll <<"\npitch: "<<pos.pry.pitch<<"\nyaw: "<<pos.pry.yaw<<"\n";
    */
    ros::Duration timeout(10);
    _cam_hand->set_position_quick(pos, timeout);
}

void SearchControl::_search()
{
    //  _state == 0
    if(_object())
        _state = 1;
    
}
    
void SearchControl::_move_to_piece()
{
    //  _state == 1
    Point err; //difference between centroid of object and center of image;
    err.x = _obj_loc.x - scene_width/2; 
    err.y = _obj_loc.y - scene_height/2;
    // pixels -> pixels/pixel
    err.y /= scene_height;
    err.x /= scene_width; 
    
    double yaw = toPRY(_cam_hand->endpoint_pose().quaternion).yaw;
    _transform(err, yaw);
    err.z = 0;
    if (err < allowable_error)
    {
        _state = 2;
        std::cout<<"arrived at object";
        return;
    }
    /*std::cout<<"y position: " << _obj_loc.y;
    std::cout<<"  y error: " << err.y << "\n";
    std::cout<<"x position: " << _obj_loc.x;
    std::cout<<"  x error: " << err.x << "\n";*/
    
    _state = 0;
    //_cam_hand->endpoint_control(err);
//     if(_roi_object())
//     {
//         ros::spinOnce();
//         _move_to_piece();
//     }
//     else
//     {
//         _state = 0; //reset if object has been lost
//         _reset_roi();
//         _reset_loc();
//     }
}

void SearchControl::_verify_piece()
{
    //  _state == 2
    _state = 0; // ONLY VALID while testing visual servoing
}

void SearchControl::_grab_piece()
{
    //  _state == 3
}

void SearchControl::_deposit_piece()
{
    //  _state == 4
}

bool SearchControl::_object()
{
    _reset_loc();
    cv::Mat scene = _search_cam->cvImage();
    whole_scene = scene;
    cv::Mat thresh;
    
    cv::cvtColor(scene, thresh, CV_BGR2GRAY);
    cv::threshold(thresh, thresh,127,255,cv::THRESH_BINARY_INV); 
    
    std::vector< std::vector< cv::Point > > contours;
    std::vector< cv::Vec4i > hierarchy;
    int largest_area = 0;
    int largest_contour_index = 0;
    cv::Rect bounding_rect;
    
    //cv::Canny(thresh, canny, 5, 250, 3);
    cv::imshow("Thresholded", thresh);
    cv::findContours(thresh, contours, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

    int iters = contours.size();
    //std::cout<<"size: "<<iters<<"\n";
    for(int i = 0; i < iters; i++)
    {
        double a = cv::contourArea(contours[i], false);
        if ( a > largest_area)
        {
            largest_area = a;
            largest_contour_index = i;
            //bounding_rect = boundingRect(contours[i]);
        }
    }
    if (largest_area < min_area)
    {
        cv::imshow("Found Object?", thresh);
        return false;
    }
    
    _obj_loc = _get_centroid(contours[largest_contour_index]);
    circle(thresh, _obj_loc, 10, cv::Scalar(255,0,0), 1, 8);
    bounding_rect = _get_bounds(contours[largest_contour_index]);
    _set_roi(bounding_rect, 2);
    rectangle(scene, bounding_rect, cv::Scalar(255,0,0), 1, 8);
    cv::imshow("bounding box", whole_scene);
    //_set_roi(largest_area); 
    //bounding_rect = cv::boundingRect(contours[largest_contour_index]);
    //rectangle(thresh, bounding_rect, cv::Scalar(255,0,0),1,8,0);
    //std::cout<<"found "<<countours.size()<<" contours.\n";
    imshow("Found Object?", thresh);
    cv::waitKey(10);
    return true;
}

bool SearchControl::_roi_object()
{
    _reset_loc();
    cv::Mat scene = _search_cam->cvImage();
    whole_scene = scene;
    try
    {
        scene = scene(_roi);
    }
    catch (cv::Exception)
    {
        ROS_ERROR("Your region of interest is poorly defined");
        return false;
    }
    cv::Mat thresh;
    
    cv::cvtColor(scene, thresh, CV_BGR2GRAY);
    cv::threshold(thresh, thresh,127,255,cv::THRESH_BINARY_INV); 
    
    std::vector< std::vector< cv::Point > > contours;
    std::vector< cv::Vec4i > hierarchy;
    int largest_area = 0;
    int largest_contour_index = 0;
    cv::Rect bounding_rect;
    
    //cv::Canny(thresh, canny, 5, 250, 3);
    cv::imshow("roi thresholded", thresh);
    cv::findContours(thresh, contours, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

    int iters = contours.size();
    //std::cout<<"size: "<<iters<<"\n";
    for(int i = 0; i < iters; i++)
    {
        double a = cv::contourArea(contours[i], false);
        if ( a > largest_area)
        {
            largest_area = a;
            largest_contour_index = i;
            //bounding_rect = boundingRect(contours[i]);
        }
    }
    if (largest_area < min_area)
    {
        cv::imshow("Found Object?", thresh);
        return false;
    }
    
    _obj_loc = _get_centroid(contours[largest_contour_index]);
    _obj_loc.x += _roi.x;
    _obj_loc.y += _roi.y;
    circle(whole_scene, _obj_loc, 10, cv::Scalar(255,0,0), 1, 8);
    bounding_rect = _get_bounds(contours[largest_contour_index]);
    bounding_rect.x += _roi.x;
    bounding_rect.y += _roi.y;
    //move bounding rect to account for fact that contour
    //was taken from subset of image
    // THENNN do below
    _set_roi(bounding_rect, 2);
    
    rectangle(whole_scene, bounding_rect, cv::Scalar(255,0,0), 1, 8);
    cv::imshow("bounding box", whole_scene);
    //_set_roi(largest_area); 
    //bounding_rect = cv::boundingRect(contours[largest_contour_index]);
    //rectangle(thresh, bounding_rect, cv::Scalar(255,0,0),1,8,0);
    //std::cout<<"found "<<countours.size()<<" contours.\n";
    imshow("ROI Found Object?", thresh);
    cv::waitKey(10);
    return true;
}

cv::Point SearchControl::_get_centroid(std::vector< cv::Point > points)
{
    cv::Point centroid;
    for(int i = 0; i < points.size(); i++)
    {
        centroid.x += points[i].x;
        centroid.y += points[i].y;
    }
    centroid.x /= points.size();
    centroid.y /= points.size();
    return centroid;
}

cv::Rect SearchControl::_get_bounds(std::vector<cv::Point> contour)
{
//     double height = _search_cam->height();
//     double width = _search_cam->width();
    std::vector<cv::Point> bounds(2,cv::Point(0,0));
    bounds[0].x = scene_width;  // xmin
    bounds[0].y = scene_height; // ymin
    bounds[1].x = 0;      // xmax
    bounds[1].y = 0;      // ymax
    cv::Point current_point;
    //std::cout<<"num points: "<<contour.size()<<std::endl;
    for(int i = 0; i < contour.size(); i++)
    {
        current_point = contour[i];
//         std::cout<<"origin: "<<bounds.x<<" "<<bounds.y<<std::endl;
//         std::cout<<"size: "<<bounds.width<<" "<<bounds.height<<std::endl;
        bounds = _increase_bounds(bounds, current_point);
    }
    return cv::Rect(bounds[0], bounds[1]);
    //return _restrict(bounds, scene_height, scene_width);
}

cv::Rect SearchControl::_restrict(std::vector<cv::Point> bounds, double height, double width)
{
    if (bounds[0].x < 0)
        bounds[0].x = 0;
    else if(bounds[1].x > width)
        bounds[1].x = width;
    
    if (bounds[0].y < 0)
        bounds[0].y = 0;
    else if(bounds[1].y > height)
        bounds[1].y = height;
    
    return cv::Rect(bounds[0], bounds[1]);
}

std::vector<cv::Point> SearchControl::_corners(cv::Rect rect)
{
    std::vector<cv::Point> corners(4, (cv::Point){0,0}); //4 corners, start at point 0,0
    corners[0] = (cv::Point){rect.x, rect.y};
    corners[1] = (cv::Point){rect.x + rect.width, rect.y};
    corners[2] = (cv::Point){rect.x + rect.width, rect.y + rect.height};
    corners[3] = (cv::Point){rect.x, rect.y + rect.height};
    return corners;
}

std::vector<cv::Point> SearchControl::_increase_bounds(std::vector<cv::Point> rect, cv::Point new_point)
{
    //std::vector<cv::Point> corners = _corners(rect);
    double ymin, ymax, xmin, xmax, x, y;
//     std::cout<<"new point: "<<new_point.x <<" "<< new_point.y<<std::endl;
//     std::cout<<"xmin: "<<xmin<<" xmax: "<<xmax<<" ymin: "<<ymin<<" ymax: "<<ymax<<std::endl;
    x = new_point.x;
    y = new_point.y;
    if (x > rect[1].x)
        rect[1].x = x;
    else if(x < rect[0].x)
        rect[0].x = x;
    
    if (y > rect[1].y)
        rect[1].y = y;
    else if(y < rect[0].y)
        rect[0].y = y;
    
    
    /*std::cout<<"new origin: "<<ret.x<<" "<<ret.y<<std::endl;
    std::cout<<"new size: "<<ret.width<<" "<<ret.height<<std::endl;*/
    return rect;
}

void SearchControl::_set_roi(cv::Rect rect, double multiplier)
{
    rect = rect * multiplier;
    double xmin,xmax,ymin,ymax;
    xmin = rect.x;
    ymin = rect.y;
    xmax = xmin + rect.width;
    ymax = ymin + rect.height;
    if (xmin < 0)
        xmin = 0;
    else if(xmax > scene_width)
        xmax = scene_width;
    
    if (ymin < 0)
        ymin = 0;
    else if(ymax > scene_height)
        ymax = scene_height;
    
    _roi = cv::Rect(cv::Point(xmin,ymin), cv::Point(xmax,ymax));
}

void SearchControl::_reset_loc()
{
    _obj_loc.x = -1;
    _obj_loc.y = -1;
}

void SearchControl::_reset_roi()
{
    _roi.x = 0;
    _roi.y = 0;
    _roi.width = scene_width;
    _roi.height = scene_height;
}

void SearchControl::_transform(Point &err, double yaw)
{
    // ASSUME THE CAMERA IS POINTING STRAIGHT DOWN (ALONG Z AXIS)
    float range = _cam_hand->gripper->range;
    //std::cout<<"range: "<<range<<"\n";
    //transform between pixels and real distances.
    // (pixels/pixel)*(m/m)*m
    double tempx,tempy;
    tempy = err.y * height_ratio * range;
    tempx = err.x * width_ratio * range;
    
    // account for angle of hand (yaw)
    // 0 yaw has gripper "facing" backwards
    err.x = -tempy*cos(yaw) + tempx*sin(yaw);
    err.y = -tempy*sin(yaw) - tempx*cos(yaw); 
    
    // account for offset of camera (in m)
    /*
     */
}

#endif