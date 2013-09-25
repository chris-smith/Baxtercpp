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

struct Predictor{ int dimension; double k;};   // 0 < k < 1

#ifndef SEARCH_CONTROL
#define SEARCH_CONTROL

class SearchControl
{
    // The purpose of this class is to implement a search control structure
    // So individual variables can be set up (limbs and cameras) before hand
    // Search Geometry should be set and checked to ensure valid choices are made
    
public:
    SearchControl(BaxterLimb*, BaxterLimb*, BaxterCamera*); //cam hand, other hand, search cam, control cam
    
    //Use this to define geometry
    SearchGeometry geometry;
    
    double min_area;       // smallest area to be considered an object. default 100
    gv::Point allowable_error;
    
    void set_predictor(Predictor);     //use to ensure 0 <= k <= 1
    
    //main functions
    void search();         //main loop that searches for pieces, adjusts _state as necessary
        // typically, search would be called in a loop. At that point, you should
        // not perform distinct actions on the limbs/cameras you passed when
        // declaring an instance of this class.
    
    void test_transform(gv::Point, double);
    
private:
    SearchControl(); //no access to default constructor
    
    BaxterLimb* _cam_hand;
    BaxterLimb* _manip_hand;
    BaxterCamera* _search_cam;          // used to find objects in scene
    BaxterCamera* _control_cam;         // used to help control positioning of _search_cam
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
    bool _track_object();
    void _reset_loc();                                   //  sets _obj_loc to (-1,-1)
    void _reset_roi();                                   //  sets _roi to (-1,-1)
    void _set_roi(cv::Rect, double);                     //  set _roi based on _obj_loc and area of obj
    cv::Point _get_centroid(std::vector< cv::Point >);   //  returns centroid of a set of points, calculated as mean of points
    void _transform(gv::Point &, double);                    //  transforms dimensionless error to real distance. accounts for hand rotation
    void _dist_transform(gv::Point &, double);
    cv::Rect _get_bounds(std::vector<cv::Point>);        //  returns a bounding rectangle for the contour
    std::vector<cv::Point> _corners(cv::Rect);
    std::vector<cv::Point> _increase_bounds(std::vector<cv::Point>, cv::Point);
    cv::Rect _restrict(std::vector<cv::Point>, double,double);     //  returns rectangle enclosed in size of image
    cv::Rect _getBlobRoi(cv::Point2d pt);
    std::vector<cv::Point> _getBlob(cv::Mat&);            //   return contour of largest blob in image
    void _endpoint_control(gv::Point);
    void _endpoint_control_accurate(gv::Point err);
    void _drop_to_table(gv::PRYPose);
    double _getBlobArea();
    
    
    //Search variables
    cv::Point2d _obj_loc;                                //  location of found object in _object()
    cv::Point2d _prev_loc;                               //   for use with kalman filter
    cv::Point2d _predicted_loc;                          //   predicted location from Kalman filter
    Predictor _predictor;
    bool _tracking;                                      //   tracking object if true
    bool _track_state;                                   //   initializes tracker
    cv::Rect _roi;                                       //   subset of image where object may be found
    double scene_height;
    double scene_width;
    bool _gripping;
    double _blob_area;
};

SearchControl::SearchControl()
{
    ROS_ERROR("This object can not be declared without passing it arguments. \
        The default constructor on this object is private. \
        You should never see this error message.");
}

SearchControl::SearchControl(BaxterLimb* a, BaxterLimb* b, BaxterCamera* cam)
{
    _cam_hand = a;
    _manip_hand = b;
    _search_cam = cam;
    _state = 0;
    _reset_loc();
    _reset_roi();
    min_area = 100;
    scene_height = _search_cam->height();
    scene_width = _search_cam->width();
    _tracking = false;
    _track_state = false;
    _predictor.dimension = 20;
    _predictor.k = 0.5;
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
            std::cout<<"0";
            _search();
            break;
        case 1:
            //  Found something, move over
            //  obj in bounds ? move over, _state = 2 : state = 0
            std::cout<<"1";
            _tracking = true;
            _move_to_piece();
            break;
        case 2:
            //  Verify piece
            //  known piece ? _state = 3 : _state = 0
            std::cout<<"2";
            _verify_piece();
            break;
        case 3:
            //  Grab piece
            //  piece grabbed ? _state = 4 : _state = 3
            std::cout<<"3";
            _grab_piece();
            break;
        case 4:
            //  Move over deposit box, deposit piece
            //  move over, piece released -> _state = 0
            std::cout<<"4";
            _deposit_piece();
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
    allowable_error = gv::Point(10,10,1); // need the 1 so err < allow is true
    if (geometry.home == gv::PRYPose(0))
    {
        gm::Pose pose = _cam_hand->endpoint_pose();
        geometry.home.position = pose.position;
        geometry.home.pry = toPRY(gv::Quaternion(pose.orientation));
    }
    gv::PRYPose pos = geometry.home;
    if (geometry.table_height == 0 || geometry.height_offset == 0)
        pos.position.z = .15;
    else
        pos.position.z = geometry.table_height + geometry.height_offset;
    geometry.home.position.z = pos.position.z;
    //std::cout<<"height: "<<pos.position.z<<"\n";
    //std::cout<<_search_cam->height()<<" "<<_search_cam->width();    std::cout<<"roll: " << pos.pry.roll <<"\npitch: "<<pos.pry.pitch<<"\nyaw: "<<pos.pry.yaw<<"\n";
    ros::Duration timeout(10);
    if ( _cam_hand->set_position_quick(pos, timeout) < 0 )
    {
        ROS_ERROR("Unable to initialize search position. \n Please restart \
             at new location");
        _search_initialized = false;
        search(); //to ensure searching doesn't begin
    }
    _cam_hand->set_position_accurate(pos, timeout);
    _cam_hand->gripper->go_to(100);
    _gripping = false;
    _blob_area = 0;
 
}

void SearchControl::_search()
{
    //  _state == 0
    if(_object())
        _state = 1;
    else{
        ros::Duration timeout(5);
        _cam_hand->set_position_quick(geometry.home, timeout);
        _cam_hand->set_position_accurate(geometry.home, timeout*0.5);
    }
    
}
    
void SearchControl::_move_to_piece()
{
    //  _state == 1
    gv::Point err; //difference between centroid of object and center of image;
    err.x = _obj_loc.x - scene_width/2; 
    err.y = _obj_loc.y - scene_height/2;
//     err.print();
    gv::Point temp = abs(err);
    //temp.print();
    //allowable_error.print();
    if (temp < allowable_error)
    {
        _state = 2;
        std::cout<<"arrived at object";
        return;
    }
    // pixels -> pixels/pixel
    err.y /= scene_height;
    err.x /= scene_width; 
    
    gm::Pose pos = _cam_hand->endpoint_pose();
    //gm::Quaternion quat = pos.orientation;
    gv::Quaternion orient = pos.orientation;
    double yaw = toPRY(orient).yaw;
    _transform(err, yaw);
    err.z = 0;
    /*std::cout<<"y position: " << _obj_loc.y;
    std::cout<<"  y error: " << err.y << "\n";
    std::cout<<"x position: " << _obj_loc.x;
    std::cout<<"  x error: " << err.x << "\n";*/
    _state = 0;
    _endpoint_control(err);
    
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
    _state = 3; // Ignore validation for now, go straight to picking up
}

void SearchControl::test_transform(gv::Point pt, double yaw)
{
    double yaw_initial = PI/2;
    _dist_transform(pt, yaw + yaw_initial);
    pt.print();
    
}

JointVelocities jv_floor(JointVelocities a, JointVelocities b)
{
    // returns the lowest value of each a and b
    int asize = a.velocities.size();
    int bsize = b.velocities.size();
    int size = (asize < bsize ? asize : bsize);
    for (int i = 0 ; i < size; i++)
    {
        a.velocities[i] = (a.velocities[i] < b.velocities[i] ?  \
            b.velocities[i] : a.velocities[i]
        );
    }
    return a;
}

void SearchControl::_grab_piece()
{
    //  _state == 3
    double blobAreaInitial = _getBlobArea();
    _cam_hand->gripper->block = true;
    _cam_hand->gripper->go_to(0);
    cv::Mat scene = _search_cam->cvImage();
    std::vector<cv::Point> blob = _getBlob(scene);
    cv::RotatedRect rect = cv::minAreaRect(blob);
    cv::Point2f rect_points[4];
    rect.points(rect_points);
    for(int j = 0; j < 4; j++)
    {
        line(scene, rect_points[j], rect_points[(j+1)%4], cv::Scalar(255,0,0),1,8);
    }
    cv::imshow("Rotated Rectangle", scene);
    cv::waitKey(30);
    float rotation = rect.angle; //this is in DEGREES
   // float range = _cam_hand->gripper->range;
    gv::Pose pose = _cam_hand->endpoint_pose();
    gv::PRYPose prypose(pose);
    std::cout<<"\nPosition at Start of Grip";
    prypose.print();
    gv::PRYPose original(prypose);
    rotation = fabs(rotation);
    std::cout<<"rectangle size\n\tx: " << rect.size.width <<"\n\ty: " << rect.size.height;
    std::cout<<"\nrotation: " << rotation << std::endl;
    gv::Point mult(1,1,0);
    if (rotation < 45)
    {
        if (rect.size.width > rect.size.height){
            rotation = 90 + rotation;
//             mult = -1;
        }
        else
            rotation = rotation;
    }
    else
    {   
        if (rect.size.width > rect.size.height){
            rotation = -( 90 - rotation );
            mult.y = -1;
            //mult.x = -1;
        }
        else{
            rotation = rotation;
            //mult.x = -1;
        }
        
    }
    std::cout<<"new rotation: " << rotation << std::endl;
    rotation = rotation*PI/180; //to radians
    prypose.pry.yaw += rotation; // ALSO NEED TO REORIENT HAND
    gv::Point offset(0.02, -.03002, 0); // .03302, .0127 -> camera to endpoint center
    double yaw = prypose.pry.yaw - rotation;
    //yaw -= 1.57;   // account for starting yaw
    _dist_transform(offset, PI + yaw);
    std::cout<<"transformed offset";
    offset.print();
    //range -= _cam_hand->gripper->length();
    //std::cout<<"range: "<<range<<std::endl;
    prypose.position -= offset;
    std::cout<<"offset position";
    prypose.print();
    
     ros::Duration timeout(8);
    _cam_hand->set_position_accurate(prypose, timeout); // move gripper over object
    //_cam_hand->set_position_accurate(original, timeout); //for testing
    prypose.position.z = geometry.table_height + .04;
    prypose.print();
    ros::spinOnce();
    ros::Time start = ros::Time::now();
    _cam_hand->set_position_accurate(prypose, timeout);
    prypose.position.z -= .017;  //go accurately to 
    /*if ( _cam_hand->set_position_accurate(prypose, timeout) > 0 )
    { 
        _cam_hand->gripper->go_to(40);
        ros::Rate delay(2);
        delay.sleep();
    }
    else
        ROS_ERROR("Unable to grab a piece at the specified location");*/
    offset.x = 0.02;
    offset.y = 0;
    _dist_transform(offset, prypose.pry.yaw + PI);
    offset.z = .05;
    offset.print();
    _cam_hand->gripper->go_to(90);
    prypose.pry = toPRY(_cam_hand->endpoint_pose().orientation);
    prypose.position = _cam_hand->endpoint_pose().position;
    prypose.position -= offset;
    _drop_to_table(prypose);
    _cam_hand->gripper->go_to(0);
    ros::Rate delay(1);
    delay.sleep();
    if ( _cam_hand->set_position_quick(original, timeout) < 0 )
        ROS_ERROR("Return me to home position");
    delay.sleep();
    std::cout<<"FINAL BLOB AREA: "<<_getBlobArea()<<std::endl;
    if (_getBlobArea() > 2*blobAreaInitial){
        _gripping = true;
        std::cout<<"gripping set true from area\n";
    }
    _cam_hand->gripper->block = false;
    _state = 4;   //this should be 4 if he grabbed the piece
    std::cout<<std::endl<<"\t--- FINISHED GRABBING ---"<<std::endl;
}

void SearchControl::_deposit_piece()
{
    //  _state == 4
    if (!_cam_hand->gripper->state.gripping && !_gripping){
        ROS_ERROR("Failed to grab the piece");
        _state = 0;
        return;
    }
    _cam_hand->gripper->block = true;
    gv::PRYPose dropoff;
    dropoff.position = gv::Point(-.15, -.95, .15);
    dropoff.pry = gv::PRY(0, 3.14, 1.57);
    ros::Duration timeout(5);
    int moved = _cam_hand->set_position_quick(dropoff, timeout);
    if (moved < 0){
        ROS_ERROR("Unable to move to dropoff location");
        _state = 0;
    }
    if (!_cam_hand->gripper->state.gripping && !_gripping){
        ROS_ERROR("Dropped the piece on route");
        _state = 0;
        return;
    }
    ros::Duration quickTimeout(1.5);
    _cam_hand->set_position_accurate(dropoff, quickTimeout);
    _cam_hand->gripper->go_to(100);
    //return to search area
    ros::Rate delay(1);
    delay.sleep();
    _cam_hand->set_position_quick(geometry.home, timeout);
    _state = 0;
    _cam_hand->gripper->block = false;
    // If he drops the piece while moving to the box, turn his head light to RED
}

void SearchControl::_drop_to_table(gv::PRYPose prypose)
{
    JointPositions desired = _cam_hand->get_position(prypose);
    JointVelocities veloc = _cam_hand->get_velocities(desired);
    JointVelocities floor;
    JointVelocities temp;
    for (int i = 0; i < veloc.velocities.size(); i++)
    {
        floor.velocities.push_back(0.01);
    }
    floor.names = veloc.names;
    int iter = 0;
    double force_start = _cam_hand->endpoint_effort().force.z;
    force_start = fabs(force_start);
    double force = force_start;
    std::cout<<"force start: "<<force<<std::endl;
    int hit = 0;
    while (hit < 2 && ros::ok())
    {
        //_endpoint_control_accurate(offset);
        if (iter % 1000 == 0){
            temp = _cam_hand->get_velocities(desired);
            if (temp > floor)
                veloc = temp;
        }
        _cam_hand->set_joint_velocities(veloc);
        ros::spinOnce();
        force = _cam_hand->endpoint_effort().force.z;
        force = fabs(force);
        if (force < 11) // 10 newtons
            hit = 0;
        else{
            hit++;
            std::cout<<"force: "<<force<<std::endl;
        }   
        
    }
    std::cout << " I've hit the table" << std::endl;
    veloc *= -1.5;
    
    _cam_hand->set_joint_velocities(veloc); // go up just a little
}

bool SearchControl::_object()
{
    _reset_loc();
    cv::Mat scene = _search_cam->cvImage();
    whole_scene = scene;
    std::vector<cv::Point> blob = _getBlob(scene);
    if (!blob.empty())
    {
        _obj_loc = _get_centroid(blob);
        circle(whole_scene, _obj_loc, 10, cv::Scalar(255,0,0), 1, 8);
        cv::imshow("scene", whole_scene);
    }
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
    cv::Rect bounding_rect;
    std::vector<cv::Point> blob = _getBlob(scene);
    if (!blob.empty())
    {
        _obj_loc = _get_centroid(blob);
        _obj_loc.x += _roi.x;
        _obj_loc.y += _roi.y;
        circle(whole_scene, _obj_loc, 10, cv::Scalar(255,0,0), 1, 8);
        bounding_rect = _get_bounds(blob);
        bounding_rect.x += _roi.x;
        bounding_rect.y += _roi.y;
        _set_roi(bounding_rect, 2);
        
        rectangle(whole_scene, bounding_rect, cv::Scalar(255,0,0), 1, 8);
        cv::imshow("bounding box", whole_scene);
    }
    cv::waitKey(10);
    return true;
}

std::vector<cv::Point> SearchControl::_getBlob(cv::Mat& scene)
{
    // finds outline of the largest non-white object
    std::vector<cv::Point> temp(0,cv::Point(0,0));
    if (scene.empty())
    {
        //ROS_ERROR("Image to search is empty");
        return temp;
    }
    cv::Mat thresh;
    try {
        cv::cvtColor(scene, thresh, CV_BGR2GRAY);
    }
    catch (cv::Exception) {
        ROS_ERROR("Unable to convert image to greyscale");
        return temp;
    }
    cv::threshold(thresh, thresh,127,255,cv::THRESH_BINARY_INV); 
    
    std::vector< std::vector< cv::Point > > contours;
    std::vector< cv::Vec4i > hierarchy;
    int largest_area = 0;
    int largest_contour_index = 0;
    
    //cv::Canny(thresh, canny, 5, 250, 3);
    cv::imshow("thresholded", thresh);
    cv::findContours(thresh, contours, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
    
    int iters = contours.size();
    for(int i = 0; i < iters; i++)
    {
        double a = cv::contourArea(contours[i], false);
        if ( a > largest_area)
        {
            largest_area = a;
            largest_contour_index = i;
        }
    }
    cv::imshow("blob", thresh);
    if (largest_area < min_area)
    {
        return temp;
    }
    
    return contours[largest_contour_index];
}

bool SearchControl::_track_object()
{
    // Kalman filter
    cv::Point pt;
    cv::Point dim(scene_height, scene_width);
    cv::Point origin(0,0);
    if (!_track_state)
    {
        _track_state = true;
        pt = _obj_loc;
    }
    else
    {
        pt = _obj_loc - _prev_loc + _obj_loc - (_obj_loc - _predicted_loc ) \
         * _predictor.k;
    }
    _predicted_loc = pt;
    int d = _predictor.dimension;
    dim = dim - cv::Point(d/2,d/2);
    pt = constrain(pt, origin, dim);
    cv::Rect roi = _getBlobRoi(pt);
    
    cv::Mat scene = _search_cam->cvImage();
    whole_scene = scene;
    try
    {
        scene = scene(roi);
    }
    catch (cv::Exception)
    {
        ROS_ERROR("Your region of interest is poorly defined");
        return false;
    }
    std::vector<cv::Point> blob = _getBlob(scene);
    if (!blob.empty())
    {
        cv::Point2d centroid = _get_centroid(blob);
        _obj_loc.x = roi.x + centroid.x;
        _obj_loc.y = roi.y + centroid.y;
        rectangle(whole_scene, roi, cv::Scalar(255,0,0), 1, 8);
        cv::imshow("tracking", whole_scene);
    }
}

void SearchControl::_endpoint_control(gv::Point err)
{
    std::cout<<"endpoint position"<<std::endl;
    gv::PRYPose pose = geometry.home;
    pose.position = _cam_hand->endpoint_pose().position;
    pose.position -= err;
    pose.position.z = geometry.home.position.z-.1;
    JointPositions desired = _cam_hand->get_position(pose);
    if(desired.angles.empty())
        return;
    //v_print(desired.angles, "positions");
    //_cam_hand->set_joint_positions(desired);
    _cam_hand->set_velocities(desired);
}

void SearchControl::_endpoint_control_accurate(gv::Point err)
{
    std::cout<<"endpoint accurate"<<std::endl;
    gv::PRYPose pose;
    pose.pry = toPRY(_cam_hand->endpoint_pose().orientation);
    pose.position = _cam_hand->endpoint_pose().position;
    pose.position -= err;
    JointPositions desired = _cam_hand->get_position(pose);
    _cam_hand->set_velocities(desired);
}

double SearchControl::_getBlobArea()
{
    cv::Mat scene = _search_cam->cvImage();
    whole_scene = scene;
    std::vector<cv::Point> blob = _getBlob(scene);
    if (!blob.empty())
        return cv::contourArea(blob, false);   
    return -1;
}

cv::Rect SearchControl::_getBlobRoi(cv::Point2d pt)
{
    int dim = _predictor.dimension;
    cv::Point2d origin(0,0);
    cv::Point2d size(scene_height, scene_width);
    cv::Point2d x,y;
    x = pt - dim;
    y = pt + dim;
    x = constrain(x, origin, size);
    y = constrain(y, origin, size);
    return cv::Rect(x,y);
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

void SearchControl::set_predictor(Predictor params)
{
    if (params.k > 1 || params.k < 0)
    {
        ROS_ERROR("Gain for Kalman predictor must be in range [0-1]");
        return;
    }
    _predictor = params;
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
    _prev_loc = _obj_loc;
}

void SearchControl::_reset_roi()
{
    _roi.x = 0;
    _roi.y = 0;
    _roi.width = scene_width;
    _roi.height = scene_height;
}

void SearchControl::_dist_transform(gv::Point &err, double yaw)
{
    // transforms from local x,y coordinate system
    //  to global x,y
    double tempx, tempy;
    tempx = err.x;
    tempy = err.y;
    //yaw -= 3.14;
    // account for angle of hand (yaw)
    // 0 yaw has gripper "facing" backwards
    err.x = tempx*cos(yaw) - tempy*sin(yaw);
    err.y = tempx*sin(yaw) + tempy*cos(yaw); 
    
    // account for offset of camera (in m)
    /*
     */
}

void SearchControl::_transform(gv::Point &err, double yaw)
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