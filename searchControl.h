#include "searchgeometry.h"
#include "rosCamWrap.h"
#include "baxterLimb.h"

#ifndef SEARCH_CONTROL
#define SEARCH_CONTROL

class SearchControl
{
    // The purpose of this class is to implement a search control structure
    // So individual variables can be set up (limbs and cameras) before hand
    // Search Geometry should be set and checked to ensure valid choices are made
    
public:
    SearchControl(BaxterLimb*, BaxterLimb*, BaxterCamera*); //cam hand, other hand, cam
    
    //Use this to define geometry
    SearchGeometry geometry;
    
    double min_area;       // smallest area to be considered an object. default 100
    
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
    void _reset_loc();                                   //  sets _obj_loc to (-1,-1)
    cv::Point _get_centroid(std::vector< cv::Point >);   //  returns centroid of a set of points, calculated as mean of points
    
    //Search variables
    cv::Point2d _obj_loc;                                //  location of found object in _object()
    
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
    min_area = 100;
}

void SearchControl::search()
{
    if(!_search_initialized)
        _init_search();
    
    switch(_state)
    {
        case 0:
            //  Searching, move around in bounding box
            //  obj found ? _state = 1 : _state = _state
            _search();
            break;
        case 1:
            //  Found something, move over
            //  obj in bounds ? move over, _state = 2 : state = 0
            _move_to_piece();
            break;
        case 2:
            //  Verify piece
            //  known piece ? _state = 3 : _state = 0
            _verify_piece();
            break;
        case 3:
            //  Grab piece
            //  piece grabbed ? _state = 4 : _state = 3
            _grab_piece();
            break;
        case 4:
            //  Move over deposit box, deposit piece
            //  move over, piece released -> _state = 0
            _deposit_piece();
            break;
        default:
            //Shouldn't happen
            ROS_ERROR("_state has been set to an unknown value");
            break;
    }
    ros::spinOnce();
}

void SearchControl::_init_search()
{
    _search_initialized = true;
    _state = 0;
    PRYPose pos;
    pos.point.x = -.1;
    pos.point.y = -1;
    pos.point.z = .2;
    pos.pry.roll = -3.14;
    pos.pry.pitch = -.1;
    pos.pry.yaw = 1.57;/*
    std::cout<<"roll: " << pos.pry.roll <<"\npitch: "<<pos.pry.pitch<<"\nyaw: "<<pos.pry.yaw<<"\n";
    Pose pos2;
    pos2.point = pos.point;
    pos2.quaternion = toQuat(pos.pry);
    std::cout<<"w: "<<pos2.quaternion.w<<"\nx: "<<pos2.quaternion.x<<"\n";
    std::cout<<"y: "<<pos2.quaternion.y<<"\nz: "<<pos2.quaternion.z<<"\n";
    pos.pry = toPRY(pos2.quaternion);
    std::cout<<"roll: " << pos.pry.roll <<"\npitch: "<<pos.pry.pitch<<"\nyaw: "<<pos.pry.yaw<<"\n";
    */ros::Duration timeout(10);
    _cam_hand->set_position(pos, true, timeout);
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
    double height, width;
    height = _search_cam->height();
    width = _search_cam->width();
    err.z = 0;
    err.x = _obj_loc.x - width/2; // + camera_offset.x
    err.y = _obj_loc.y - height/2; // + camera_offset.y
    
    //transform between pixels and real distances. figure this out as
    //function of distance
    err.y /= -1000;
    err.x /= -1000;
    
    std::cout<<"y position: " << _obj_loc.x;
    std::cout<<"  y error: " << err.y << "\n";

    //search for object again
    _state = 0;
    _cam_hand->endpoint_control(err);
}

void SearchControl::_verify_piece()
{
    //  _state == 2
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
    cv::Mat thresh, canny;
    
    cv::cvtColor(scene, thresh, CV_BGR2GRAY);
    cv::threshold(thresh, thresh,127,255,cv::THRESH_BINARY_INV); 
    
    std::vector< std::vector< cv::Point > > contours;
    std::vector< cv::Vec4i > hierarchy;
    int largest_area = 0;
    int largest_contour_index = 0;
    cv::Rect bounding_rect;
    bounding_rect.x = -1;
    bounding_rect.y = -1;
    
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
        imshow("Found Object?", thresh);
        return false;
    }
    
    _obj_loc = _get_centroid(contours[largest_contour_index]);
    circle(thresh, _obj_loc, 10, cv::Scalar(255,0,0), 1, 8);
    //bounding_rect = cv::boundingRect(contours[largest_contour_index]);
    //rectangle(thresh, bounding_rect, cv::Scalar(255,0,0),1,8,0);
    //std::cout<<"found "<<countours.size()<<" contours.\n";
    imshow("Found Object?", thresh);
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

void SearchControl::_reset_loc()
{
    _obj_loc.x = -1;
    _obj_loc.y = -1;
}

#endif