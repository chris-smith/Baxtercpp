#include "searchgeometry.h"
#include "rosCamWrap.h"
#include "baxterGripper.h"
#include "baxterLimb.h"

class SearchControl
{
    // The purpose of this class is to implement a search control structure
    // So individual variables can be set up (limbs and cameras) before hand
    // Search Geometry should be set and checked to ensure valid choices are made
    
public:
    SearchControl(BaxterLimb, BaxterLimb, BaxterCamera&); //cam hand, other hand, cam
    
    //Use this to define geometry
    SearchGeometry geometry;
    
    //main functions
    void search();         //main loop that searches for pieces, adjusts _state as necessary

private:
    SearchControl(); //no access to default constructor
    
    BaxterLimb* _cam_hand;
    BaxterLimb* _manip_hand;
    BaxterCamera _search_cam;
    
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
};

SearchControl::SearchControl()
{
    ROS_ERROR("This object can not be declared without passing it arguments. \
        The default constructor on this object is private. \
        You should never see this error message.");
}

SearchControl::SearchControl(BaxterLimb a, BaxterLimb b, BaxterCamera& cam)
{
    _cam_hand = &a;
    _manip_hand = &b;
    _search_cam = cam;
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
}

void SearchControl::_search()
{
    //  _state == 0
    PRYPose new_position;    
}
    
void SearchControl::_move_to_piece()
{
    //  _state == 1
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