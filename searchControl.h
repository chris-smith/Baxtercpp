#include "backgroundController.h" // includes baxterLimb.h
#include "searchgeometry.h"
#include "objectClassifier.h"
#include "nxtKit.h"

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
    SearchControl(BaxterLimb*, BaxterLimb*, BaxterCamera*, BaxterCamera*); //cam hand, other hand, search cam, control cam
    ~SearchControl();
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
    void swap_hands();                          // starts to search with other hand
    
    
private:
    SearchControl(); //no access to default constructor
    
    BaxterLimb* _right_hand;
    BaxterLimb* _left_hand;
    //BackgroundController* _right_controller;
    BaxterCamera* _right_cam;          // used to find objects in scene
    BaxterCamera* _left_cam;         // used to help control positioning of _right_cam
    ObjectClassifier* _classifier;
    NxtKit* _kit;
    //cv::SimpleBlobDetector* _blob_detector;
    
    ros::AsyncSpinner* _spinner;
    
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
    bool _object();                                      //  looks for an object in _right_cam
    bool _object(cv::Rect);                              //  looks for object within bounds of rectangle
    bool _track_object();
    void _reset_loc();                                   //  sets _obj_loc to (-1,-1)
    void _reset_roi();                                   //  sets _roi to (-1,-1)
    void _set_roi(cv::Rect, double);                     //  set _roi based on _obj_loc and area of obj
    cv::Point _get_centroid(std::vector< cv::Point >);   //  returns centroid of a set of points, calculated as mean of points
    void _transform(gv::Point &, double);                    //  transforms dimensionless error to real distance. accounts for hand rotation
    void _transform(gv::Point &, double, double);
    cv::Rect _get_bounds(std::vector<cv::Point>);        //  returns a bounding rectangle for the contour
    std::vector<cv::Point> _corners(cv::Rect);
    std::vector<cv::Point> _increase_bounds(std::vector<cv::Point>, cv::Point);
    cv::Rect _restrict(std::vector<cv::Point>, double,double);     //  returns rectangle enclosed in size of image
    cv::Rect _getBlobRoi(cv::Point2d pt);
    std::vector<cv::Point> _getBlob(cv::Mat&);            //   return contour of largest blob in image
    std::vector<cv::Point> _getBlob(cv::Mat&, cv::Rect);
    void _endpoint_control(gv::Point);
    void _endpoint_control(gv::Point, double);
    void _endpoint_control_accurate(gv::Point err);
    void _lower(double);                                      //  lowers down to table keeping object in center of camera
    void _dropToTable(JointPositions);                     // blind drop until force increase
    double _getBlobArea();
    cv::Point2f _best_gripping_location(std::vector<cv::Point>);
    bool _on_edge(std::vector<cv::Point> blob);
    void _remove_grippers();            // finds grippers in test image for removal later
    bool _breakLower(cv::RotatedRect, double);
    
    //Search variables
    cv::Point2d _obj_loc;                                //  location of found object in _object()
    cv::Point2d _prev_loc;                               //   for use with kalman filter
    cv::Point2d _predicted_loc;                          //   predicted location from Kalman filter
    Predictor _predictor;
    bool _tracking;                                      //   tracking object if true
    bool _track_state;                                   //   initializes tracker
    cv::Rect _roi;                                       //   subset of image where object may be found
    cv::Rect _bounding_rect;
    cv::RotatedRect _rotated_rect;
    double scene_height;
    double scene_width;
    bool _gripping;
    double _blob_area;
    cv::Mat _gripper_mask;
    cv::Mat _bgra_gripper_mask;
    std::string _found_piece;
    gv::Point _last_err;                                // last error in _move_to_piece
};

SearchControl::SearchControl()
{
    ROS_ERROR("This object can not be declared without passing it arguments. \
        The default constructor on this object is private. \
        You should never see this error message.");
}

SearchControl::SearchControl(BaxterLimb* a, BaxterLimb* b, BaxterCamera* cam_a, BaxterCamera* cam_b)
{
    _right_hand = a;
    _left_hand = b;
    _right_cam = cam_a;
    _left_cam = cam_b;
    //_right_controller = new BackgroundController(_right_hand);
    ////_right_controller->start();
    _classifier = new ObjectClassifier("/home/ceeostud2/Pictures/ObjectTemplates/", 300);
    _kit = new NxtKit( cv::RotatedRect(cv::Point2f(0.58,-0.61), cv::Size2f(.38,.275), -35 ), .1);
    _kit->show_containers();
    cv::waitKey(3000);
    //_spinner = new ros::AsyncSpinner(2);
    //_spinner->start();
    _state = 0;
    _reset_loc();
    _reset_roi();
    min_area = 75;
    scene_height = _right_cam->height();
    scene_width = _right_cam->width();
    _tracking = false;
    _track_state = false;
    _predictor.dimension = 20;
    _predictor.k = 0.5;
}

SearchControl::~SearchControl()
{
    /*delete _right_hand;
    delete _left_hand;
    delete _right_cam;
    delete _left_cam;*/
    //_spinner->stop();
    delete _classifier;
    //delete _right_controller;
    delete _kit;
    //delete _blob_detector;
}

void SearchControl::swap_hands()
{
    // swap limbs
    std::swap(_right_hand, _left_hand);
    // swap cameras
    std::swap(_right_cam, _left_cam);
    
}

void SearchControl::search()
{
    // this is the main function for SearchControl
    //  calls functions based on internal state
    if(!_search_initialized)
        _init_search();
    
    if (!ros::ok())
        return;
    scene_height = _right_cam->height();
    scene_width = _right_cam->width();
    ros::Rate r(20);
    //_state = 2;
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
            //  obj in bounds ? _state = 2 : state = 0
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
            _state = 0; 
            break;
    }
    std::cout<<std::endl;
    ros::spinOnce();
}

cv::SimpleBlobDetector::Params _blob_params()
{
    // Setup parameters for OpenCV's simple blob detector
    //   Using SBD currently crashes entire program
    cv::SimpleBlobDetector::Params params;
    params.minDistBetweenBlobs = 50.0f;
    params.filterByInertia = false;
    params.filterByConvexity = false;
    params.filterByColor = false;
    params.blobColor = 20;
    params.filterByCircularity = false;
    params.filterByArea = true;
    params.minArea = 10.0f;
    params.maxArea = 500.0f;
    return params;
}

void SearchControl::_init_search()
{
    // initalizes a number of search parameters
    _search_initialized = true;
    _state = 0;
    allowable_error = gv::Point(10,10,1); // need the 1 so err < allow is true
    if (geometry.home == gv::RPYPose(0))
    {
        geometry.home = _right_hand->endpoint_pose();
        //geometry.home.position = pose.position;
        //geometry.home.rpy = toPRY(gv::Quaternion(pose.orientation));
    }
    gv::RPYPose pos = geometry.home;
    if (geometry.table_height == 0 || geometry.height_offset == 0)
        pos.position.z = .15;
    else
        pos.position.z = geometry.table_height + geometry.height_offset;
    geometry.home.position.z = pos.position.z;
    //std::cout<<"height: "<<pos.position.z<<"\n";
    //std::cout<<_right_cam->height()<<" "<<_right_cam->width();    std::cout<<"roll: " << pos.rpy.roll <<"\npitch: "<<pos.rpy.pitch<<"\nyaw: "<<pos.rpy.yaw<<"\n";
    ros::Duration timeout(10);
   //   COMMENTED OUT WHILE TESTING IMAGE RECOGNITION
//     if ( _right_hand->set_service_position_quick(pos, timeout) < 0 )
//     {
//         ROS_ERROR("Unable to initialize search position");
//         _search_initialized = false;
//         search(); //to ensure searching doesn't begin
//     }
    // if arm doesn't receive a command for 0.5 seconds, will disable
    //   This is controlled ON Baxter, not by me
    _right_hand->set_command_timeout(0.5);
    pos.print("Starting Endpoint Position");
    JointPositions jp = _right_hand->get_simple_positions(pos.position, 0);
    //_right_controller->set( jp );
    pos.position.z += .05;
    //_right_hand->set_simple_positions(pos.position, 0, timeout);
    //_right_hand->set_simple_positions(_kit->origin(), 0, timeout);
    pos.position.z -= .05;
    _right_hand->set_simple_positions(pos.position, 0, timeout);
    _right_hand->gripper->open();
    ros::spinOnce();
    _remove_grippers();
    //std::cout<<"Grippers Masked\n";
    //_blob_detector = new cv::SimpleBlobDetector();
    //_blob_detector->create("SimpleBlob");
    /*
 *  x: -0.22869011299
    y: -0.714390159574
    z: -0.0736401235418
    
    x: -0.37282466562
    y: -0.514911105068
    z: -0.0697321340386
    */
    //cv::RotatedRect nxtkit(cv::Point2f(-.22869,-.71439),cv::Size2f(.38,.27),-54.15);
    //_kit = new NxtKit(nxtkit,-.065);
    _gripping = false;
    _blob_area = 0; 
    _right_hand->set_joint_position_speed(.1);
    _right_hand->wait_for_arrival( jp, ros::Duration(5) );
    gv::RPYPose( _right_hand->endpoint_pose() ).position.print("Current Endpoint Position");
    //_right_controller->stop();
    std::cout<<"Search Initialized\n";
}

void SearchControl::_remove_grippers()
{
    // This creates a mask for the open gripper state.
    //   Should possibly be extended to build mask for closed gripper state??
    
    // if no objects are found, grippers must not be in scene
    if (!_object())
        return;
    
    // move to open position
    _right_hand->gripper->block = true;
    _right_hand->gripper->go_to(100);
    std::vector<cv::Point> blob;
    cv::Mat scene = _right_cam->cvImage();
    if (scene.empty()){
        // This could cause an infinite loop if the cameras 
        //  aren't working correctly
        ROS_ERROR("Empty scene :/");
        _remove_grippers();
    }
    cv::Mat mask;
    int type = cv::MORPH_ELLIPSE;
    int sz = 2;
    cv::Mat element = getStructuringElement( type,
                                             cv::Size(2*sz+1, 2*sz+1),
                                             cv::Point(sz, sz) );
    cv::Mat thresh;
    
    //  convert to greyscale
    try {
        cv::cvtColor(scene, thresh, CV_BGR2GRAY);     
    }
    catch (cv::Exception) {
        ROS_ERROR("Unable to convert image to greyscale");
        return;
    }
    // threshold and dilate image
    cv::threshold(thresh, thresh,40,255,cv::THRESH_BINARY_INV); 
    cv::dilate(thresh, mask, element);   
    cv::dilate(mask, mask, element);
    
    // set mask
    _gripper_mask = mask;
    if(_right_hand->gripper->state.position > 90)
        ROS_INFO("Set mask for open position");
    else
        ROS_INFO("Set mask for closed position");
    
    // show what mask looks like
    cv::Mat zeros;
    cv::cvtColor(mask, zeros, CV_GRAY2BGRA);
    _bgra_gripper_mask = zeros;
    cv::add(zeros, scene, scene);
    cv::imshow("mask add", scene);
    
    cv::waitKey(50);
}

void SearchControl::_search()
{
    //  _state == 0
    //    Look for an object. If it exists, state = 1;
    //                        else,         keep looking
    if(_object()) {
        _state = 1;
        _right_hand->reset_clock();
    }
    else{
        cv::imshow("scene", _right_cam->cvImage());
        cv::waitKey(30);
        ros::spinOnce();
        //_right_hand->exit_control_mode();
        // Find new location to search
    }
    
}
    
void SearchControl::_move_to_piece()
{
    //  _state = 1
    //    Move over to piece
    _right_hand->set_joint_position_speed(.08);
    gv::Point err; 
    // err is pixel difference between centroid of object and center of image;
    err.x = _obj_loc.x - scene_width/2; 
    err.y = _obj_loc.y - scene_height/2;
    //std::cout<<"error"<<err.x<<" "<<err.y<<"\n";
    gv::Point temp = abs(err);
    if (temp < allowable_error)
    {
        // if at piece, stop moving, _state = 2;
        _state = 2; //state = 2
        std::cout<<"arrived at object";
        //_right_controller->stop();
        _right_hand->clear_integral();
        _right_hand->exit_control_mode();
        return;
    }
    
    // convert pixels -> pixels/pixel
    err.y /= scene_height;
    err.x /= scene_width; 
    //err.print("Non dimensionalized error");
    double rotation = _rotated_rect.angle; //this is in DEGREES
    if (_rotated_rect.size.width < _rotated_rect.size.height){
        rotation = 90 - rotation;
    }
    ros::spinOnce();
    gm::Pose pos = _right_hand->endpoint_pose();
    //gm::Quaternion quat = pos.orientation;
    gv::Quaternion orient = pos.orientation;
    double yaw = gv::RPY(orient).yaw;
    double height = pos.position.z;
    //std::cout<<"yaw: "<<yaw<<"\n";
    // transform dimensionless error into approx distance error
    
    _transform(err, yaw, height-geometry.table_height);
    //_last_err.print("last error");
    // pseudo integral term
    //  if err*last_err > 0, steady state error is building
    if (err.x*_last_err.x > 0)
        err.x += _last_err.x*.5;
    if  (err.y*_last_err.y > 0)
        err.y += _last_err.y*.5;
    
    //err.print("error");
    //cv::waitKey(500);
    //err.z = 0.005;
    /*std::cout<<"y position: " << _obj_loc.y;
    std::cout<<"  y error: " << err.y << "\n";
    std::cout<<"x position: " << _obj_loc.x;
    std::cout<<"  x error: " << err.x << "\n";*/
    
    // set _state to 0 to look for piece again next time
    //   This should change to actively track this piece
    _state = 0;
    
    err.z = height-geometry.home.position.z;
    _endpoint_control(err,0);
    _last_err = err;
}

void SearchControl::_verify_piece()
{
    //  _state == 2
    //   This should reference _classifier to figure out which piece is in view
    //     Should verify that piece exists in NXTKit
    //   While testing, may ask for piece name, rather than referencing classifier

    std::cout<<"What piece is this?\n";
    std::cin>>_found_piece;
    
    _state = 3; // Ignore validation for now, go straight to picking up
    if ( !_kit->contains(_found_piece) ){
        _state = 0;
        ROS_ERROR( "%s is an unknown piece", _found_piece.c_str() );
    }
}

void SearchControl::_grab_piece()
{
    //  _state == 3
    //   This tracks the piece down until ~1cm over piece
    //   Then blind drops, checks if piece has been grabbed
    
    double blobAreaInitial = _getBlobArea();
    // open gripper
    _right_hand->gripper->block = true;
    _right_hand->gripper->go_to(100);
    std::vector<cv::Point> blob = _getBlob(scene);
    
    // Lower arm down to 2cm over table
    _lower(geometry.table_height+.03);
    gv::Point pos;
    pos = _right_hand->endpoint_pose().position;
    pos.z = geometry.table_height - 0.01;//-.012;
    ros::Duration tout(5);
    JointPositions jp = _right_hand->joint_positions();
    double w2_ = jp.at("_w2");
    
    // Blind Drop
    std::cout<<"Final drop to height "<<pos.z<<"\n";
    JointPositions desired = _right_hand->get_simple_positions(pos, w2_);
    _dropToTable(desired);
    pos.z += 0.005;
    tout = ros::Duration(1);
    // move up just a little bit
    _right_hand->set_simple_positions(pos, w2_, tout);
    
    //  Grip object, pause
    _right_hand->gripper->go_to(5);
    ros::Duration pause(1);
    pause.sleep();
    ros::spinOnce();
    
    // Check if holding object -- unreliable
    bool gripped = _right_hand->gripper->state.gripping;
    if (gripped) {
        ROS_INFO("GRABBED PIECE CORRECTLY");
        _right_hand->set_joint_position_speed(0.15);
        pos.z = _kit->height() + .1;
        pos.print("desired position");
        _right_hand->set_simple_positions(pos, 0.0, ros::Duration(10) );
        //_right_hand->gripper->go_to(100);
        gv::RPYPose(_right_hand->endpoint_pose()).position.print("actual position");
        // set internal state. 
        //   4 -> deposit piece 
        //   0 -> look for piece
        _state = 4;
    }
    else {
        ROS_INFO("Failed to grab piece");
        _state = 0;
        _right_hand->gripper->go_to(100);
        _right_hand->set_simple_positions(geometry.home.position, 0);
    }
    
    return;
}

void SearchControl::_deposit_piece()
{
    //  _state == 4
    //  Deposit grabbed piece in NxtKit
    
    // Get Coordinates of dropoff
    cv::RotatedRect container = _kit->get_coordinates(_found_piece);
    if(container.size == cv::Size2f(-1,-1)){
        ROS_ERROR("The piece [%s] does not exist in this kit", _found_piece.c_str());
        _state = 0;
        return;
    }
    std::cout<<"container :" << container.center <<"\n--------\n";
    _right_hand->gripper->block = true;
    //cv::Point dropoff_pt = container.center;
    gv::RPYPose dropoff;
    gv::Point dropoff_pt;
    dropoff_pt.x = container.center.x;
    dropoff_pt.y = container.center.y;
    dropoff_pt.z = _kit->height();
    //dropoff.position = container.center;
    dropoff_pt.print("Deposit location");
    double w2 = PI * _kit->angle() / 180;
    w2 += PI/2;
    _right_hand->set_simple_positions( dropoff_pt, w2, ros::Duration(15) );
    //gv::RPYPose(_right_hand->endpoint_pose()).position.print("actual deposit");
    ros::Duration(1).sleep();
    _right_hand->gripper->go_to(100);
    ros::Duration(1).sleep();
    _right_hand->set_simple_positions( geometry.home.position, 0, ros::Duration(5) );
    _state = 0;
    return;
    dropoff.position.z = _kit->height() + .005;
    dropoff.rpy = gv::RPY(0, 3.14, 1.57);
    ros::Duration timeout(5);
    int moved = _right_hand->set_service_position_quick(dropoff, timeout);
    if (moved < 0){
        ROS_ERROR("Unable to move to dropoff location");
        _state = 0;
    }
    if (!_right_hand->gripper->state.gripping && !_gripping){
        ROS_ERROR("Dropped the piece on route");
        _state = 0;
        return;
    }
    ros::Duration quickTimeout(1.5);
    _right_hand->set_simple_positions(dropoff.position, 0.0, quickTimeout);
    _right_hand->gripper->go_to(100);
    //return to search area
    ros::Rate delay(1);
    delay.sleep();
    _right_hand->set_service_position_quick(geometry.home, timeout);
    _state = 0;
    _right_hand->gripper->block = false;
    // If he drops the piece while moving to the box, turn his head light to RED
}

bool _in_range(const gv::Point &err, const double &thresh)
{
    bool ret = true;
    ret = ret && err.x < thresh;
    ret = ret && err.y < thresh;
    ret = ret && err.z < thresh;
    return ret;    
}

void SearchControl::_lower(const double stop_height)
{
    // tracks the object down until it reaches stop_height
    //_right_controller->reset();
    _right_hand->set_joint_position_speed(.03);
    float rotation = 1;
    cv::RotatedRect rect;
    cv::Mat scene;
    cv::Point2d centroid;
    std::vector<cv::Point> blob;
    gv::Point error(1);
    gv::Point integral_error(0);
    gv::RPYPose pose;
    JointPositions jp;
    ros::Duration timeout(.01);
    jp.names.push_back(_right_hand->name()+"_w2");
    double height = _right_hand->endpoint_pose().position.z;
    double w2 = 0;
    double area, rectArea, percentArea, rectRatio;
    double ki = 0.8;    // integral gain
    ros::Time last = ros::Time::now();
    ros::Duration dt;
    while ((!(error.abs() < 0.001) || (fabs(rotation) > 0.01)) && ros::ok())
    {
        scene = _right_cam->cvImage();
        blob = _getBlob(scene);
        //_best_gripping_location(blob);
        dt = ros::Time::now() - last;
        last = ros::Time::now();
        try{
            if(blob.empty())
                continue;
            //centroid = _best_gripping_location(blob);
            //centroid = _get_centroid(blob);
            area = cv::contourArea(blob, false);
            rect = cv::minAreaRect(blob);
            rectArea = rect.size.height*rect.size.width;
            percentArea = area/rectArea;
            //std::cout<<"Percent Area: " << percentArea<<"\n";
            rectRatio = rect.size.height/rect.size.width;
            // assume object has a regular shaped
            //  -- square, circular, rectangular, etc
            //  so we can use the bounding rectangle's center
            centroid = rect.center;
            if ( (rectRatio > 0.9) && (rectRatio < 1.1) )
            {
                // object is pretty square, don't rotate
                std::cout<<"object is square\n";
                rect.angle = 0;
            }
            else if (percentArea < .75)
            {
                // the object isn't largely rectangular
                std::cout<<"object is irregularly shaped\n";
            }
            circle(scene, centroid, 10, cv::Scalar(-1), 1, 8);
            error.x = centroid.x - scene_width/2; 
            error.y = centroid.y - scene_height/2;
            error.z = 0;
            //error.print("pixel error");
            error.y /= scene_height;
            error.x /= scene_width; 
            // maybe too noisy?
            //error.y *= 1.05;
            //integral_error += error*toSec(dt.nsec);
            //integral_error *= ki;
            //integral_error.print("integral error");
            //error += integral_error*ki;
            //error.print("error");
            cv::Point2f rect_points[4];
            rect.points(rect_points);
            for(int j = 0; j < 4; j++)
            {
                line(scene, rect_points[j], rect_points[(j+1)%4], cv::Scalar(255,0,0),1,8);
            }
            cv::imshow("scene", scene);
            cv::waitKey(50);
            
            // Rotation angle is always negative
            rotation = rect.angle; //this is in degrees
            //std::cout<<"rotation: "<<rotation<<"\n";
            
            // OBJECT HEIGHT IS GREATER THAN WIDTH
            // FOR OBJECTS THAT LOOK LIKE  \ \
            //                             \_\
            //
            // OBJECT WIDTH IS GREATER THAN HEIGHT
            // FOR OBJECTS THAT LOOK LIKE /  /
            //                           /__/
            
            //std::cout<<"rectangle size\n\tx: " << rect.size.width <<"\n\ty: " << rect.size.height;
            
            // Get minimum _w2-rotation angle
            if (rotation == 0)
                ; //do nothing
            else if (rotation < 45)
            {
                if (rect.size.width > rect.size.height)
                    rotation = 90 + rotation;
            }
            else
            {   
                if (rect.size.width > rect.size.height)
                    rotation = -( 90 - rotation );
            }
            rotation = rotation*PI/180; //to radians
            ros::spinOnce();
            pose = _right_hand->endpoint_pose();
            jp = _right_hand->joint_positions();
            w2 = jp.at("right_w2") + rotation;
            double yaw = pose.rpy.yaw;
            height = pose.position.z;
            _transform(error, yaw, height-geometry.table_height);
            //error.print("transformed error");
            //std::cout<<"yaw: "<<yaw<<" height: "<<height<<"  stop height: "<<stop_height<<"\n";
            error.z = 0;//height - stop_height;
            if(height > stop_height)
                error.z = .05;
            //std::cout<<"rotation: "<<rotation<<"\n";
            _endpoint_control(error, w2); //takes in error, w2 angle
            if ( _breakLower(rect, height-geometry.table_height) ) {
                if ( fabs(rotation) < 0.02 ) {
                    std::cout<<"break out of lower loop";
                    break;
                }
            }
            ros::spinOnce();
            pose = _right_hand->endpoint_pose();
        }
        catch(cv::Exception e){
            _right_hand->exit_control_mode();
            ROS_ERROR("OpenCV error: %s",e.what());
        }
        //error.z = pose.position.z - geometry.table_height;    
    }
    //_right_controller->stop();
    _right_hand->exit_control_mode();
    std::cout<<"LOWER COMPLETE\n";
    //pose.print();
}

bool SearchControl::_breakLower(cv::RotatedRect rect, double height)
{
    // tries to determine if the object bounding by rect is close enough to the
    // center of the image given the height over table
    std::cout<<"height "<<height<<"\n";
    
    // only break if at most 5cm above table
    if (height > .03)
        return false;
    
    double x = scene_width / 2;
    double y = scene_height / 2;
    double xDiff = x - rect.center.x;
    double yDiff = y - rect.center.y;
    double dist = std::sqrt( pow( xDiff, 2) + pow( yDiff, 2) );
    // false if pixel distance is too large
    std::cout<<"dist "<<dist<<" ";
    if (dist > 40)
        return false;
    else if (dist < 5)
        return true;
    // grippers are fairly robust to yDiff
    if (3*xDiff < rect.size.width)
        return true;
    
    // come up with some tunable algorithm for scenarios where
    //  we can blind drop -- skinny pieces can be off in x more
    double ratio = dist/height; // height in m
    std::cout<<" ratio "<<ratio<<"\n";
    return false;
}

void SearchControl::_dropToTable(JointPositions jp)
{
    ros::Duration(1).sleep();
    double speed = 0.01;
    //_right_hand->set_joint_position_speed();
    ros::spinOnce();
    const geometry_msgs::Vector3 startForce(_right_hand->endpoint_effort().force);
    geometry_msgs::Vector3 endpointForce;
    ros::Rate r(100);
    gv::RPYPose pose;
    double height;
    while ( ros::ok() )
    {
        //_right_hand->set_joint_position_speed(speed);
        //_right_hand->set_joint_positions(jp);
        _right_hand->set_velocities(jp);
        ros::spinOnce();
        endpointForce = _right_hand->endpoint_effort().force;
        //std::cout<<endpointForce<<"\n";
        pose = _right_hand->endpoint_pose();
        //pose.position.print("position");
        // if it's reached the table from height
        height = pose.position.z - geometry.table_height;
        if (height < 0.003)
        {
            std::cout<<"I should stop from height\n";
            break;
        }
        // it's running into something -- z force is negative
        if (endpointForce.z < -15)
        {
            std::cout<<"I should stop from force\n";
            //break;
        }
        std::cout<<height<<","<<pose.position.x<<","<<pose.position.y;
        std::cout<<","<<pose.position.z<<"\n";
        // slowly decrease speed
        speed /= 1.05;
        r.sleep();
    }
    ros::spinOnce();
    _right_hand->exit_control_mode();
}

bool SearchControl::_object()
{
    // looks for an object within the entire scene
    _reset_loc();
    cv::Mat scene = _right_cam->cvImage();
    whole_scene = scene;
    std::vector<cv::Point> blob = _getBlob(scene);
    if (!blob.empty())
    {
        //_classifier->_is_circle(blob);
        _bounding_rect = _get_bounds(blob);
        _rotated_rect = cv::minAreaRect(blob);
        _obj_loc = _rotated_rect.center;
        //_obj_loc = _get_centroid(blob);
        circle(whole_scene, _obj_loc, 10, cv::Scalar(255,0,0), 1, 8);
        rectangle(whole_scene, _bounding_rect, cv::Scalar(0,255,0),1,8);
        cv::imshow("scene", whole_scene);
    }
    else
        return false;
    cv::waitKey(30);
    return true;
}

bool SearchControl::_object(cv::Rect bounds)
{
    // search for object within the specified bounds
    _reset_loc();
    cv::Mat scene = _right_cam->cvImage();
    whole_scene = scene;
    std::vector<cv::Point> blob = _getBlob(scene, bounds);
    if (!blob.empty())
    {
        _bounding_rect = _get_bounds(blob);
        // account for offset of search area
        _bounding_rect += bounds.tl();
        _rotated_rect = cv::minAreaRect(blob);
        _obj_loc = _get_centroid(blob);
        _obj_loc.x += bounds.tl().x;
        _obj_loc.y += bounds.tl().y;
        circle(whole_scene, _obj_loc, 10, cv::Scalar(255,0,0), 1, 8);
        rectangle(whole_scene, _bounding_rect, cv::Scalar(0,255,0),1,8);
        cv::imshow("scene", whole_scene);
    }
    else
        return false;
    cv::waitKey(30);
    return true;
}

cv::Point2f SearchControl::_best_gripping_location(std::vector<cv::Point> blob)
{
    // attempts to determine the best gripping location for provided blob
    //  if blob is relatively square (non convex), will return the center
    //  of blob's bounding rectangle
    
    int max_height = 0;
    int min_width = scene_width;
    double area = cv::contourArea(blob, false);
    cv::RotatedRect rect = cv::minAreaRect(blob);
    double rectArea = rect.size.height*rect.size.width;
    double percentArea = area/rectArea;
    std::cout<<"Percent Area: " << percentArea<<"\n";
    double rectRatio = rect.size.height/rect.size.width;
    if ( (rectRatio > 0.9) && (rectRatio < 1.1) )
    {
        // object is pretty square
    }
    if (percentArea > .75)
    {
        // the object is fairly rectangular, just grab it at the default location
        std::cout<<"object is pretty square\n";
        return rect.center;
    }
    // temporary while working on grip location algorithm
    return rect.center;
    
    
    std::vector<cv::Vec4i> defects;
    std::vector<int> hull;
    convexHull(blob, hull, false, false);
    if (hull.size() > 2)
        convexityDefects(blob, hull, defects);
    for(int i = 0; i < defects.size(); i++)
    {
        std::cout<<"convexity: "<<defects[i].depth<<"\n";
    }
    return cv::Point2f();
}

bool SearchControl::_on_edge(std::vector<cv::Point> blob)
{
    // determines if the blob is on the edge of the image
    //  image taken from _right_cam
    int size = blob.size();
    cv::Point a, b;
    for(int i = 0; i < size; i++)
    {
        a = blob[i];
        b = blob[(i+1)%size];
        b = b-a;
        if (b.x == 0)
        {
            //  horizontal line
            if (a.x == 0 || a.x == scene_height)
                return true;
        }
        else if(b.y == 0)
        {
            //   vertical line
            if(a.y == 0 || a.y == scene_width)
                return true;
        }
    }
}

/*bool SearchControl::_on_edge(std::vector<cv::Point> blob)
{
    char state = 0;
    for(int i = 0; i < blob.size(); i++)
    {
        if (blob[i].x == 0)
        {
            // object on left edge of camera
            state = 1;
        }
        if(blob[i].x == scene_width)
        {
            // object on right edge of camera
            state += 2;
        }
        if(blob[i].y == 0)
        {
            // object on top edge of camera
            state += 4;
        }
        if(blob[i].y == scene_height)
        {
            // object on bottom edge of camera
            state += 8;
        }
    }
}*/

std::vector<cv::Point> SearchControl::_getBlob(cv::Mat& scene, cv::Rect search_bounds)
{
    // finds outline of the largest non-white object within search bounds
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
    cv::threshold(thresh, thresh,70,255,cv::THRESH_BINARY_INV);
    thresh = thresh - _gripper_mask;
    thresh = thresh(search_bounds);
    int type = cv::MORPH_ELLIPSE;
    int sz = 1;
    cv::Mat element = getStructuringElement( type,
                                             cv::Size(2*sz+1, 2*sz+1),
                                             cv::Point(sz, sz) );
    cv::erode(thresh, thresh, element); 
    
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
    //std::cout<<"blob area: "<<largest_area<<"\n";
    //cv::imshow("blob", thresh);
    if (largest_area < min_area)
    {
        return temp;
    }
    
    return contours[largest_contour_index];
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
    cv::threshold(thresh, thresh,100,255,cv::THRESH_BINARY_INV);
    thresh = thresh - _gripper_mask;
    int type = cv::MORPH_ELLIPSE;
    int sz = 1;
    cv::Mat element = getStructuringElement( type,
                                             cv::Size(2*sz+1, 2*sz+1),
                                             cv::Point(sz, sz) );
    cv::erode(thresh, thresh, element); 
    //element = getStructuringElement( type, (3,3) );
    cv::morphologyEx(thresh, thresh, cv::MORPH_CLOSE, cv::Mat() );
    cv::morphologyEx(thresh, thresh, cv::MORPH_CLOSE, cv::Mat() );
    
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
    //std::cout<<"blob area: "<<largest_area<<"\n";
    //cv::imshow("blob", thresh);
    if (largest_area < min_area)
    {
        return temp;
    }
    // get convexHull of contour -- just outer edges
    std::vector<cv::Point> hull;
    convexHull(contours[largest_contour_index], hull, false, true);
    
    return hull;
    //return contours[largest_contour_index];
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
    
    cv::Mat scene = _right_cam->cvImage();
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

void SearchControl::_endpoint_control(gv::Point err, double w2)
{
    if(!ros::ok())
        return;
    std::cout<<"endpoint control"<<std::endl;
    ros::spinOnce();
    gv::RPYPose pose = geometry.home;
    pose.position = _right_hand->endpoint_pose().position;
    //pose.position.print("current position");
    //err.print("Error");
    pose.position -= err;
    //pose.position.z = this->geometry.home.position.z;
    //pose.position.print("desired position");
    //_right_hand->joint_positions().print("current angles");
    JointPositions desired = _right_hand->get_simple_positions(pose.position,w2);
    //desired.print("desired angles");
    if(desired.angles.empty())
        return;
    ////_right_controller->set(desired);
    //_right_hand->set_joint_positions(desired);
    _right_hand->set_velocities(desired);
    //_right_hand->exit_control_mode();
}

void SearchControl::_endpoint_control(gv::Point err)
{
    //std::cout<<"endpoint position"<<std::endl;
    ros::spinOnce();
    gv::RPYPose pose = geometry.home;
    pose.position = _right_hand->endpoint_pose().position;
    pose.position -= err;
    //pose.position.z = geometry.home.position.z-.1;
    pose.position.z = this->geometry.home.position.z;
    JointPositions desired = _right_hand->get_simple_positions(pose.position,0);
    //desired.print();
    if(desired.angles.empty())
        return;
    //v_print(desired.angles, "positions");
    //_right_hand->set_joint_positions(desired);
    _right_hand->set_command_timeout(0.8);
    _right_hand->set_velocities(desired);
    //_right_hand->exit_control_mode();
//    _endpoint_control(err, 0);
}

double SearchControl::_getBlobArea()
{
    cv::Mat scene = _right_cam->cvImage();
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
    // gets centroid of blob specified by points
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
    // gets bounding rectangle of contour
    //  openCV's function to do this crashes
    std::vector<cv::Point> bounds(2,cv::Point(0,0));
    bounds[0].x = scene_width;  // xmin
    bounds[0].y = scene_height; // ymin
    bounds[1].x = 0;      // xmax
    bounds[1].y = 0;      // ymax
    cv::Point current_point;
    for(int i = 0; i < contour.size(); i++)
    {
        current_point = contour[i];
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
    // increases bounds of rect to if new point is outside rect
    double x, y;
    x = new_point.x;
    y = new_point.y;
    
    // x > x_max
    if (x > rect[1].x)
        rect[1].x = x;
    // x < x_min
    else if(x < rect[0].x)
        rect[0].x = x;
    // y > y_max
    if (y > rect[1].y)
        rect[1].y = y;
    // y < y_min
    else if(y < rect[0].y)
        rect[0].y = y;
    
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
    // resets object location used in visual servoing
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

void SearchControl::_transform(gv::Point &err, double yaw, double range)
{
    // ASSUME THE CAMERA IS POINTING STRAIGHT DOWN (ALONG Z AXIS)
    //transform between pixels and real distances.
    // (pixels/pixel)*(m/m)*m
    double tempx,tempy;
    // height_ratio is not linear,constant with range
    //  maybe develop a better model...
    //std::cout<<"range: "<<range;
    if (range > .08){
        //std::cout<<" is *large*\n";
        tempy = err.y * height_ratio * range;
        tempx = err.x * width_ratio * range;
    }
    else{
        //std::cout<<" is *small*\n";
        tempy = err.y * 2.28 * range;
        tempx = err.x * 3.648 * range;
    }
    
    // account for angle of hand (yaw)
    // 0 yaw has gripper "facing" backwards
    err.x = -tempy*cos(yaw) + tempx*sin(yaw);
    err.y = -tempy*sin(yaw) - tempx*cos(yaw); 
}

void SearchControl::_transform(gv::Point &err, double yaw)
{
    // ASSUME THE CAMERA IS POINTING STRAIGHT DOWN (ALONG Z AXIS)
    //transform between pixels and real distances.
    // (pixels/pixel)*(m/m)*m
    float range = _right_hand->gripper->range;
    double tempx,tempy;
    tempy = err.y * height_ratio * range;
    tempx = err.x * width_ratio * range;
    
    // account for angle of hand (yaw)
    // 0 yaw has gripper "facing" backwards
    err.x = -tempy*cos(yaw) + tempx*sin(yaw);
    err.y = -tempy*sin(yaw) - tempx*cos(yaw); 
}

#endif