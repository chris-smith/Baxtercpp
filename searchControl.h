#include "baxterLimb.h"
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
    
    void test_transform(gv::Point, double);
    
private:
    SearchControl(); //no access to default constructor
    
    BaxterLimb* _cam_hand;
    BaxterLimb* _manip_hand;
    BaxterCamera* _search_cam;          // used to find objects in scene
    BaxterCamera* _control_cam;         // used to help control positioning of _search_cam
    ObjectClassifier* _classifier;
    NxtKit* _kit;
    //cv::SimpleBlobDetector* _blob_detector;
    
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
    bool _object(cv::Rect);                              //  looks for object within bounds of rectangle
    bool _roi_object();                                  //  looks for an object in subset of the image
    bool _track_object();
    void _reset_loc();                                   //  sets _obj_loc to (-1,-1)
    void _reset_roi();                                   //  sets _roi to (-1,-1)
    void _set_roi(cv::Rect, double);                     //  set _roi based on _obj_loc and area of obj
    cv::Point _get_centroid(std::vector< cv::Point >);   //  returns centroid of a set of points, calculated as mean of points
    void _transform(gv::Point &, double);                    //  transforms dimensionless error to real distance. accounts for hand rotation
    void _transform(gv::Point &, double, double);
    void _dist_transform(gv::Point &, double);
    cv::Rect _get_bounds(std::vector<cv::Point>);        //  returns a bounding rectangle for the contour
    std::vector<cv::Point> _corners(cv::Rect);
    std::vector<cv::Point> _increase_bounds(std::vector<cv::Point>, cv::Point);
    cv::Rect _restrict(std::vector<cv::Point>, double,double);     //  returns rectangle enclosed in size of image
    cv::Rect _getBlobRoi(cv::Point2d pt);
    std::vector<cv::Point> _getBlob(cv::Mat&);            //   return contour of largest blob in image
    std::vector<cv::Point> _getBlob(cv::Mat&, cv::Rect);
    std::vector<cv::Point> _getCentralBlob(cv::Mat&);
    void _endpoint_control(gv::Point);
    void _endpoint_control(gv::Point, double);
    void _endpoint_control_accurate(gv::Point err);
    void _drop_to_table(gv::RPYPose);
    void _drop_to_table_two(gv::RPYPose desired);
    void _lower(double);                                      //  lowers down to table keeping object in center of camera
    double _getBlobArea();
    cv::Point2f _best_gripping_location(std::vector<cv::Point>);
    bool _on_edge(std::vector<cv::Point> blob);
    void _remove_grippers();            // finds grippers in test image for removal later
    void _limit_bounds(cv::Rect& bounds);
    
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
};

SearchControl::SearchControl()
{
    ROS_ERROR("This object can not be declared without passing it arguments. \
        The default constructor on this object is private. \
        You should never see this error message.");
}

SearchControl::SearchControl(BaxterLimb* a, BaxterLimb* b, BaxterCamera* cam_a, BaxterCamera* cam_b)
{
    _cam_hand = a;
    _manip_hand = b;
    _search_cam = cam_a;
    _control_cam = cam_b;
    _classifier = new ObjectClassifier("/home/ceeostud2/Pictures/ObjectTemplates/", 300);
    _state = 0;
    _reset_loc();
    _reset_roi();
    min_area = 75;
    scene_height = _search_cam->height();
    scene_width = _search_cam->width();
    _tracking = false;
    _track_state = false;
    _predictor.dimension = 20;
    _predictor.k = 0.5;
}

SearchControl::~SearchControl()
{
    /*delete _cam_hand;
    delete _manip_hand;
    delete _search_cam;
    delete _control_cam;*/
    delete _classifier;
    //delete _blob_detector;
}

void SearchControl::swap_hands()
{
    // swap limbs
    std::swap(_cam_hand, _manip_hand);
    // swap cameras
    std::swap(_search_cam, _control_cam);
    
}

void SearchControl::search()
{
    if(!_search_initialized)
        _init_search();
    
    if (!ros::ok())
        return;
    scene_height = _search_cam->height();
    scene_width = _search_cam->width();
    ros::Rate r(20);
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
            _state = 0;  //should this happen??
            break;
    }
    std::cout<<std::endl;
    ros::spinOnce();
}

cv::SimpleBlobDetector::Params _blob_params()
{
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
    _search_initialized = true;
    _state = 0;
    allowable_error = gv::Point(10,10,1); // need the 1 so err < allow is true
    if (geometry.home == gv::RPYPose(0))
    {
        geometry.home = _cam_hand->endpoint_pose();
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
    //std::cout<<_search_cam->height()<<" "<<_search_cam->width();    std::cout<<"roll: " << pos.rpy.roll <<"\npitch: "<<pos.rpy.pitch<<"\nyaw: "<<pos.rpy.yaw<<"\n";
    ros::Duration timeout(10);
   //   COMMENTED OUT WHILE TESTING IMAGE RECOGNITION
//     if ( _cam_hand->set_service_position_quick(pos, timeout) < 0 )
//     {
//         ROS_ERROR("Unable to initialize search position");
//         _search_initialized = false;
//         search(); //to ensure searching doesn't begin
//     }
    _cam_hand->set_command_timeout(0.5);
    pos.print("Starting Position");
    _cam_hand->set_simple_positions(pos.position, 0, timeout);
    //std::cout<<"Position set\n";
    _cam_hand->gripper->open();
    //std::cout<<"gripper opened\n";
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
    cv::RotatedRect nxtkit(cv::Point2f(-.22869,-.71439),cv::Size2f(.38,.27),-54.15);
    _kit = new NxtKit(nxtkit,-.065);
    pos.position.x -= .05;
    pos.position.y -= .05;
    //_cam_hand->set_simple_positions(pos.position, 0, timeout);
    _gripping = false;
    _blob_area = 0; 
    std::cout<<"Search Initialized\n";
}

void SearchControl::_remove_grippers()
{
    std::cout<<"Removing grippers from scene\n";
    if (!_object())
        return;
    _cam_hand->gripper->block = true;
    _cam_hand->gripper->go_to(100);
    std::vector<cv::Point> blob;
    cv::Mat scene = _search_cam->cvImage();
    if (scene.empty()){
        ROS_ERROR("Empty scene :/");
        _remove_grippers();
    }
    //std::cout<<"scene type "<<scene.type()<<"\n";
    cv::Mat mask;
    int type = cv::MORPH_ELLIPSE;
    int sz = 2;
    cv::Mat element = getStructuringElement( type,
                                             cv::Size(2*sz+1, 2*sz+1),
                                             cv::Point(sz, sz) );
    cv::Mat thresh;
    try {
        cv::cvtColor(scene, thresh, CV_BGR2GRAY);     
    }
    catch (cv::Exception) {
        ROS_ERROR("Unable to convert image to greyscale");
        return;
    }
    cv::threshold(thresh, thresh,40,255,cv::THRESH_BINARY_INV); 
    cv::dilate(thresh, mask, element);   
    cv::dilate(mask, mask, element);
    
    _gripper_mask = mask;
    if(_cam_hand->gripper->state.position > 90)
        ROS_INFO("Set mask for open position");
    else
        ROS_INFO("Set mask for closed position");
    
    cv::Mat zeros;
    cv::cvtColor(mask, zeros, CV_GRAY2BGRA);
    _bgra_gripper_mask = zeros;
    cv::add(zeros, scene, scene);
    cv::imshow("mask add", scene);
    
    cv::waitKey(1000);
}

void SearchControl::_search()
{
    //  _state == 0
    if(_object())
        _state = 1;
    else{
        ros::Duration timeout(5);
        cv::imshow("scene", _search_cam->cvImage());
        //_cam_hand->set_service_position_quick(geometry.home, timeout);
        //_cam_hand->set_simple_positions(geometry.home.position, 0, timeout*0.5);
    }
    
}

void SearchControl::_limit_bounds(cv::Rect& bounds)
{
    if (bounds.x < 0)
        bounds.x = 0;
    if (bounds.y < 0)
        bounds.y = 0;
    if (bounds.x + bounds.width > scene_width)
        bounds.width = scene_width - bounds.x;
    if (bounds.y + bounds.height > scene_height)
        bounds.height = scene_height - bounds.y;
}
    
void SearchControl::_move_to_piece()
{
    //  _state == 1
    /*cv::Rect search_bounds = _rotated_rect.boundingRect();
    search_bounds += cv::Size(30,30);
    _limit_bounds(search_bounds);
    /*if (!_object(search_bounds))
    {
        ROS_INFO("Lost object");
        _state = 0;
        return;
    }*/
    gv::Point err; //difference between centroid of object and center of image;
    err.x = _obj_loc.x - scene_width/2; 
    err.y = _obj_loc.y - scene_height/2;
    std::cout<<"error"<<err.x<<" "<<err.y<<"\n";
    gv::Point temp = abs(err);
//     temp.print("current error");
//     allowable_error.print("allowable error");
    double scene_area = scene_height*scene_width;
    double obj_area = _bounding_rect.height*_bounding_rect.width;
    double ratio = obj_area/scene_area;
    //std::cout<<"bounding rect ratio: "<<ratio<<"\n";
    if (temp < allowable_error)
    {
        _state = 0;
        std::cout<<"arrived at object";
        return;
    }
    // pixels -> pixels/pixel
    err.y /= scene_height;
    err.x /= scene_width; 
    //err.print("Non dimensionalized error");
    double rotation = _rotated_rect.angle; //this is in DEGREES
    if (_rotated_rect.size.width < _rotated_rect.size.height){
        rotation = 90 - rotation;
    }
    ros::spinOnce();
    gm::Pose pos = _cam_hand->endpoint_pose();
    //gm::Quaternion quat = pos.orientation;
    gv::Quaternion orient = pos.orientation;
    double yaw = gv::RPY(orient).yaw;
    double height = pos.position.z;
    //std::cout<<"yaw: "<<yaw<<"\n";
    _transform(err, yaw, height-geometry.table_height);
    err.print("error");
    //cv::waitKey(500);
    //err.z = 0.005;
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
    /*Resolution orig = _search_cam->resolution();
    Resolution ver(480,300);
    //_search_cam->resize(ver);
    _lower(geometry.table_height+.08);
    cv::Mat scene = _search_cam->cvImage();
    std::vector<cv::Point> blob = _getCentralBlob(scene);
    cv::Rect bounded = _get_bounds(blob);
    scene = scene(bounded);
    _classifier->scene(scene);
    _classifier->match();
    //_search_cam->resize(orig);
    cv::waitKey(500);*/
    std::cout<<"What piece is this?\n";
    std::cin>>_found_piece;
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
    _cam_hand->gripper->go_to(100);
    std::vector<cv::Point> blob = _getBlob(scene);
    _lower(geometry.table_height+.01);
    gv::Point pos;
    pos = _cam_hand->endpoint_pose().position;
    pos.z = geometry.table_height;//-.012;
    ros::Duration tout(5);
    JointPositions jp = _cam_hand->joint_positions();
    double w2_ = jp.at("_w2");
    std::cout<<"Final drop to height "<<pos.z<<"\n";
    _cam_hand->set_simple_positions(pos, w2_, tout);
    _cam_hand->gripper->go_to(5);
    ros::Duration pause(1);
    pause.sleep();
    bool gripped = _cam_hand->gripper->state.gripping;
    if (gripped)
        ROS_INFO("GRABBED PIECE CORRECTLY");
    _cam_hand->set_simple_positions(geometry.home.position, 0.0, tout);
    _cam_hand->gripper->go_to(100);
    _state = 4;
    return;
    float rotation;
    cv::RotatedRect rect;
    cv::Mat scene = _search_cam->cvImage();
    //std::vector<cv::Point> blob = _getBlob(scene);
    //_best_gripping_location(blob); // SHOULD USE cv::CONVEXITYDEFECTS() TO HELP DO THIS
    rect = cv::minAreaRect(blob);
    cv::Point2f rect_points[4];
    rect.points(rect_points);
    for(int j = 0; j < 4; j++)
    {
        line(scene, rect_points[j], rect_points[(j+1)%4], cv::Scalar(255,0,0),1,8);
    }
    cv::imshow("Rotated Rectangle", scene);
    cv::waitKey(30);
    // THE ROTATION ANGLE IS ALWAYS NEGATIVE
    rotation = rect.angle; //this is in DEGREES
    std::cout<<"rotation: "<<rotation<<"\n";
    std::cout<<"box height: "<<rect.size.height<<"  box width: "<<rect.size.width<<"\n";
    ros::spinOnce();
    // FOR OBJECTS THAT LOOK LIKE  \ \
    //                             \_\
    // OBJECT HEIGHT IS GREATER THAN WIDTH
    // FOR OBJECTS THAT LOOK LIKE /  /
    //                           /__/
    // OBJECT WIDTH IS GREATER THAN HEIGHT
   // float range = _cam_hand->gripper->range;
    gv::Pose pose = _cam_hand->endpoint_pose();
    gv::RPYPose rpypose(pose);
    std::cout<<"\nPosition at Start of Grip";
    rpypose.print();
    gv::RPYPose original(rpypose);
    //rotation = fabs(rotation);
    std::cout<<"rectangle size\n\tx: " << rect.size.width <<"\n\ty: " << rect.size.height;
    std::cout<<"\nrotation: " << rotation << std::endl;
    gv::Point mult(1,1,0);
    if (rect.size.width > rect.size.height){
        rotation = 90 + rotation;
    }
    /*if (rotation < 45)
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
    }*/
    
    std::cout<<"new rotation: " << rotation << std::endl;
    rotation = rotation*PI/180; //to radians
    rpypose.rpy.yaw += rotation; // ALSO NEED TO REORIENT HAND
    gv::Point offset(-.012, -.03825, 0); // .03302, .0127 -> camera to endpoint center
    //double yaw = rpypose.rpy.yaw - rotation;
    //yaw -= 1.57;   // account for starting yaw
    _dist_transform(offset, rotation);
    std::cout<<"transformed offset";
    offset.print();
    //range -= _cam_hand->gripper->length();
    //std::cout<<"range: "<<range<<std::endl;
    rpypose.position -= offset;
    //std::cout<<"offset position";
    rpypose.print("accounting for rotation...");
    
     ros::Duration timeout(8);
    _cam_hand->set_simple_positions(rpypose.position, rotation, timeout); // move gripper over object
    //_cam_hand->set_position(original, timeout); //for testing
    /*rpypose.position.z -=.1;
    rpypose.print("initial drop");
    ros::spinOnce();
    ros::Time start = ros::Time::now();
    _cam_hand->set_position(rpypose, timeout);
    //rpypose.position.z -= .017;  //go accurately to 
    /*if ( _cam_hand->set_position(rpypose, timeout) > 0 )
    { 
        _cam_hand->gripper->go_to(40);
        ros::Rate delay(2);
        delay.sleep();
    }
    else
        ROS_ERROR("Unable to grab a piece at the specified location");*/
    offset.x = 0.0;
    offset.y = 0;
    //_dist_transform(offset, rpypose.rpy.yaw + PI);
    offset.z = .05;
    offset.print();
    _cam_hand->gripper->go_to(100);
    gv::RPYPose currentPos = _cam_hand->endpoint_pose();
    double w2 = rotation;
    _cam_hand->set_simple_positions(currentPos.position, w2, timeout);
    currentPos.position.z = geometry.table_height-.01;
    _cam_hand->set_simple_positions(currentPos.position, w2, timeout);
    //rpypose = _cam_hand->endpoint_pose();
    //rpypose.position -= offset;
    //_drop_to_table_two(rpypose);
    _cam_hand->gripper->go_to(0);
    ros::Rate delay(1);
    delay.sleep();
    if ( _cam_hand->set_service_position_quick(original, timeout) < 0 )
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
    cv::RotatedRect container = _kit->get_coordinates(_found_piece);
    if(container.size == cv::Size2f(-1,-1)){
        ROS_ERROR("The piece [%s] does not exist in this kit", _found_piece.c_str());
        _state = 0;
        return;
    }
    _cam_hand->gripper->block = true;
    //cv::Point dropoff_pt = container.center;
    gv::RPYPose dropoff;
    dropoff.position = container.center;
    dropoff.position.print("Deposit location");
    _state = 0;
    return;
    dropoff.position.z = _kit->height() + .005;
    dropoff.rpy = gv::RPY(0, 3.14, 1.57);
    ros::Duration timeout(5);
    int moved = _cam_hand->set_service_position_quick(dropoff, timeout);
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
    _cam_hand->set_simple_positions(dropoff.position, 0.0, quickTimeout);
    _cam_hand->gripper->go_to(100);
    //return to search area
    ros::Rate delay(1);
    delay.sleep();
    _cam_hand->set_service_position_quick(geometry.home, timeout);
    _state = 0;
    _cam_hand->gripper->block = false;
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
    float rotation = 1;
    cv::RotatedRect rect;
    cv::Mat scene;
    cv::Point2d centroid;
    std::vector<cv::Point> blob;
    gv::Point error(1);
    gv::RPYPose pose;
    JointPositions jp;
    ros::Duration timeout(.01);
    jp.names.push_back(_cam_hand->name()+"_w2");
    double height = _cam_hand->endpoint_pose().position.z;
    double w2 = 0;
    while ((!(error.abs() < 0.002) || (fabs(rotation) > 0.01)) && ros::ok())
    {
        scene = _search_cam->cvImage();
        blob = _getBlob(scene);
        //_best_gripping_location(blob);
        try{
            if(!blob.empty())
                centroid = _best_gripping_location(blob);
            //centroid = _get_centroid(blob);
            circle(scene, centroid, 10, cv::Scalar(-1), 1, 8);
            error.x = centroid.x - scene_width/2; 
            error.y = centroid.y - scene_height/2;
            error.y /= scene_height;
            error.x /= scene_width; 
            error.y *= 3;
            error.print("error");
            rect = cv::minAreaRect(blob);
            cv::Point2f rect_points[4];
            rect.points(rect_points);
            for(int j = 0; j < 4; j++)
            {
                line(scene, rect_points[j], rect_points[(j+1)%4], cv::Scalar(255,0,0),1,8);
            }
            cv::imshow("Rotated Rectangle", scene);
            cv::waitKey(10);
            // THE ROTATION ANGLE IS ALWAYS NEGATIVE
            rotation = rect.angle; //this is in DEGREES
            //std::cout<<"rotation: "<<rotation<<"\n";
            // OBJECT HEIGHT IS GREATER THAN WIDTH
            // FOR OBJECTS THAT LOOK LIKE  \ \
            //                             \_\
            //
            // OBJECT WIDTH IS GREATER THAN HEIGHT
            // FOR OBJECTS THAT LOOK LIKE /  /
            //                           /__/
            //std::cout<<"rectangle size\n\tx: " << rect.size.width <<"\n\ty: " << rect.size.height;
            if (rotation < 45)
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
            pose = _cam_hand->endpoint_pose();
            jp = _cam_hand->joint_positions();
            w2 = jp.at("right_w2") + rotation;
            double yaw = pose.rpy.yaw;
            height = pose.position.z;
            _transform(error, yaw, height-geometry.table_height);
            error.print("error");
            error.z = 0;
            if(height > stop_height)
                error.z = .02;
            //std::cout<<"rotation: "<<rotation<<"\n";
            _endpoint_control(error, w2); //takes in error, w2 angle
            ros::spinOnce();
            pose = _cam_hand->endpoint_pose();
        }
        catch(cv::Exception e){
            ROS_ERROR(e.what());
        }
        //error.z = pose.position.z - geometry.table_height;    
    }
    //pose.print();
}

void SearchControl::_drop_to_table(gv::RPYPose rpypose)
{
    JointPositions desired = _cam_hand->get_simple_positions(rpypose.position, 0);
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

void SearchControl::_drop_to_table_two(gv::RPYPose desired)
{
    gv::RPYPose cur = _cam_hand->endpoint_pose();
    ros::Duration timeout(10);
    cur.position.z = geometry.table_height;
    _cam_hand->set_jacobian_position(cur, timeout);
}

bool SearchControl::_object()
{
    _reset_loc();
    cv::Mat scene = _search_cam->cvImage();
    whole_scene = scene;
    std::vector<cv::Point> blob = _getBlob(scene);
    if (!blob.empty())
    {
        _bounding_rect = _get_bounds(blob);
        _rotated_rect = cv::minAreaRect(blob);
        _obj_loc = _get_centroid(blob);
        circle(whole_scene, _obj_loc, 10, cv::Scalar(255,0,0), 1, 8);
        rectangle(whole_scene, _bounding_rect, cv::Scalar(0,255,0),1,8);
        cv::imshow("scene", whole_scene);
    }
    else
        return false;
    cv::waitKey(10);
    return true;
}

bool SearchControl::_object(cv::Rect bounds)
{
    _reset_loc();
    cv::Mat scene = _search_cam->cvImage();
    whole_scene = scene;
    std::vector<cv::Point> blob = _getBlob(scene, bounds);
    if (!blob.empty())
    {
        _bounding_rect = _get_bounds(blob);
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
    cv::waitKey(10);
    return true;
}

cv::Point2f SearchControl::_best_gripping_location(std::vector<cv::Point> blob)
{
    // return a negative number if the hand needs to drop down
    // positive number if the hand needs to move up
    int max_height = 0;
    int min_width = scene_width;
    double area = cv::contourArea(blob, false);
    cv::RotatedRect rect = cv::minAreaRect(blob);
    double rectArea = rect.size.height*rect.size.width;
    double percentArea = area/rectArea;
    std::cout<<"Percent Area: " << percentArea<<"\n";
    if (percentArea > .75)
    {
        // the object is fairly square, just grab it at the default location
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

std::vector<cv::Point> SearchControl::_getCentralBlob(cv::Mat& scene)
{
    //this will ignore blobs on the edge of the image
    // finds outline of the largest non-white object
    scene_height = _search_cam->height();
    scene_width = _search_cam->width();
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
    cv::threshold(thresh, thresh,100,200,cv::THRESH_BINARY_INV); 
    
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
            if(!_on_edge(contours[i])){
                largest_area = a;
                largest_contour_index = i;
            }
        }
    }
    std::cout<<"blob area: "<<largest_area<<"\n";
    cv::imshow("blob", thresh);
    if (largest_area < min_area)
    {
        return temp;
    }
    
    return contours[largest_contour_index];
}

std::vector<cv::Point> SearchControl::_getBlob(cv::Mat& scene, cv::Rect search_bounds)
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
    /*cv::SimpleBlobDetector::Params params;
    params.minDistBetweenBlobs = 50.0f;
    params.filterByInertia = false;
    params.filterByConvexity = false;
    params.filterByColor = true;
    params.blobColor = 0;
    params.filterByCircularity = false;
    params.filterByArea = true;
    params.minArea = 100.0f;
    cv::Ptr<cv::FeatureDetector> blob_detector = new cv::SimpleBlobDetector(params);
    blob_detector->create("SimpleBlob");
    std::vector<cv::KeyPoint> keypoints;
    blob_detector->detect(thresh, keypoints);
    if(!keypoints.empty())
        //cv::drawKeypoints(scene, keypoints, scene);
    //cv::imshow("keypoints", scene);*/
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
    cv::imshow("blob", thresh);
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
    //cv::erode(thresh, thresh, element); 
    
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

void SearchControl::_endpoint_control(gv::Point err, double w2)
{
    if(!ros::ok())
        return;
    //std::cout<<"endpoint position"<<std::endl;
    gv::RPYPose pose = geometry.home;
    ros::spinOnce();
    pose.position = _cam_hand->endpoint_pose().position;
    //pose.print("current position");
    //err.print("Error");
    pose.position -= err;
    //pose.position.z = geometry.home.position.z;
    pose.position.print("desired position");
    JointPositions desired = _cam_hand->get_simple_positions(pose.position,w2);
    desired.print();
    if(desired.angles.empty())
        return;
    //v_print(desired.angles, "positions");
    //_cam_hand->set_joint_positions(desired);
    //_cam_hand->set_joint_positions(desired);
    _cam_hand->exit_control_mode();
}

void SearchControl::_endpoint_control(gv::Point err)
{
    //std::cout<<"endpoint position"<<std::endl;
    gv::RPYPose pose = geometry.home;
    pose.position = _cam_hand->endpoint_pose().position;
    pose.position -= err;
    //pose.position.z = geometry.home.position.z-.1;
    pose.position.z = this->geometry.home.position.z;
    JointPositions desired = _cam_hand->get_simple_positions(pose.position,0);
    //desired.print();
    if(desired.angles.empty())
        return;
    //v_print(desired.angles, "positions");
    //_cam_hand->set_joint_positions(desired);
    _cam_hand->set_velocities(desired);
    _cam_hand->exit_control_mode();
//    _endpoint_control(err, 0);
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

void SearchControl::_transform(gv::Point &err, double yaw, double range)
{
    // ASSUME THE CAMERA IS POINTING STRAIGHT DOWN (ALONG Z AXIS)
    //std::cout<<"range: "<<range<<"\n";
    //transform between pixels and real distances.
    // (pixels/pixel)*(m/m)*m
    std::cout<<"yaw: "<<yaw<<"\n";
    double tempx,tempy;
    tempy = err.y * height_ratio * range;
    tempx = err.x * width_ratio * range;
    // camera pixel axis is rotated from endpoint axis
    //yaw += PI;
    // account for angle of hand (yaw)
    // 0 yaw has gripper "facing" backwards
    err.x = -tempy*cos(yaw) + tempx*sin(yaw);
    err.y = -tempy*sin(yaw) - tempx*cos(yaw); 
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
}

#endif