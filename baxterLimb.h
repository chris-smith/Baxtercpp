// These need to be included before ros.h
#include <Eigen/SVD>
#include <Eigen/LU>
#include <kdl/segment.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/treejnttojacsolver.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include "ros/ros.h"
#include <iostream>
#include <string>
#include <stdio.h>
#include <sstream>
#include <baxter_core_msgs/JointCommand.h>
#include <baxter_core_msgs/EndpointState.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <baxter_msgs/SolvePositionIK.h>        // this is probably deprecated
#include <fstream>
#include <math.h>

//each limb has a gripper
#include "baxterGripper.h"
#include "geometryTypes.h"


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

double toSec(int);
int toNsec(double);


#ifndef VECTOR_MATH
#define VECTOR_MATH
// some quick math used with vectors for controllers

std::vector<double> v_difference(std::vector<double>,std::vector<double>);
std::vector<double> v_sum(std::vector<double>,std::vector<double>);
std::vector<double> product(std::vector<double>, double);
std::vector<double> quotient(std::vector<double>, double);
void v_print(std::vector<double>);
void v_print(std::vector<double>, std::string);
double limit(double, double); //limits val to max. preserves sign

#endif


#ifndef BAXTER_LIMB
#define BAXTER_LIMB

#define NUMJOINTS 7
#define FAST true
#define SLOW false
#define GripperLength 7.5

// gains for PID
struct Gains { double kp; double ki; double kd; 
    Gains() : kp(0), ki(0), kd(0) {}
    Gains(double a, double b, double c) : kp(a), ki(b), kd(c) {}
};

// list of joint ids
std::string joint_ids[NUMJOINTS] = {"_s0","_s1","_e0","_e1","_w0","_w1","_w2"};

class BaxterLimb {
public:
    BaxterLimb(KDL::Tree, std::string,uint);       //kdl tree, side, buffer size
    BaxterLimb(std::string);                       //side
    ~BaxterLimb();
    
    BaxterGripper* gripper;
    
    void Init();
    
    //  Access Subscriber data
    std::vector<double> joint_angles();                 //return _joint_angle
    std::vector<double> joint_velocities();             //return _joint_velocity
    std::vector<double> joint_efforts();                //return _joint_effort
    JointPositions joint_positions();
    std::string name();
    std::vector<std::string> joint_names();             //return _joint_names
    gm::Pose endpoint_pose();                           //return endpoint pose
    gm::Twist endpoint_velocity();                      //return endpoint velocity
    gm::Wrench endpoint_effort();                       //return endpoint effort
    unsigned short state_rate();                        //return _state_rate
    
    // Set/Get control parameters
    void set_allowable_error(std::vector<double>);
    void set_allowable_error(double);
    void set_max_velocity(std::vector<double>);
    void set_max_velocity(double);
    void set_max_acceleration(std::vector<double>);
    void set_max_acceleration(double);
    void set_joint_pid(std::string, Gains);
    Gains get_joint_pid(std::string);
    std::vector<Gains> get_joint_pids();
    
    // Set velocities/positions/efforts -- Baxter Defined Controllers
    void set_joint_positions(JointPositions);
    void set_joint_velocities(JointVelocities);
    void set_joint_efforts(JointEfforts);
    void set_command_timeout(double);
    void exit_control_mode(double);
    void set_joint_position_speed(double);
    
    // Simplified Geometry IK/Controllers - Relies on KDL Tree
    JointPositions get_simple_positions(gv::Point, double);
    void set_simple_positions(gv::Point, double, ros::Duration);
    
    // Baxter Service based IK/Controllers -- DEPRECATED??
    JointPositions get_service_position(gv::RPYPose);
    int set_service_position_quick(gv::RPYPose, ros::Duration);
    int set_service_position_accurate(gv::RPYPose, ros::Duration);
    int set_service_position(gv::RPYPose, ros::Duration);
    
    // KDL IK/Controllers -- EXPERIMENTAL
    int set_jacobian_position(gv::RPYPose, ros::Duration);
    void lock_joints(std::vector<int>);
    void lock_joints(std::vector<std::string>);
    void unlock_joints();
    void unlock_joints(std::vector<int>);
    void unlock_joints(std::vector<std::string>);
    KDL::Jacobian jacobian();
    KDL::Jacobian get_jacobian();
    void set_jacobian();
    void resize_jacobian(int);
    gv::RPYPose endpoint_error;
    
    /* The following are assorted controllers, getters, setters 
     * --------------------------------------------------------------*/
    
    // Velocity Controllers, blocking
    int accurate_to_position(JointPositions, ros::Duration);
    int quickly_to_position(JointPositions, ros::Duration);
    
    // Position Controller, blocking
    int to_position(JointPositions, ros::Duration);
    
    // Returns instantaneous velocities to move to desired position - conservative
    JointVelocities get_velocities(JointPositions);
    
    // sets velocities to move to desired position, non-blocking
    int set_velocities(JointPositions);
    
    // check if joint position error is within allowed range
    bool in_range(std::vector<double>);
    
    // compute control values based on PID gains
    std::vector<double> compute_gains(std::vector<double>, std::vector<double>, std::vector<double>, bool);
    
    // limits vels, accs to within specified limits
    void limit_velocity(std::vector<double>&);                                      //v = ( |v| > _max_vel ? _max_vel : v )
    void limit_acceleration(std::vector<double>&, std::vector<double>, double);     //ensures velocity not changing too fast
    
private:
    BaxterLimb();
    bool _set; //true if any subscribers have returned data yet
    
    // various member data
    std::string _name;                          //name of limb (left, right)
    std::string _joint_names[NUMJOINTS];        //explicit joint names
    std::vector<double> _joint_angle;
    std::vector<double> _joint_velocity;
    std::vector<double> _joint_effort;
    std::vector<double> _allowed_error;         // error to be considered "there" for velocity control
    std::vector<double> _max_velocity;          // strictly >= 0
    std::vector<double> _max_acceleration;      // strictly >= 0
    std::vector<Gains> _pid;
    gm::Pose _cartesian_pose;
    gm::Twist _cartesian_velocity;
    gm::Wrench _cartesian_effort;
    //baxter_msgs::JointCommandMode _pub_mode;   
    
    ros::NodeHandle _nh;        
    unsigned int _bufferSize;                    // size of subscriber buffer
    ros::Subscriber _joint_state_sub;
    ros::Subscriber _cartesian_state_sub;
    ros::Publisher _pub_joint_cmd;
    ros::Publisher _pub_joint_cmd_timeout;
    ros::Publisher _pub_speed_ratio;
    
    unsigned short _state_rate;
    ros::Time _last_state_time;
    
    // KDL & Eigen
    bool _kdl_set;                              // true if kdl params have been set
    bool _kdl_constructor;                      // true if kdl constructor called
    std::vector<bool> _locked_joints;
    KDL::Chain _chain;
    KDL::Jacobian _jacobian;
    KDL::ChainJntToJacSolver *_jacobian_solver;
    KDL::JntArray _kdl_jointpositions;
    void _check_kdl();                          // checks that kdl FK calculations concur with subscriber data
    void _set_shoulder_mount();                 // find location of shoulder mount for simple IK
    gv::Point _shoulder_mount;                  // location of shoulder mount
    
    void _set_names();                          //sets _joint_names in constructor
    
    // subscriber callbacks
    void _on_endpoint_states(const baxter_core_msgs::EndpointState::ConstPtr&); 
    void _on_joint_states(const sensor_msgs::JointState::ConstPtr&); 
    
    // checks if error < _allowed_error
    // SHOULD REMOVE, REPLACE WITH PUBLIC VERSION
    bool _in_range(std::vector<double>, bool);  
    
    const char* _max_error(std::vector<double>); //returns joint name and error of largest error
    
};


BaxterLimb::BaxterLimb() {
    ROS_ERROR("Private Default Constructor -- Should never be called");
}

BaxterLimb::BaxterLimb(KDL::Tree tree,std::string name,uint bufferSize) {
    // constructor if KDL is required
    _name = name;
    _bufferSize = bufferSize;
    _kdl_constructor = true;
    _kdl_set = false;
    std::string endpoint = name + "_gripper";
    tree.getChain("base", endpoint, _chain);
    Init();
}

BaxterLimb::BaxterLimb(std::string name) {
    // basic constructor
    _name = name;
    _kdl_constructor = false;
    _kdl_set = false;
    _bufferSize = 1;
    std::cout<<"Initializing "<<name<<" limb...\n";
    Init();
}

BaxterLimb::~BaxterLimb() {
    // destructor
    delete gripper;
    
    if (_jacobian_solver != NULL)
        delete _jacobian_solver;
}

void BaxterLimb::Init() {
    // initialization common to both
    ros::Rate r(5);
    _set = false;
    
    // Resize vectors
    _joint_angle.resize(NUMJOINTS, 0);
    _joint_velocity.resize(NUMJOINTS, 0);
    _joint_effort.resize(NUMJOINTS, 0);
    _allowed_error.resize(NUMJOINTS, 0);
    _max_velocity.resize(NUMJOINTS, 0);
    _max_acceleration.resize(NUMJOINTS, 0);
    _kdl_jointpositions.resize(NUMJOINTS);
    _pid.resize(NUMJOINTS);
    KDL::SetToZero(_kdl_jointpositions);
    // set up joint names
    _set_names();
    
    _jacobian_solver = NULL;
    _state_rate = 0;
    _last_state_time = ros::Time::now();
    
    // setup subscribers
    std::string topic = "/robot/limb/"+_name;
    _joint_state_sub = _nh.subscribe("/robot/joint_states", _bufferSize, &BaxterLimb::_on_joint_states, this);
    _cartesian_state_sub = _nh.subscribe(topic+"/endpoint_state", _bufferSize, &BaxterLimb::_on_endpoint_states, this);
    
    // wait for subscriber to connect
    while (!_set && ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }
    
    // setup publishers
    _pub_joint_cmd = _nh.advertise<baxter_core_msgs::JointCommand>(topic+"/joint_command", 10);
    _pub_joint_cmd_timeout = _nh.advertise<std_msgs::Float64>(topic+"/joint_command_timeout", 1, true);
    _pub_speed_ratio = _nh.advertise<std_msgs::Float64>(topic+"/set_speed_ratio",1, true);
    
    // Enable Robot, try to make sure this happens
    ros::Publisher _pub_enable;
    std::string enable_topic = "/robot/set_super_enable";
    _pub_enable = _nh.advertise<std_msgs::Bool>(enable_topic, 1, true);
    ros::Duration _wait(5);
    ros::Time start = ros::Time::now();
    // wait for a subscriber to connect
    while ((_pub_enable.getNumSubscribers() < 1) && (ros::Time::now() - start < _wait)){
        r.sleep();
        ros::spinOnce();
    }
    if (_pub_enable.getNumSubscribers() < 1)
        ROS_ERROR("Unable to enable robot");
    // send enable message
    std_msgs::Bool enable_msg;
    enable_msg.data = true;
    _pub_enable.publish(enable_msg);
    
    // each limb has a gripper object
    gripper = new BaxterGripper(_name);
    
    // setup KDL if KDL_Tree was provided in constructor
    if (_kdl_constructor) {
        int chain_joints = _chain.getNrOfJoints();
        if (chain_joints != NUMJOINTS) {
            ROS_ERROR("The number of joints in the kinematic chain is incorrect. \
             [%d] were expected, only [%d] were supplied.", NUMJOINTS, chain_joints);
            _kdl_constructor = false;
            return;
        }
        // no joints locked at start
        for(int j = 0; j < NUMJOINTS; j++)
            _locked_joints.push_back(false);
        // set up kdl solvers
        _jacobian_solver = new KDL::ChainJntToJacSolver(_chain);
        _jacobian.resize(NUMJOINTS);
        KDL::JntArray jnts(NUMJOINTS);
        KDL::SetToZero(jnts);
        ros::spinOnce();
        
        // kdl set is working
        _kdl_set = true;
        // check that KDL data is now being set in subscriber callback
        // SHOULD TIMEOUT, DOESN'T NOW
        while(jnts == _kdl_jointpositions && ros::ok()){
            ros::spinOnce();
            r.sleep();
        }
        
        // allowable error in endpoint for KDL controlled moves
        endpoint_error = gv::RPYPose(gv::Point(0.002), gv::RPY(.006));
        
        // set position of shoulder mount
        _set_shoulder_mount();
        
        _check_kdl();
    }
    std::cout<<_name<<" limb is initialized!\n";
}

void BaxterLimb::_set_names() {
    // This is called during initalization to set joint names as well as default
    //  array values
    for(int i = 0; i < NUMJOINTS; i++) {
        _allowed_error[i] = 0.035; // ~2 degrees
        _joint_names[i] = _name+joint_ids[i];
        _pid[i] = (Gains){0,0,0};
        _max_velocity[i] = 1; //radians/sec
        _max_acceleration[i] = 100; //radians/sec^2
        
    }
}

std::string BaxterLimb::name() {
    // returns the name of the limb object ("left, right");
    return _name;
}

void BaxterLimb::_on_joint_states(const sensor_msgs::JointState::ConstPtr& msg) {
    // callback for joint state subscriber
    _set = true;
    ros::Time now = ros::Time::now();
    
    // find rate these messages are received
    double rate = 1.0/(now-_last_state_time).toSec();
    _state_rate = ((99.0*_state_rate) + rate)/100.0;
    _last_state_time = now;
    
    // spin message data into member data
    for(int i = 0; i < NUMJOINTS; i++) {
        int j = 0;
        // make sure order of joints is kept constant
        // chance of infinite loop if message returns unknown name
        while(msg->name[j] != _joint_names[i]) {
            j++;
        }
        _joint_angle[i] = msg->position[j];
        _joint_velocity[i] = msg->velocity[j];
        _joint_effort[i] = msg->effort[j];
        
        // set kdl joint positions if you're supposed to
        if (_kdl_set){
            _kdl_jointpositions(i) = _joint_angle[i];
        }
    }
}

void BaxterLimb::_on_endpoint_states(const baxter_core_msgs::EndpointState::ConstPtr& msg) {
    // callback for endpoint subscriber
    _set = true;
    _cartesian_pose = msg->pose;
    _cartesian_velocity = msg->twist;
    _cartesian_effort = msg->wrench;
}

// The following return various pieces of information about the limb object

std::vector<std::string> BaxterLimb::joint_names() {
    return std::vector<std::string>( _joint_names, _joint_names + sizeof _joint_names / sizeof _joint_names[0]);
}

std::vector<double> BaxterLimb::joint_angles() {
    return _joint_angle;
}

JointPositions BaxterLimb::joint_positions() {
    JointPositions jp;
    jp.angles = joint_angles();
    jp.names = joint_names();
    return jp;
}

std::vector<double> BaxterLimb::joint_velocities() {
    return _joint_velocity;
}

std::vector<double> BaxterLimb::joint_efforts()
{
    return _joint_effort;
}

gm::Pose BaxterLimb::endpoint_pose()
{
    return _cartesian_pose;
}

gm::Twist BaxterLimb::endpoint_velocity()
{
    return _cartesian_velocity;
}

gm::Wrench BaxterLimb::endpoint_effort()
{
    return _cartesian_effort;
}

unsigned short BaxterLimb::state_rate()
{
    return _state_rate;
}

void BaxterLimb::set_allowable_error(std::vector<double> new_err) {
    // set allowable error for each joint
    for(int i = 0; i < NUMJOINTS; i++) {
        _allowed_error[i] = new_err[i];
    }
}

void BaxterLimb::set_allowable_error(double new_err) {
    // set blanket allowable error for all joints
    for(int i = 0; i < NUMJOINTS; i++) {
        _allowed_error[i] = new_err;
    }
}

void BaxterLimb::set_max_velocity(std::vector<double> max) {
    // set max velocity for each joint
    for(int i = 0; i < NUMJOINTS; i++) {
        _max_velocity[i] = fabs(max[i]);
    }
}

void BaxterLimb::set_max_velocity(double max) {
    // set blanket max velocity for all joints
    for(int i = 0; i < NUMJOINTS; i++) {
        _max_velocity[i] = fabs(max);
    }
}

void BaxterLimb::set_max_acceleration(std::vector<double> max) {
    // set max acceleration for each joint
    for(int i = 0; i < NUMJOINTS; i++) {
        _max_acceleration[i] = fabs(max[i]);
    }
}

void BaxterLimb::set_max_acceleration(double max) {
    // set blanket max velocity for all joints
    for(int i = 0; i < NUMJOINTS; i++) {
        _max_acceleration[i] = fabs(max);
    }
}
void BaxterLimb::set_joint_positions(JointPositions position) {
    // sets desired joint positions through position controller
    // non blocking, times out
    baxter_core_msgs::JointCommand msg;
    msg.mode = baxter_core_msgs::JointCommand::POSITION_MODE;
    msg.names = position.names;
    msg.command = position.angles;
    _pub_joint_cmd.publish(msg);
}

void BaxterLimb::set_joint_velocities(JointVelocities velocity) {
    // sets desired joint velocities through velocity controller
    // non blocking, times out
    baxter_core_msgs::JointCommand msg;
    msg.mode = baxter_core_msgs::JointCommand::VELOCITY_MODE;
    msg.names = velocity.names;
    msg.command = velocity.velocities;
    _pub_joint_cmd.publish(msg);
}

void BaxterLimb::set_joint_efforts(JointEfforts effort) {
    // sets desired joint efforts through effort controller
    // non blocking, times out
    baxter_core_msgs::JointCommand msg;
    msg.mode = baxter_core_msgs::JointCommand::TORQUE_MODE;
    msg.names = effort.names;
    msg.command = effort.efforts;
    _pub_joint_cmd.publish(msg);
}

void BaxterLimb::set_command_timeout(double timeout) {
    // Set timeout in seconds for the Joint Position Controller
    std_msgs::Float64 msg;
    msg.data = timeout;
    this->_pub_joint_cmd_timeout.publish(msg);
}

void BaxterLimb::exit_control_mode(double timeout = 0.2) {
    // Clean exit from advanced control modes (joint torque or velocity)
    // otherwise motors will disable themselves if timeout occurs
    ros::spinOnce();
    this->set_command_timeout(timeout);
    this->set_joint_positions( JointPositions( this->joint_names(), this->joint_angles() ) );
}

void BaxterLimb::set_joint_position_speed(double speed = 0.3) {
    // Set ratio of max joint speed to use during joint position moves
    // This is used on Baxter's Position Controller
    //          Param speed is ratio of maximum joint speed for execution
    //          default = 0.3; range = [0.0-1.0]
    std_msgs::Float64 msg;
    if (speed > 1)
        speed = 1;
    else if (speed < 0)
        speed = 0;
    msg.data = speed;
    this->_pub_speed_ratio.publish(msg);
}

void BaxterLimb::set_joint_pid(std::string name, Gains gain) {
    // joint name must be explicit 
    // e.g left_e1 is ok
    //     e1 or _e1 is not ok
    int i = 0;
    while(_joint_names[i] != name) {
        i++;
        if(i >= NUMJOINTS) {
            ROS_ERROR("Tried to set PID for unknown joint [%s]", name.c_str());
            return;
        }
    }
    _pid[i] = gain;
}

Gains BaxterLimb::get_joint_pid(std::string name) {
    // joint name must be explicit 
    // e.g left_e1 is ok
    //     e1 or _e1 is not ok
    int i = 0;
    Gains temp(-1,-1,-1);
    while(_joint_names[i] != name) {
        i++;
        if(i >= NUMJOINTS) {
            ROS_ERROR("Tried to get PID for unknown joint [%s]", name.c_str());
            return temp;
        }
    }
    
    return _pid[i];
}

std::vector<Gains> BaxterLimb::get_joint_pids() {
    return _pid;
}

bool BaxterLimb::in_range(std::vector<double> error) {
    // checks if specified error is within range of allowed error
    bool temp = true;
    for(int i = 0; i < error.size(); i++) {
        temp = temp && (fabs(error[i]) < _allowed_error[i]);
    }
    return temp;
}

double roll_over(double angle, double limit) {
    // roll over angle 
    // used to keep angle with range of -pi to pi
    // only works for angles with range of +-2pi
    limit = fabs(limit);
    int sign = (angle < 0 ? 1 : -1);
    if (fabs(angle) > limit) {
        angle = (fabs(angle)-limit)*sign;
    }
    return angle;
}

JointPositions BaxterLimb::get_simple_positions(gv::Point pt, const double w2) {
    // simple IK method - valid for small moves

    double r,phi,z;   
    gv::Point offset;
    JointPositions jp = this->joint_positions();
    offset = pt - this->_shoulder_mount;            // xyz offset of desired position from shoulder mount

    // a bunch of math
    double d1, d2, d3, d4, a, b, c, d, t1, t2, t3, t4, o1, o2;
    d1 = .27035;
    d2 = .36442;
    d3 = .37429;
    d4 = .279525;
    o1 = 0.069;
    o2 = 0.01;
    t1 = jp.at(_name+"_s0");
    t2 = jp.at(_name+"_s1");
    t3 = jp.at(_name+"_e1");
    t4 = jp.at(_name+"_w1");
    a = o1/sin(t3);
    b = a*cos(t3);
    c = o2/sin(t4);
    d = c*cos(t4);
    z = offset.z + d4 - d1;
    // more math
    double h, l2, l3, diag, alpha, gamma, w2_new;
    l2 = d2 - b;
    l3 = d3+a-d;
    h = z + c;
    r = sqrt(offset.x*offset.x + offset.y*offset.y) - .069;
    diag = sqrt(r*r + h*h);
    phi = atan2(h,r);
    alpha = acos((l3*l3 - diag*diag - l2*l2)/(-2*l2*diag));
    gamma = acos((l2*l2 - diag*diag - l3*l3)/(-2*l3*diag));
    // these are the angles we can command
    t3 = alpha + gamma;
    t2 = phi + alpha;
    t4 = PI/2 - t3 + t2;
    t2 = -t2;
    if (this->_name == "left") {
        t1 = atan2(offset.y,offset.x) - PI/4;
    }
    else {
        t1 = atan2(offset.y,offset.x) + PI/4;
    }
    w2_new = roll_over(w2, PI);
    // setup joint positions
    jp.names.clear();
    jp.angles.clear();
    jp.angles.push_back(t1);jp.names.push_back(_name+"_s0");
    jp.angles.push_back(t2);jp.names.push_back(_name+"_s1");
    jp.angles.push_back(0);jp.names.push_back(_name+"_e0");
    jp.angles.push_back(t3);jp.names.push_back(_name+"_e1");
    jp.angles.push_back(0);jp.names.push_back(_name+"_w0");
    jp.angles.push_back(t4);jp.names.push_back(_name+"_w1");
    jp.angles.push_back(w2_new);jp.names.push_back(_name+"_w2");
    return jp;
}

void BaxterLimb::set_simple_positions(gv::Point pt, double w2, ros::Duration timeout=ros::Duration(5) ) {
    // move to xyz position using simple IK, position controller
    JointPositions jp;
    jp = get_simple_positions(pt, w2);
    this->to_position(jp, timeout);
}

JointPositions BaxterLimb::get_service_position(gv::RPYPose x) {
    // get set of joint angles that will satisfies the desired pose

    // setup service
    std::string serv = "sdk/robot/limb/"+_name+"/solve_ik_position";
    ros::ServiceClient client = _nh.serviceClient<baxter_msgs::SolvePositionIK>(serv);
    // setup message for service call
    baxter_msgs::SolvePositionIK srv;
    geometry_msgs::PoseStamped posestamp;
    gm::Pose mypose;
    mypose.position = x.position;
    mypose.orientation = toQuat(x.rpy);
    posestamp.pose.position = mypose.position;
    posestamp.pose.orientation = mypose.orientation;
    posestamp.header.frame_id = "base";
    srv.request.pose_stamp.push_back(posestamp);
    JointPositions positions;
    if(client.call(srv)) {
        // if service call returns a response
        for(int i = 0; i < srv.response.isValid.size(); i++) {
            // there may be multiple responses -- I've only ever seen one
            if(srv.response.isValid[i]) {
                // we end up just taking the last one
                positions.names = srv.response.joints[i].names;
                positions.angles = srv.response.joints[i].angles;
            }
            else {
                ROS_ERROR("Not a valid pose");
                positions.angles.clear();
            }
        }
    }
    else
        ROS_ERROR("Failed to call SolvePositionIK service");
    
    return positions;
}

int BaxterLimb::set_service_position_quick(gv::RPYPose mypose, ros::Duration timeout) {
    // move to position using service IK, quick velocity controller
    JointPositions positions = get_service_position(mypose);
    if(positions.angles.empty())
        return -1;
    
    return quickly_to_position(positions, timeout);
}

int BaxterLimb::set_service_position_accurate(gv::RPYPose mypose, ros::Duration timeout) {
    // move to position using service IK, accurate velocity controller
    JointPositions positions = get_service_position(mypose);
    if(positions.angles.empty())
        return -1;
    
    return accurate_to_position(positions, timeout);
}

int BaxterLimb::set_service_position(gv::RPYPose mypose, ros::Duration timeout) {
    // move to position using service IK, position controller
    JointPositions positions = get_service_position(mypose);
    if(positions.angles.empty())
        return -1;
    
    return to_position(positions, timeout);
}

JointVelocities BaxterLimb::get_velocities(JointPositions desired) {
    // get set of proportional velocities to move towards desired joint positions from current
    std::vector<double> position = desired.angles;
    std::vector<double> current = joint_angles();
    std::vector<double> error = v_difference(current, position);
    std::vector<double> integral(position.size(),0);
    std::vector<double> derivative(position.size(),0);
    
    JointVelocities output;
    output.names = desired.names;
    ros::spinOnce();

    error = v_difference(position, joint_angles());
    output.velocities = compute_gains(error, integral, derivative, FAST);
    
    limit_velocity(output.velocities);
   
    return output;
}

int BaxterLimb::set_velocities(JointPositions desired) {
    // set proportional velocities to move towards desired joint positions from current
    // non blocking
    JointVelocities output = get_velocities(desired);
    
    set_joint_velocities(output);
    ros::spinOnce();
    return 1;
}

void BaxterLimb::limit_velocity(std::vector<double> &vel) {
    // limits joint velocities to specified range
    for(int i = 0; i < vel.size(); i++) {
        vel[i] = limit(_max_velocity[i], vel[i]);
    }
}

void BaxterLimb::limit_acceleration(std::vector<double> &vel, std::vector<double> last, double sec) {
    // limit joint acceleration to specified range
    double accel;
    for(int i = 0; i < last.size(); i++) {
        // a = dv/dt
        accel = (vel[i] - last[i])/sec;
        if (fabs(accel) > _max_acceleration[i]) {
            vel[i] = _max_acceleration[i]*sec*(vel[i] < 0 ? -1:1) + last[i];
        }
        vel[i] = limit(_max_velocity[i], vel[i]);
    }
}

std::vector<double> BaxterLimb::compute_gains(std::vector<double> error, std::vector<double> integral, std::vector<double> derivative, bool quick) {
    // return controller values for specified errors
    // FAST controller uses no integral term
    // SLOW controller uses fraction of gains
    std::vector<double> temp(error.size(),0);
    if (quick){
        //NO Integral term for this
        for(int i = 0; i < error.size(); i++) {
            temp[i] = error[i]*_pid[i].kp;
            //temp[i] += integral[i]*_pid[i].ki;
            temp[i] += derivative[i]*_pid[i].kd;
        }
    }
    else{
        for(int i = 0; i < error.size(); i++) {
            temp[i] = error[i]*_pid[i].kp/3;
            temp[i] += integral[i]*_pid[i].ki/3;
            temp[i] += derivative[i]*_pid[i].kd/3;
        }
    }
    return temp;
}

//returns true if error[i] < _allowed_error[i] for all i
bool BaxterLimb::_in_range(std::vector<double> error, bool quick) {
    // REMOVE THIS?? ONLY USE PUBLIC VERSION
    
    // stricter limits on allowed error for slow controller
    bool temp = true;
    if(quick) {
        for(int i = 0; i < error.size(); i++) {
            temp = temp && (fabs(error[i]) < _allowed_error[i]);
            
        }
    }
    else {
        for(int i = 0; i < error.size(); i++) {
            temp = temp && (fabs(error[i]) < _allowed_error[i]/4);
        }
    }
    return temp;
}

const char* BaxterLimb::_max_error(std::vector<double> err) {
    // return name, error of maximum error
    int index = 0;
    int max = 0;
    for(int i = 0; i <err.size(); i++) {
        if (fabs(err[i]) > max) {
            index = i;
            max = fabs(err[i]);
        }
    }
    std::stringstream ss;
    ss << _joint_names[index] << ": " << err[index];
    return ss.str().c_str();
}

// convert between secs, nsecs
double toSec(int nsec) {
    return double(nsec)/1000000000;
}

int toNsec(double sec) {
    return sec*1000000000;
}

void _saturate(std::vector<double> &integral, double sat_limit) {
    // saturate the integral term
    // to prevent windup
    for(int i = 0; i < integral.size(); i++) {
        integral[i] = (fabs(integral[i]) > sat_limit ? sat_limit*(integral[i]<0?-1:1) : integral[i]);
    }
}

void _reset_integral(std::vector<double> &integral, const std::vector<double> &error, const std::vector<double> &previous_error) {
    // resets the integral term if error has flipped sign
    //   ( position has passed reference )
    for(int i = 0; i < integral.size(); i++) {
        // if error*prev_error < 0, error has flipped sign
        // we will reset the integral term to prevent windup
        if(error[i]*previous_error[i] < 0)
            integral[i] = 0;
    }
}

void to_vector(const gv::RPYPose &pose, Eigen::VectorXd &vec) {
    // this converts a pose{x,y,z,r,p,y} to a eigen vector 
    vec(0) = pose.position.x;
    vec(1) = pose.position.y;
    vec(2) = pose.position.z;
    vec(3) = pose.rpy.roll;
    vec(4) = pose.rpy.pitch;
    vec(5) = pose.rpy.yaw;
}

void from_vector(std::vector<double> &to, const Eigen::VectorXd &from, int size) {
    //this converts an Eigen::Vector to a std::vector
    to.clear();
    for(int i = 0; i < size; i++)
    {
        to.push_back(from(i));
    }
}


int BaxterLimb::set_jacobian_position(gv::RPYPose desired, ros::Duration timeout) {
    // EXPERIMENTAL -- currently unstable
    // move to desired position using a jacobian based controller - KDL, eigen
    // tries to be accurate
    
    // desired rate
    double hz = 100;
    ros::Time last = ros::Time::now();
    ros::Rate r(hz); 
    ros::Duration dt;
    
    //setup variables
    gv::RPYPose endpoint_err;
    std::vector<double> position = joint_angles();
    std::vector<double> current = joint_angles();
    std::vector<double> error (position.size(),1);
    std::vector<double> previous_error(position.size(),0);
    std::vector<double> integral(position.size(),0);
    std::vector<double> derivative(position.size(),0);
    std::vector<double> last_vel = joint_velocities();
    std::vector<double> accel(position.size(),0);
    Eigen::VectorXd x(7);       // joint velocities
    //Eigen::VectorXd last_vel(7);
    Eigen::VectorXd b(6);       // endpoint error
    
    JointVelocities output;
    output.names = joint_names();
    output.velocities = joint_velocities();  
    KDL::JntArray jnts;
    desired.print("Desired Position");
    
    // lock some joints
    ros::Time start  = ros::Time::now();
    std::vector<std::string> lock;
    lock.push_back("right_w2");
    lock_joints(lock);
    output.remove(lock);
    _jacobian.resize(7 - lock.size());
    
    Eigen::MatrixXd A;
    set_jacobian();
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(_jacobian.data, Eigen::ComputeFullU | Eigen::ComputeFullV);
    
    // loop until arrived, timeout, cancellation
    while( !(_in_range(error, SLOW)) && ( ros::Time::now() - start < timeout ) && ros::ok()) {
        ros::spinOnce();
        dt = ros::Time::now() - last;
        last = ros::Time::now();
        
        //  set jacobian
        set_jacobian();
        //  setup solver
        svd.compute(_jacobian.data);
        
        //  Set endpoint error, sum PID components
        //gv::RPYPose(endpoint_pose()).print();
        endpoint_err = desired - endpoint_pose();
        endpoint_err.print();
        to_vector(endpoint_err, b);
        
        // solve jacobian
        x = svd.solve(b);
        from_vector(error, x, output.velocities.size());
        
        // calculate controller values
        integral = v_sum(integral, product(error, toSec(dt.nsec)));
        _saturate(integral, 0);
        derivative = quotient(v_difference(error, previous_error), toSec(dt.nsec));
        output.velocities = compute_gains(error, integral, derivative, FAST);
        
        limit_acceleration(output.velocities, last_vel, toSec(dt.nsec));
        
        set_joint_velocities(output);
        
        last_vel = output.velocities;
        previous_error = error;        
        r.sleep();
    }
    this->exit_control_mode();
    
    if(_in_range(error, SLOW))
        return 1;
    
    ROS_ERROR("Timeout [%.2fs]: jacobian to position",timeout.sec+toSec(timeout.nsec));
    return -1;
}

int BaxterLimb::quickly_to_position(JointPositions desired, ros::Duration timeout) {
    // moves quickly to desired position using velocity controller
    double hz = 100;
    ros::Time start  = ros::Time::now();
    ros::Time last = ros::Time::now();
    ros::Rate r(hz); 
    ros::Duration dt;
    
    std::vector<double> position = desired.angles;
    std::vector<double> current = joint_angles();
    std::vector<double> error = v_difference(current, position);
    std::vector<double> previous_error(position.size(),0);
    std::vector<double> integral(position.size(),0);
    std::vector<double> derivative(position.size(),0);
    std::vector<double> last_vel = joint_velocities();
    std::vector<double> accel(position.size(),0);
    
    JointVelocities output;
    output.names = desired.names;
    
    // loop until arrived, timeout, cancellation
    while( !_in_range(error,FAST) && ( ros::Time::now() - start < timeout ) && ros::ok()) {
        dt = ros::Time::now() - last;
        dt.nsec = (toSec(dt.nsec) < 1/hz ? toNsec(1/hz) : dt.nsec);
        last = ros::Time::now();
        
        error = v_difference(position, joint_angles());
        integral = v_sum(integral, product(error, toSec(dt.nsec)));
        _saturate(integral, 0.75);
        derivative = quotient(v_difference(error, previous_error), toSec(dt.nsec));
        output.velocities = compute_gains(error, integral, derivative, FAST);
        
        limit_acceleration(output.velocities, last_vel, toSec(dt.nsec));
        
        set_joint_velocities(output);
        
        last_vel = output.velocities;
        previous_error = error;    
        
        ros::spinOnce();
        r.sleep();
    }
    this->exit_control_mode();
    if(_in_range(error, FAST))
        return 1;
    
    ROS_ERROR("Timeout: moving quickly to position. Max error at [ %s ]", _max_error(error));
    return -1;
}

int BaxterLimb::accurate_to_position(JointPositions desired, ros::Duration timeout) {
    // moves accurately to desired position using velocity controller
    double hz = 100;
    desired.print("desired joint angles");
    this->joint_positions().print("current joint angles");
    ros::Time start  = ros::Time::now();
    ros::Time last = ros::Time::now();
    ros::Rate r(hz); 
    ros::Duration dt;
    
    std::vector<double> position = desired.angles;
    std::vector<double> current = joint_angles();
    std::vector<double> error = v_difference(current, position);
    std::vector<double> previous_error(position.size(),0);
    std::vector<double> integral(position.size(),0);
    std::vector<double> derivative(position.size(),0);
    std::vector<double> last_vel = joint_velocities();
    std::vector<double> accel(position.size(),0);
    
    JointVelocities output;
    output.names = desired.names;
    
    // loop until arrived, timeout, cancellation
    while( !_in_range(error,SLOW) && ( ros::Time::now() - start < timeout ) && ros::ok()) {
        dt = ros::Time::now() - last;
        dt.nsec = (toSec(dt.nsec) < 1/hz ? toNsec(1/hz) : dt.nsec);
        last = ros::Time::now();
        
        error = v_difference(position, joint_angles());
        integral = v_sum(integral, product(error, toSec(dt.nsec)));
        _saturate(integral, .2);
        _reset_integral(integral, error, previous_error);
        derivative = quotient(v_difference(error, previous_error), toSec(dt.nsec));
        output.velocities = compute_gains(error, integral, derivative, SLOW);
        
        limit_acceleration(output.velocities, last_vel, toSec(dt.nsec));
        
        output.names = desired.names;
        
        set_joint_velocities(output);
        
        last_vel = output.velocities;
        previous_error = error;        
        
        ros::spinOnce();
        r.sleep();
    }
    this->exit_control_mode();
    
    if(_in_range(error, SLOW))
        return 1;
    
    ROS_ERROR("Timeout: moving accurately to position. Max error at [ %s ]", _max_error(error));
    return -1;
}

int BaxterLimb::to_position(JointPositions desired, ros::Duration timeout) {
    // position controller

    double hz = 100;
    bool speed = FAST;
    ros::Time start  = ros::Time::now();
    ros::Rate r(hz); 
    std::vector<double> position = desired.angles;
    std::vector<double> current = joint_angles();
    std::vector<double> error = v_difference(current, position);
    
    // loop until arrived, timeout
    while( !_in_range(error,speed) && ( ros::Time::now() - start < timeout ) ) {
        set_joint_positions(desired);   
        error = v_difference(this->joint_angles(), desired.angles);
        ros::spinOnce();
        r.sleep();
    }
    if(_in_range(error, speed))
        return 1;
    
    ROS_ERROR("Timeout: moving to position. Max error at [ %s ]", _max_error(error));
    return -1;
}

/*------------------------------------------------------------------
 *     Here is KDL Stuff
 *-----------------------------------------------------------------*/

void BaxterLimb::_set_shoulder_mount() {
    // Finds xyz position of the shoulder mount
    KDL::Frame base;
    base = KDL::Frame::Identity();
    KDL::Vector vec;
    std::string name;
    int i = 0;
    
    // move through all stationary joints in kdl chain
    while (_chain.getSegment(i).getJoint().getType() == KDL::Joint::None) {
        name = _chain.getSegment(i).getJoint().getName();
        vec = _chain.getSegment(i).pose(0.0).p;
        base = base*_chain.getSegment(i).pose(0.0);
        i++;
    }
    // move to shoulder mount kdl frame
    base = base*_chain.getSegment(i).pose(0.0);
    
    // get position of shoulder mount
    KDL::Vector pos = base.p;
    gv::Point basePos;
    basePos.x = pos[0];
    basePos.y = pos[1];
    basePos.z = pos[2];
    _shoulder_mount = basePos;
}

void BaxterLimb::_check_kdl() {
    // This checks that the endpoint position calculated using kdl is accurate compared
    // to the value provided by /endpoint_state
    
    // update variables -- _kdl_jointpositions needs to be set
    ros::spinOnce();
    KDL::ChainFkSolverPos_recursive fk(_chain);
    KDL::Frame frame;
    // kdl forward kinematic solver
    int pos = fk.JntToCart(_kdl_jointpositions, frame);
    if (pos < 0){
        ROS_ERROR("KDL's FK solver failed");
        return;
    }
    
    // Need to account for finger length in endpoint pose for forward solver
    // finger length should be ~7cm BUT
    //  not accounting for finger length I'm consistently off by 2cm
    // So that's what I'm doing here
    KDL::Frame finger_frame( KDL::Vector(0, 0, 0.02) );
    frame = frame*finger_frame;
    
    // get pose from baxter subscriber
    gv::RPYPose from_baxter(endpoint_pose());
    double roll,pitch,yaw;
    double x,y,z;
    
    // KDL::Frame has origin vector p and rotation matrix M
    frame.M.GetRPY(roll,pitch,yaw);
    x = frame.p.x();
    y = frame.p.y();
    z = frame.p.z();
    gv::RPYPose from_fk(x,y,z,roll,pitch,yaw);
    gv::RPYPose err = from_fk - from_baxter;
    
    // this is pretty loose right now
    gv::RPYPose allow(gv::Point(0.02), gv::RPY(0.3));
    // check if in range
    bool inRange = err.abs() < allow;
    if (inRange)
        return;
    ROS_ERROR("Discrepancy between KDL calculated endpoint position and that provided by topic [/endpoint/state]");
}

void BaxterLimb::lock_joints(std::vector<int> to_lock) {
    // this locks the specified joints for KDL's jacobian solver
    std::stringstream ss;
    for (int i = 0 ; i < _locked_joints.size(); i++) {
        for(int j = 0; j < to_lock.size(); j++) {
            if (i == to_lock[j]) {
                ss << i << ", ";
                _locked_joints[i] = true;
                break;
            }
        }
    }
    int check = _jacobian_solver->setLockedJoints(_locked_joints);
    if (check < 0)
        ROS_ERROR("The Jacobian solver expected a different number of joints \
            than were supplied for the locking operation");
    std::string info = ss.str();
    ROS_INFO("Locked joints [%s]", info.substr(0,info.size()-2).c_str());
}

void BaxterLimb::lock_joints(std::vector<std::string> names) {
    // this locks the specified joints for KDL's jacobian solver
    std::stringstream ss;
    std::vector<std::string> joints = joint_names();
    for (int i = 0 ; i < names.size(); i++) {
        for(int j = 0; j < _locked_joints.size(); j++) {
            if (joints[j] == names[i]) {
                ss << names[i] << ", ";
                _locked_joints[j] = true;
                break;
            }
        }
    }
    int check = _jacobian_solver->setLockedJoints(_locked_joints);
    if (check < 0)
        ROS_ERROR("The Jacobian solver expected a different number of joints \
            than were supplied for the locking operation");
    std::string info = ss.str();
    ROS_INFO("Locked joints [%s]", info.substr(0,info.size()-2).c_str());
}

void BaxterLimb::unlock_joints() {
    // this unlocks all joints for KDL's jacobian solver
    std::vector<bool> locked(NUMJOINTS, false);
    _locked_joints = locked;
    int check = _jacobian_solver->setLockedJoints(_locked_joints);
    if (check < 0)
        ROS_ERROR("The Jacobian solver expected a different number of joints \
            than were supplied for the unlocking operation");
    ROS_INFO("Unlocked all joints");
}

void BaxterLimb::unlock_joints(std::vector<int> to_unlock)
{
    // unlocks the provided joints for KDL's jacobian solver
    std::stringstream ss;
    for (int i = 0 ; i < _locked_joints.size(); i++) {
        for(int j = 0; j < to_unlock.size(); j++) {
            if (i == to_unlock[j]) {
                ss << i << ", ";
                _locked_joints[i] = false;
                break;
            }
        }
    }
    int check = _jacobian_solver->setLockedJoints(_locked_joints);
    if (check < 0)
        ROS_ERROR("The Jacobian solver expected a different number of joints \
            than were supplied for the unlocking operation");
    std::string info = ss.str();
    ROS_INFO("Unlocked joints [%s]", info.substr(0,info.size()-2).c_str());
}
void BaxterLimb::unlock_joints(std::vector<std::string> to_unlock) {
    // this unlocks the provided joints for KDL's jacobian solver
    std::stringstream ss;
    std::vector<std::string> joints = joint_names();
    for (int i = 0 ; i < to_unlock.size(); i++) {
        for(int j = 0; j < _locked_joints.size(); j++) {
            if (joints[j] == to_unlock[i]) {
                ss << to_unlock[i] << ", ";
                _locked_joints[j] = false;
                break;
            }
        }
    }
    int check = _jacobian_solver->setLockedJoints(_locked_joints);
    if (check < 0)
        ROS_ERROR("The Jacobian solver expected a different number of joints \
            than were supplied for the unlocking operation");
    std::string info = ss.str();
    ROS_INFO("Unlocked joints [%s]", info.substr(0,info.size()-2).c_str());
}

KDL::Jacobian BaxterLimb::jacobian() {
    // Sets jacobian for current position. Prints jacobian to screen. Optional return 
    set_jacobian();
    std::cout << "-- Jacobian for [" << _name << "] limb--\n";
    std::cout<< _jacobian.data << "\n";
    return _jacobian;
}

KDL::Jacobian BaxterLimb::get_jacobian() {
    // Sets and return jacobian for current position
    set_jacobian();
    return _jacobian;
}

void BaxterLimb::set_jacobian() {
    // sets jacobian for current position
    _jacobian_solver->JntToJac(_kdl_jointpositions, _jacobian);
}

void BaxterLimb::resize_jacobian(int size) {
    // Resizes jacobian to specified size. 
    //   Default Jacobian is 7x6 (7 dof, 6 state {x,y,z,r,p,y})
    _jacobian.resize(size);
}

void displayJointPositions(const KDL::JntArray& pos) {
    // Prints Joint Positions in the KDL JntArray in Degrees
    for(int i =0; i < pos.rows(); i++)
    {
        std::cout<<pos(i)*180/PI<<"\n";
    }
    std::cout<<"----\n";
}


/*----------------------------------------------------------------------
 * Everything after this line is not a part of the BaxterLimb Class
 * ---------------------------------------------------------------------*/


void v_print(std::vector<double> x) {
    for(int i = 0; i < x.size(); i++) {
        std::cout<<"Vector member ";
        std::cout<<i;
        std::cout<<": ";
        std::cout<<x[i]<<std::endl;
    }
}

void v_print(std::vector<double> x, std::string name) {
    for(int i = 0; i < x.size(); i++) {
        std::cout<< name << " member ";
        std::cout<<i;
        std::cout<<": ";
        std::cout<<x[i]<<std::endl;
    }
}

// Math functions named v_<name> are performed on two vector doubles
// other functions use one vector double and one double
double limit(double max, double val) {
    if (fabs(val) > max)
        return max*double((val < 0) ? -1 : 1);
    return val;
}

std::vector<double> v_difference(std::vector<double> x, std::vector<double> y) {
    std::vector<double> temp;
    for(int i = 0; i < x.size(); i++) {
        temp.push_back(x[i] - y[i]);
    }
    return temp;
}

std::vector<double> v_sum(std::vector<double> x, std::vector<double> y) {
    std::vector<double> temp;
    for(int i = 0; i < x.size(); i++) {
        temp.push_back(x[i] + y[i]);
    }
    return temp;
}

std::vector<double> product(std::vector<double> x, double y) {
    std::vector<double> temp;
    for(int i = 0; i < x.size(); i++) {
        temp.push_back(x[i]*y);
    }
    return temp;
}

std::vector<double> quotient(std::vector<double> x, double y) {
    std::vector<double> temp;
    double q;
    for(int i = 0; i < x.size(); i++) {
        if(x[i] == 0)
            q = 0;
        else if (y == 0)
            q = 1000*(x[i] > 0 ? 1 : -1);
        else
            q = x[i]/y;
        temp.push_back(q);
    }
    return temp;
}

#endif
