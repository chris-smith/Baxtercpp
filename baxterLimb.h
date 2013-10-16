#include <Eigen/SVD>
#include <Eigen/LU>
#include <kdl/segment.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/treejnttojacsolver.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jntarray.hpp>

#include "ros/ros.h"
#include <iostream>
#include <string>
#include <stdio.h>
#include <sstream>
#include <baxter_msgs/JointCommandMode.h>
#include <baxter_msgs/JointPositions.h>
#include <baxter_msgs/JointVelocities.h>
#include <sensor_msgs/JointState.h>
#include <baxter_msgs/EndpointState.h>
#include <baxter_msgs/SolvePositionIK.h>
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


gv::Quaternion toQuat(gv::PRY);                             //convert pitch, roll, yaw to quaternion
gv::PRY toPRY(gv::Quaternion);                              //convert quaternion to pitch, roll, yaw
double toSec(int);
int toNsec(double);


#ifndef VECTOR_MATH
#define VECTOR_MATH
std::vector<double> v_difference(std::vector<double>,std::vector<double>);
std::vector<double> v_sum(std::vector<double>,std::vector<double>);
std::vector<double> product(std::vector<double>, double);
std::vector<double> quotient(std::vector<double>, double);
void v_print(std::vector<double>);
void v_print(std::vector<double>, std::string);
double limit(double, double); //limits val to max. preserves sign
void to_file(std::vector<double>, std::string, bool);
#endif

#ifndef BAXTER_LIMB
#define BAXTER_LIMB

#define numJoints 7
#define FAST true
#define SLOW false

struct Gains{ double kp; double ki; double kd; };
std::string joint_ids[numJoints] = {"_s0","_s1","_e0","_e1","_w0","_w1","_w2"};

class BaxterLimb
{
public:
    BaxterLimb(KDL::Tree, std::string,uint);       //node handle, side, buffer size
    BaxterLimb(std::string);                            //side
    ~BaxterLimb();
    
    BaxterGripper* gripper;
    
    void Init();
    
    std::vector<double> joint_angles();                 //return _joint_angle
    std::vector<double> joint_velocities();             //return _joint_velocity
    std::vector<double> joint_efforts();                //return _joint_effort
    std::vector<std::string> joint_names();             //return _joint_names
    gm::Pose endpoint_pose();                               //return endpoint pose
    gm::Twist endpoint_velocity();                          //return endpoint velocity
    gm::Wrench endpoint_effort();                           //return endpoint effort
    unsigned short state_rate();                        //return _state_rate
    
    void set_joint_position_mode();                     //set limb to use position controller
    void set_joint_velocity_mode();                     //set limb to use velocity controller
    void set_joint_positions(JointPositions);           //moves using position controller. Sets mode
    void set_joint_velocities(JointVelocities);         //moves using velocity controller. Sets mode
    void set_joint_pid(std::string, Gains);             //set _pid gains for joint
    Gains get_joint_pid(std::string joint);             //get _pid value for joint
    void set_endpoint_pid(Gains);                       //set _endpoint_pid
    void set_allowable_error(std::vector<double>);      //set maximum errors allowed to be considered "there"
    void set_allowable_error(double);                   //set blanket maximum error allowed to be considered "there"
    gv::PRYPose endpoint_error;                         //allowable error when moving by jacobian
    void set_max_velocity(std::vector<double>);         //set maximum velocity allowed for each joint
    void set_max_velocity(double);                      //set blanket maximum velocity allowed for each joint
    void set_max_acceleration(std::vector<double>);     //set maximum acceleration allowed for each joint
    void set_max_acceleration(double);                  //set blanket maximum acceleration allowed for each joint
    
    JointPositions get_position(gv::PRYPose);               //returns joint angles to reach PRYPose
    int set_position_quick(gv::PRYPose, ros::Duration);    //moves endpoint to the specified position quickly
    int set_position_accurate(gv::PRYPose, ros::Duration); //moves endpoint to the specified position accurately
    int set_position(gv::PRYPose, ros::Duration);          //moves endpoint to the specified position using position controller
    int set_endpoint_velocity(gv::PRYPose);                //calls set_velocities after getting JointPosition exits
    int set_velocities(JointPositions desired);            //sets velocities once
    JointVelocities get_velocities(JointPositions); // return velocities computed to reach position
    /*    Controllers
     *      
     * uses velocity controller to move to specified positions
     * until error for joints is <= _allowed_error or until timeout
     * returns -1 if timeout occured. 
     ************************************************************/
    int quickly_to_position(JointPositions, ros::Duration); 
    /* Similar to the above except the following
     *  moves until joint error is <= _allowed_error/4
     *  uses 0.5(_pid)
     *****************************************************/
    int accurate_to_position(JointPositions, ros::Duration);
    /* Moves to position until with error or timeout
     *  using position controller
     * ***************************************************/
    int to_position(JointPositions, ros::Duration);
    
    int endpoint_control(gm::Point); //add point to current endpoint state, sets velocities, exits
    
    //  Jacobian controller
    int jacobian_to_position(gv::PRYPose, ros::Duration);
    void lock_joints(std::vector<int>);                 // lock joints for jacobian solver by number
    void lock_joints(std::vector<std::string>);         // lock joints for jacobian solver by name                
    void unlock_joints();                               // unlocks all joints
    void unlock_joints(std::vector<int>);
    void unlock_joints(std::vector<std::string>);
    
    //computes pid. If bool is true, does so for "quick". If false, for "accurate"
    //Receives arguments error, integral, derivative, FAST/SLOW
    std::vector<double> compute_gains(std::vector<double>, std::vector<double>, std::vector<double>, bool);
    bool in_range(std::vector<double>); //exposed version of _in_range. assumes fast
    bool endpoint_in_range(gv::PRYPose);
private:
    BaxterLimb();
    bool _set; //true if any subscribers have returned data yet
    
    std::string _name; //name of limb (left, right)
    std::string _joint_names[numJoints]; //explicit joint names
    double _joint_angle[numJoints];
    double _joint_velocity[numJoints];
    double _joint_effort[numJoints];
    double _allowed_error[numJoints]; // error to be considered "there" for velocity control
    double _max_velocity[numJoints]; // strictly >= 0
    double _max_acceleration[numJoints]; // strictly >= 0
    Gains _pid[numJoints];
    gm::Pose _cartesian_pose;
    gm::Twist _cartesian_velocity;
    gm::Wrench _cartesian_effort;
    baxter_msgs::JointCommandMode _pub_mode;   
    
    ros::NodeHandle _nh; //node handle used to setup subscriber if one is not provided
    unsigned int _bufferSize; //size of subscriber buffer
    ros::Subscriber _joint_state_sub;
    ros::Subscriber _cartesian_state_sub;
    ros::Publisher _pub_joint_mode;
    ros::Publisher _pub_joint_position;
    ros::Publisher _pub_joint_velocity;
    unsigned short _state_rate;
    ros::Time _last_state_time;
    
    // KDL & Eigen
    bool _kdl_set;              // true if kdl chain has been set from tree
    Gains _endpoint_pid;
    std::vector<bool> _locked_joints;
    Eigen::VectorXd _eigen_endpoint; 
    KDL::Chain _chain;
    KDL::Jacobian _jacobian;
    KDL::ChainJntToJacSolver *_jacobian_solver;
    
    void _set_names(); //sets _joint_names in constructor
    void _on_endpoint_states(const baxter_msgs::EndpointState::ConstPtr&); //callback for /robot/joint_states
    void _on_joint_states(const sensor_msgs::JointState::ConstPtr&); //callback for /endpoint_state
    bool _in_range(std::vector<double>, bool); //checks if error < _allowed_error
    void _limit_velocity(std::vector<double>&); //v = (v > _max_vel ? _max_vel : v)
    void _limit_acceleration(std::vector<double>&, std::vector<double>, double); //ensures velocity not changing too fast
    
    const char* _max_error(std::vector<double>); //returns joint name and error of largest error
    
};


BaxterLimb::BaxterLimb()
{
    ROS_ERROR("Private Default Constructor -- Should never be called");
}

BaxterLimb::BaxterLimb(KDL::Tree tree,std::string name,uint bufferSize)
{
    _name = name;
    _bufferSize = bufferSize;
    _kdl_set = true;
    std::string endpoint = name + "_gripper";
    tree.getChain("torso", endpoint, _chain);
    Init();
}

BaxterLimb::BaxterLimb(std::string name)
{
    _name = name;
    _kdl_set = false;
    _bufferSize = 1;
    Init();
}

BaxterLimb::~BaxterLimb()
{
    delete gripper;
    delete _jacobian_solver;
}

void BaxterLimb::Init()
{
    ros::Rate r(10);
    _set = false;
    _set_names();
    _state_rate = 0;
    _pub_mode.mode = baxter_msgs::JointCommandMode::POSITION;
    _last_state_time = ros::Time::now();
    std::string topic = "/robot/limb/"+_name;
    _joint_state_sub = _nh.subscribe("/robot/joint_states", _bufferSize, &BaxterLimb::_on_joint_states, this);
    _cartesian_state_sub = _nh.subscribe(topic+"/endpoint/state", _bufferSize, &BaxterLimb::_on_endpoint_states, this);
    while (!_set)
    {
        ros::spinOnce();
        r.sleep();
    }
    _pub_joint_mode = _nh.advertise<baxter_msgs::JointCommandMode>(topic+"/joint_command_mode", 10);
    _pub_joint_position = _nh.advertise<baxter_msgs::JointPositions>(topic+"/command_joint_angles",10);
    _pub_joint_velocity = _nh.advertise<baxter_msgs::JointVelocities>(topic+"/command_joint_velocities",10);
    gripper = new BaxterGripper(_name);
    gripper->calibrate();
    if (_kdl_set)
    {
        int chain_joints = _chain.getNrOfJoints();
        if (chain_joints != numJoints)
        {
            ROS_ERROR("The number of joints in the kinematic chain is incorrect. \
             [%d] were expected, only [%d] were supplied.", numJoints, chain_joints);
            _kdl_set = false;
            return;
        }
        for(int j = 0; j < numJoints; j++)
            _locked_joints.push_back(false);
        _eigen_endpoint = Eigen::VectorXd(6);
        _jacobian_solver = new KDL::ChainJntToJacSolver(_chain);
        endpoint_error = gv::PRYPose(0.01);
        _endpoint_pid = (Gains){0,0,0};
    }
}

void BaxterLimb::lock_joints(std::vector<int> to_lock)
{
    std::stringstream ss;
    for (int i = 0 ; i < _locked_joints.size(); i++)
    {
        for(int j = 0; j < to_lock.size(); j++)
        {
            if (i == to_lock[j])
            {
                ss << i;
                _locked_joints[i] = true;
                break;
            }
        }
        if (i != _locked_joints.size() - 1)
            ss << ", ";
    }
    int check = _jacobian_solver->setLockedJoints(_locked_joints);
    if (check < 0)
        ROS_ERROR("The Jacobian solver expected a different number of joints \
            than were supplied for the locking operation");
    ROS_INFO("Locked joints [%s]", ss.str().c_str());
}

void BaxterLimb::lock_joints(std::vector<std::string> names)
{
    std::stringstream ss;
    std::vector<std::string> joints = joint_names();
    for (int i = 0 ; i < names.size(); i++)
    {
        for(int j = 0; j < _locked_joints.size(); j++)
        {
            if (joints[j] == names[i])
            {
                ss << names[i];
                _locked_joints[j] = true;
                break;
            }
        }
        if (i != names.size()-1)
            ss << ", ";
    }
    int check = _jacobian_solver->setLockedJoints(_locked_joints);
    if (check < 0)
        ROS_ERROR("The Jacobian solver expected a different number of joints \
            than were supplied for the locking operation");
    ROS_INFO("Locked joints [%s]", ss.str().c_str());
}

void BaxterLimb::unlock_joints()
{
    std::vector<bool> locked(numJoints, false);
    _locked_joints = locked;
    int check = _jacobian_solver->setLockedJoints(_locked_joints);
    if (check < 0)
        ROS_ERROR("The Jacobian solver expected a different number of joints \
            than were supplied for the unlocking operation");
    ROS_INFO("Unlocked all joints");
}

void BaxterLimb::unlock_joints(std::vector<int> to_unlock)
{
    std::stringstream ss;
    for (int i = 0; i < to_unlock.size(); i++)
    {
        for(int j = 0; j < _locked_joints.size(); j++)
        {
            if (j = to_unlock[i])
            {
                ss << j;
                _locked_joints[j] = false;
                break;
            }
        }
        if ( i != to_unlock.size()-1 )
            ss << ", ";
    }
    int check = _jacobian_solver->setLockedJoints(_locked_joints);
    if (check < 0)
        ROS_ERROR("The Jacobian solver expected a different number of joints \
            than were supplied for the unlocking operation");
    ROS_INFO("Unlocked joints [%s]", ss.str().c_str());
}
void BaxterLimb::unlock_joints(std::vector<std::string> to_unlock)
{
    std::stringstream ss;
    std::vector<std::string> joints = joint_names();
    for (int i = 0 ; i < to_unlock.size(); i++)
    {
        for(int j = 0; j < _locked_joints.size(); j++)
        {
            if (joints[j] == to_unlock[i])
            {
                ss << to_unlock[j];
                _locked_joints[j] = false;
                break;
            }
        }
        if ( i != to_unlock.size()-1 )
            ss << ", ";
    }
    int check = _jacobian_solver->setLockedJoints(_locked_joints);
    if (check < 0)
        ROS_ERROR("The Jacobian solver expected a different number of joints \
            than were supplied for the unlocking operation");
    ROS_INFO("Unlocked joints [%s]", ss.str().c_str());
}

void to_vector(const gv::PRYPose &pose, Eigen::VectorXd &vec)
{
    vec(0) = pose.position.x;
    vec(1) = pose.position.y;
    vec(2) = pose.position.z;
    vec(3) = pose.pry.roll;
    vec(4) = pose.pry.pitch;
    vec(5) = pose.pry.yaw;
}

int BaxterLimb::jacobian_to_position(gv::PRYPose desired, ros::Duration timeout)
{
    //std::cout<<"accurate"<<"\n";
    double hz = 100;
    ros::Time start  = ros::Time::now();
    ros::Time last = ros::Time::now();
    ros::Rate r(hz); 
    ros::Duration dt;
    /*std::vector<double> position;
    std::vector<double> current = joint_angles();
    std::vector<double> error = v_difference(current, position);
    std::vector<double> previous_error(position.size(),0);
    std::vector<double> integral(position.size(),0);
    std::vector<double> derivative(position.size(),0);
    std::vector<double> last_vel = joint_velocities();
    std::vector<double> accel(position.size(),0);*/
    gv::PRYPose current = endpoint_pose();
    gv::PRYPose error = current - desired;
    gv::PRYPose previous_error = error;
    gv::PRYPose summation(0);
    gv::PRYPose integral(0);
    gv::PRYPose derivative(0);
    KDL::JntArray jointpositions(7);
    Eigen::VectorXd x(7);       // joint velocities
    Eigen::VectorXd last_vel(7);
    Eigen::VectorXd b(6);       // endpoint error
    
    JointVelocities output;
    //output.names = desired.names;
    
    while( !endpoint_in_range(error) && ( ros::Time::now() - start < timeout ) && ros::ok())
    {
        dt = ros::Time::now() - last;
        dt.nsec = (toSec(dt.nsec) < 1/hz ? toNsec(1/hz) : dt.nsec);
        last = ros::Time::now();
        //  Get current jacobian
        _jacobian_solver->JntToJac(jointpositions, _jacobian);
        Eigen::JacobiSVD<Eigen::MatrixXd> svdOfA(_jacobian.data, Eigen::ComputeThinU | Eigen::ComputeThinV);
        //  Set endpoint error, sum PID components
        error = current - desired;
        integral = integral + error*toSec(dt.nsec);
        derivative = (error - previous_error)/toSec(dt.nsec);
        summation = error*_endpoint_pid.kp + integral*_endpoint_pid.ki \
            + derivative*_endpoint_pid.kd;
        to_vector(summation,b);
        //  Solve jacobian for joint velocity outputs, limit accordingly
        x = svdOfA.solve(b);
        //  Set velocities
        
        //_limit_acceleration(output.velocities, last_vel, toSec(dt.nsec));
        set_joint_velocities(output);
        
        last_vel = x;
        //previous_error = error;        
        
        ros::spinOnce();
        r.sleep();
    }
    //std::cout << "\nOperating time: ";
    //std::cout << ros::Time::now() - start << std::endl;
    if(endpoint_in_range(error))
        return 1;
    
    ROS_ERROR("Timeout: jacobian to position");
    return -1;
}

bool BaxterLimb::endpoint_in_range(gv::PRYPose current)
{
    current = current.abs();
    bool ret = true;
    ret = ret && current.position.x < endpoint_error.position.x;
    ret = ret && current.position.y < endpoint_error.position.y;
    ret = ret && current.position.z < endpoint_error.position.z;
    ret = ret && current.pry.pitch < endpoint_error.pry.pitch;
    ret = ret && current.pry.roll < endpoint_error.pry.roll;
    ret = ret && current.pry.yaw < endpoint_error.pry.yaw;
    return ret;
}

void BaxterLimb::_set_names()
{
    for(int i = 0; i < numJoints; i++)
    {
        _allowed_error[i] = 0.035; // ~2 degrees
        _joint_names[i] = _name+joint_ids[i];
        _pid[i] = (Gains){0,0,0};
        _max_velocity[i] = 1; //radians/sec
        _max_acceleration[i] = 100; //radians/sec^2
    }
}

void BaxterLimb::_on_joint_states(const sensor_msgs::JointState::ConstPtr& msg)
{
    _set = true;
    ros::Time now = ros::Time::now();
    double rate = 1.0/(now-_last_state_time).toSec();
    _state_rate = ((99.0*_state_rate) + rate)/100.0;
    
    _last_state_time = now;
    for(int i = 0; i < numJoints; i++)
    {
        int j = 0;
        while(msg->name[j] != _joint_names[i])
        {
            j++;
        }
        _joint_angle[i] = msg->position[j];
        _joint_velocity[i] = msg->velocity[j];
        _joint_effort[i] = msg->effort[j];
    }
}

void BaxterLimb::_on_endpoint_states(const baxter_msgs::EndpointState::ConstPtr& msg)
{
    _set = true;
    _cartesian_pose = msg->pose;
    _cartesian_velocity = msg->twist;
    _cartesian_effort = msg->wrench;
}

std::vector<std::string> BaxterLimb::joint_names()
{
    return std::vector<std::string>( _joint_names, _joint_names + sizeof _joint_names / sizeof _joint_names[0]);
}

std::vector<double> BaxterLimb::joint_angles()
{
    return std::vector<double>(_joint_angle, _joint_angle + sizeof _joint_angle / sizeof _joint_angle[0]);
}

std::vector<double> BaxterLimb::joint_velocities()
{
    return std::vector<double>(_joint_velocity, _joint_velocity + sizeof _joint_velocity / sizeof _joint_velocity[0]);
}

std::vector<double> BaxterLimb::joint_efforts()
{
    return std::vector<double>(_joint_effort, _joint_effort + sizeof _joint_effort / sizeof _joint_effort[0]);
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

void BaxterLimb::set_allowable_error(std::vector<double> new_err)
{
    for(int i = 0; i < numJoints; i++)
    {
        _allowed_error[i] = new_err[i];
    }
}

void BaxterLimb::set_allowable_error(double new_err)
{
    for(int i = 0; i < numJoints; i++)
    {
        _allowed_error[i] = new_err;
    }
}

void BaxterLimb::set_max_velocity(std::vector<double> max)
{
    for(int i = 0; i < numJoints; i++)
    {
        _max_velocity[i] = fabs(max[i]);
    }
}

void BaxterLimb::set_max_velocity(double max)
{
    for(int i = 0; i < numJoints; i++)
    {
        _max_velocity[i] = fabs(max);
    }
}

void BaxterLimb::set_max_acceleration(std::vector<double> max)
{
    for(int i = 0; i < numJoints; i++)
    {
        _max_acceleration[i] = fabs(max[i]);
    }
}

void BaxterLimb::set_max_acceleration(double max)
{
    for(int i = 0; i < numJoints; i++)
    {
        _max_acceleration[i] = fabs(max);
    }
}

void BaxterLimb::set_joint_position_mode()
{
    _pub_mode.mode = baxter_msgs::JointCommandMode::POSITION;
    _pub_joint_mode.publish(_pub_mode);
}

void BaxterLimb::set_joint_velocity_mode()
{
    _pub_mode.mode = baxter_msgs::JointCommandMode::VELOCITY;
    _pub_joint_mode.publish(_pub_mode);
}

//struct JointPositions{ std::vector<std::string> names; std::vector<double> angles; };
void BaxterLimb::set_joint_positions(JointPositions position)
{
    //ROS_INFO("Publishing %s", position);
    baxter_msgs::JointPositions msg;
    msg.names = position.names;
    msg.angles = position.angles;
    set_joint_position_mode();
    _pub_joint_position.publish(msg);
}

void BaxterLimb::set_joint_velocities(JointVelocities velocity)
{
    baxter_msgs::JointVelocities msg;
    msg.names = velocity.names;
    msg.velocities = velocity.velocities;
    set_joint_velocity_mode();
    _pub_joint_velocity.publish(msg);
}

void BaxterLimb::set_joint_pid(std::string name, Gains gain)
{
    // joint name must be explicit 
    // e.g left_e1 is ok
    //     e1 or _e1 is not ok
    int i = 0;
    while(_joint_names[i] != name)
    {
        i++;
        if(i >= numJoints)
        {
            ROS_ERROR("Tried to set PID for unknown joint [%s]", name.c_str());
            return;
        }
    }
    _pid[i] = gain;
}

Gains BaxterLimb::get_joint_pid(std::string name)
{
    // joint name must be explicit 
    // e.g left_e1 is ok
    //     e1 or _e1 is not ok
    int i = 0;
    Gains temp = {-1,-1,-1};
    while(_joint_names[i] != name)
    {
        i++;
        if(i >= numJoints)
        {
            ROS_ERROR("Tried to get PID for unknown joint [%s]", name.c_str());
            return temp;
        }
    }
    
    return _pid[i];
}

void BaxterLimb::set_endpoint_pid(Gains gain)
{
    _endpoint_pid = gain;
}

bool BaxterLimb::in_range(std::vector<double> error)
{
    bool temp = true;
    for(int i = 0; i < error.size(); i++)
    {
        temp = temp && (fabs(error[i]) < _allowed_error[i]);
        
    }
    return temp;
}

JointPositions BaxterLimb::get_position(gv::PRYPose x)
{
    std::string serv = "sdk/robot/limb/"+_name+"/solve_ik_position";
    ros::ServiceClient client = _nh.serviceClient<baxter_msgs::SolvePositionIK>(serv);
    baxter_msgs::SolvePositionIK srv;
    geometry_msgs::PoseStamped posestamp;
    gm::Pose mypose;
    mypose.position = x.position;
    mypose.orientation = toQuat(x.pry);
//     std::cout<<"\n";
    posestamp.pose.position = mypose.position;
//     std::cout<<mypose.point.x<<" ";
//     std::cout<<mypose.point.z<<" ";
    posestamp.pose.orientation = mypose.orientation;
//     std::cout<<mypose.quaternion.x<<" ";
    posestamp.header.frame_id = "base";
    srv.request.pose_stamp.push_back(posestamp);
//     std::cout<<"\n";
    JointPositions positions;
    if(client.call(srv))
    {
        for(int i = 0; i < srv.response.isValid.size(); i++){
            if(srv.response.isValid[i])
            {
                positions.names = srv.response.joints[i].names;
                positions.angles = srv.response.joints[i].angles;
            }
            else
            {
                ROS_ERROR("Not a valid pose");
                positions.angles.clear();
            }
        }
    }
    else
        ROS_ERROR("Failed to call SolvePositionIK service");
    
    return positions;
}

int BaxterLimb::set_position_quick(gv::PRYPose mypose, ros::Duration timeout)
{
    JointPositions positions = get_position(mypose);
    if(positions.angles.empty())
        return -1;
    
    return quickly_to_position(positions, timeout);
}

int BaxterLimb::set_position_accurate(gv::PRYPose mypose, ros::Duration timeout)
{
    JointPositions positions = get_position(mypose);
    if(positions.angles.empty())
        return -1;
    
    return accurate_to_position(positions, timeout);
}

int BaxterLimb::set_position(gv::PRYPose mypose, ros::Duration timeout)
{
    JointPositions positions = get_position(mypose);
    if(positions.angles.empty())
        return -1;
    
    return to_position(positions, timeout);
}

int BaxterLimb::set_endpoint_velocity(gv::PRYPose mypose)
{
    JointPositions desired = get_position(mypose);
    set_velocities(desired);
    
}

JointVelocities BaxterLimb::get_velocities(JointPositions desired)
{
    std::vector<double> position = desired.angles;
    std::vector<double> current = joint_angles();
    std::vector<double> error = v_difference(current, position);
    std::vector<double> integral(position.size(),0);
    std::vector<double> derivative(position.size(),0);
    
    JointVelocities output;
    output.names = desired.names;

    error = v_difference(position, joint_angles());
    output.velocities = compute_gains(error, integral, derivative, FAST);
    _limit_velocity(output.velocities);
   
    return output;
}

int BaxterLimb::set_velocities(JointPositions desired)
{
    std::vector<double> position = desired.angles;
    std::vector<double> current = joint_angles();
    std::vector<double> error = v_difference(current, position);
    std::vector<double> integral(position.size(),0);
    std::vector<double> derivative(position.size(),0);
    
    JointVelocities output;
    output.names = desired.names;

    error = v_difference(position, joint_angles());
    output.velocities = compute_gains(error, integral, derivative, SLOW);
    _limit_velocity(output.velocities);
    set_joint_velocities(output);
    ros::spinOnce();
    return 1;
}

void BaxterLimb::_limit_velocity(std::vector<double> &vel)
{
    for(int i = 0; i < vel.size(); i++)
    {
        vel[i] = limit(_max_velocity[i], vel[i]);
    }
}

void BaxterLimb::_limit_acceleration(std::vector<double> &vel, std::vector<double> last, double sec)
{
    double accel;
    for(int i = 0; i < last.size(); i++)
    {
        accel = (vel[i] - last[i])/sec;
        if (fabs(accel) > _max_acceleration[i])
        {
            vel[i] = _max_acceleration[i]*sec*(vel[i] < 0 ? -1:1) + last[i];
            //std::cout << " \tadj: " << vel[i];
        }
        vel[i] = limit(_max_velocity[i], vel[i]);
    }
}

std::vector<double> BaxterLimb::compute_gains(std::vector<double> error, std::vector<double> integral, std::vector<double> derivative, bool quick)
{
    //temp = kp*error + ki*integral + kd*derivative
    std::vector<double> temp(error.size(),0);
    if (quick){
        //NO Integral term for this
        for(int i = 0; i < error.size(); i++)
        {
            temp[i] = error[i]*_pid[i].kp;
            //temp[i] += integral[i]*_pid[i].ki;
            temp[i] += derivative[i]*_pid[i].kd;
        }
    }
    else{
        for(int i = 0; i < error.size(); i++)
        {
            temp[i] = error[i]*_pid[i].kp/2;
            temp[i] += integral[i]*_pid[i].ki/2;
            temp[i] += derivative[i]*_pid[i].kd/2;
        }
    }
    return temp;
}

//returns true if error[i] < _allowed_error[i] for all i
bool BaxterLimb::_in_range(std::vector<double> error, bool quick)
{
    bool temp = true;
    if(quick)
    {
        for(int i = 0; i < error.size(); i++)
        {
            temp = temp && (fabs(error[i]) < _allowed_error[i]);
            
        }
    }
    else{
        for(int i = 0; i < error.size(); i++)
        {
            temp = temp && (fabs(error[i]) < _allowed_error[i]/4);
        }
    }
    return temp;
}

const char* BaxterLimb::_max_error(std::vector<double> err)
{
    int index = 0;
    int max = 0;
    for(int i = 0; i <err.size(); i++)
    {
        if (fabs(err[i]) > max)
        {
            index = i;
            max = fabs(err[i]);
        }
    }
    std::stringstream ss;
    ss << _joint_names[index] << ": " << err[index];
    return ss.str().c_str();
}

double toSec(int nsec)
{
    return double(nsec)/1000000000;
}

int toNsec(double sec)
{
    return sec*1000000000;
}

void graph_print(std::vector<double> print)
{
    for(int i = 0; i < print.size(); i++)
    {
        std::cout << print[i] << ",";
    }
    std::cout << "\n";
}

void _saturate(std::vector<double> &integral, double sat_limit)
{
    for(int i = 0; i < integral.size(); i++)
    {
        integral[i] = (fabs(integral[i]) > sat_limit ? sat_limit*(integral[i]<0?-1:1) : integral[i]);
    }
}

int BaxterLimb::quickly_to_position(JointPositions desired, ros::Duration timeout)
{
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
    //v_print(integral);
    
    while( !_in_range(error,FAST) && ( ros::Time::now() - start < timeout ) && ros::ok())
    {
        dt = ros::Time::now() - last;
        dt.nsec = (toSec(dt.nsec) < 1/hz ? toNsec(1/hz) : dt.nsec);
        last = ros::Time::now();
        
        error = v_difference(position, joint_angles());
        integral = v_sum(integral, product(error, toSec(dt.nsec)));
        _saturate(integral, 0.75);
        derivative = quotient(v_difference(error, previous_error), toSec(dt.nsec));
        
        output.velocities = compute_gains(error, integral, derivative, FAST);
        _limit_acceleration(output.velocities, last_vel, toSec(dt.nsec));
        //v_print(joint_angles(), "current angles");
        
        set_joint_velocities(output);
        
        last_vel = output.velocities;
        previous_error = error;    
        
        ros::spinOnce();
        r.sleep();
    }
    std::cout << "\nOperating time: ";
    std::cout << ros::Time::now() - start << std::endl;
    if(_in_range(error, FAST))
        return 1;
    
    ROS_ERROR("Timeout: moving quickly to position. Max error at [ %s ]", _max_error(error));
    return -1;
}

int BaxterLimb::accurate_to_position(JointPositions desired, ros::Duration timeout)
{
    //std::cout<<"accurate"<<"\n";
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
    
    while( !_in_range(error,SLOW) && ( ros::Time::now() - start < timeout ) && ros::ok())
    {
        dt = ros::Time::now() - last;
        dt.nsec = (toSec(dt.nsec) < 1/hz ? toNsec(1/hz) : dt.nsec);
        last = ros::Time::now();
        error = v_difference(position, joint_angles());
        integral = v_sum(integral, product(error, toSec(dt.nsec)));
        _saturate(integral, 1);
        derivative = quotient(v_difference(error, previous_error), toSec(dt.nsec));
        output.velocities = compute_gains(error, integral, derivative, SLOW);
        _limit_acceleration(output.velocities, last_vel, toSec(dt.nsec));
        set_joint_velocities(output);
        
        last_vel = output.velocities;
        previous_error = error;        
        
        ros::spinOnce();
        r.sleep();
    }
    //std::cout << "\nOperating time: ";
    //std::cout << ros::Time::now() - start << std::endl;
    if(_in_range(error, SLOW))
        return 1;
    
    ROS_ERROR("Timeout: moving accurately to position. Max error at [ %s ]", _max_error(error));
    return -1;
}

int BaxterLimb::to_position(JointPositions desired, ros::Duration timeout)
{
    //std::cout<<"accurate"<<"\n";
    double hz = 100;
    bool speed = FAST;
    ros::Time start  = ros::Time::now();
    ros::Rate r(hz); 
    std::vector<double> position = desired.angles;
    std::vector<double> current = joint_angles();
    std::vector<double> error = v_difference(current, position);
    
    while( !_in_range(error,speed) && ( ros::Time::now() - start < timeout ))
    {
        set_joint_positions(desired);   
        error = v_difference(joint_angles(), desired.angles);
        ros::spinOnce();
        r.sleep();
    }
    //std::cout << "\nOperating time: ";
    //std::cout << ros::Time::now() - start << std::endl;
    if(_in_range(error, speed))
        return 1;
    
    ROS_ERROR("Timeout: moving to position. Max error at [ %s ]", _max_error(error));
    return -1;
}


/*----------------------------------------------------------------------
 * Everything after this line is not a part of the BaxterLimb Class
 * ---------------------------------------------------------------------*/


void v_print(std::vector<double> x)
{
    for(int i = 0; i < x.size(); i++)
    {
        std::cout<<"Vector member ";
        std::cout<<i;
        std::cout<<": ";
        std::cout<<x[i]<<std::endl;
    }
}

void v_print(std::vector<double> x, std::string name)
{
    for(int i = 0; i < x.size(); i++)
    {
        std::cout<< name << " member ";
        std::cout<<i;
        std::cout<<": ";
        std::cout<<x[i]<<std::endl;
    }
}

// Math functions named v_<name> are performed on two vector doubles
// other functions use one vector double and one double

double limit(double max, double val)
{
    //std::cout << "max: " << max << "  val: " << val << std::endl;
    //std::cout << "  " << ((val < 0) ? -1 : 1) << "  " << max << " ";
    //std::cout << " new: " << val << std::endl;
    if (fabs(val) > max)
        return max*double((val < 0) ? -1 : 1);
    return val;
}

std::vector<double> v_difference(std::vector<double> x, std::vector<double> y)
{
    std::vector<double> temp;
    for(int i = 0; i < x.size(); i++)
    {
        temp.push_back(x[i] - y[i]);
    }
    return temp;
}

std::vector<double> v_sum(std::vector<double> x, std::vector<double> y)
{
    std::vector<double> temp;
    for(int i = 0; i < x.size(); i++)
    {
        temp.push_back(x[i] + y[i]);
    }
    return temp;
}

std::vector<double> product(std::vector<double> x, double y)
{
    std::vector<double> temp;
    for(int i = 0; i < x.size(); i++)
    {
        temp.push_back(x[i]*y);
    }
    return temp;
}

std::vector<double> quotient(std::vector<double> x, double y)
{
    std::vector<double> temp;
    double q;
    for(int i = 0; i < x.size(); i++)
    {
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