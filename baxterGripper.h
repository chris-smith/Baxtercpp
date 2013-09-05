#include "ros/ros.h"
#include <iostream>
#include <string>
#include <sstream>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <baxter_msgs/GripperCommand.h>

class BaxterGripper
{
    //interface for gripper on Baxter Research Robot
public:
    BaxterGripper(std::string); //constructor. declare side
    BaxterGripper(ros::NodeHandle, std::string, int);
    
    //Expose Publishers
    void calibrate();
    void reset();
    void command(baxter_msgs::GripperCommand);
    void go_to(float);
    
    std::string name;
    
private:
    void _init(); //initializing common to both constructors
    BaxterGripper(); //need to declare a side, so don't reveal default constructor
    ros::NodeHandle _nh;
    int _bufferSize;
    
    ros::Publisher _pub_reset;
    ros::Publisher _pub_calibrate;
    ros::Publisher _pub_command;
    ros::Publisher _pub_goto;
    ros::Subscriber _sub_identity;
    ros::Subscriber _sub_properties;
    ros::Subscriber _sub_state;
};

BaxterGripper::BaxterGripper()
{
    ROS_ERROR("Private Default Constructor -- Should never be called");
}

BaxterGripper::BaxterGripper(std::string side)
{
    _bufferSize = 1;
    name = side;
    _init();
}

BaxterGripper::BaxterGripper(ros::NodeHandle nh, std::string side, int bufferSize)
{
    _nh = nh;
    _bufferSize = bufferSize;
    name = side;
    _init();
}

void BaxterGripper::_init()
{
    //Calibrate the gripper
    std::string topic = "/robot/limb/"+name+"/accessory/gripper/";
    _pub_reset = _nh.advertise<std_msgs::Bool>(topic+"command_reset",_bufferSize);
    _pub_calibrate = _nh.advertise<std_msgs::Empty>(topic+"command_calibrate", _bufferSize);
    _pub_command = _nh.advertise<baxter_msgs::GripperCommand>(topic+"command_set",_bufferSize);
    _pub_goto = _nh.advertise<std_msgs::Float32>(topic+"command_goto",_bufferSize);
    calibrate();
}

void BaxterGripper::calibrate()
{
    std_msgs::Empty msg;
    _pub_calibrate.publish(msg);
}

void BaxterGripper::reset()
{
    std_msgs::Bool msg;
    msg.data = true;
    _pub_reset.publish(msg);
}

void BaxterGripper::command(baxter_msgs::GripperCommand msg)
{
    _pub_command.publish(msg);
}

void BaxterGripper::go_to(float pos)
{
    std_msgs::Float32 msg;
    msg.data = pos;
    _pub_goto.publish(msg);
}