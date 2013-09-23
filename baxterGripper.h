#include "ros/ros.h"
#include <iostream>
#include <string>
#include <sstream>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <baxter_msgs/GripperCommand.h>
#include <baxter_msgs/GripperIdentity.h>
#include <baxter_msgs/GripperProperties.h>
#include <baxter_msgs/GripperState.h>
#include <sensor_msgs/Range.h>

#ifndef BAXTER_GRIPPER
#define BAXTER_GRIPPER

enum gripperType {  NORMAL, WIDE, LONG, CUSTOM  };

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
    
    //Subscriber Data
    baxter_msgs::GripperIdentity identity;
    baxter_msgs::GripperProperties properties;
    baxter_msgs::GripperState state;
    float range;                                // ir_range value from message
    sensor_msgs::Range range_msg;               // ir_range message
    
    float length();
    
    gripperType type;
    float gripperLength;
    bool block;                                 // if true, waits until gripper is finished moving before exit
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
    ros::Subscriber _sub_ir_range;
    
    void _on_gripper_identity(const baxter_msgs::GripperIdentity::ConstPtr&);
    void _on_gripper_properties(const baxter_msgs::GripperProperties::ConstPtr&);
    void _on_gripper_state(const baxter_msgs::GripperState::ConstPtr&);
    void _on_ir_range(const sensor_msgs::Range::ConstPtr&);
    
    void _wait();                               //  waits until the gripper is no longer moving
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
    _sub_identity = _nh.subscribe(topic+"identity",_bufferSize,&BaxterGripper::_on_gripper_identity, this); 
    _sub_properties = _nh.subscribe(topic+"properties",_bufferSize,&BaxterGripper::_on_gripper_properties, this);
    _sub_state = _nh.subscribe(topic+"state",_bufferSize,&BaxterGripper::_on_gripper_state, this);
    _sub_ir_range = _nh.subscribe("/robot/range/"+name+"_hand_range",_bufferSize,&BaxterGripper::_on_ir_range, this);
    
    
    calibrate();
    block = false;
    gripperLength = 0;
    type = CUSTOM;
}

void BaxterGripper::_on_gripper_identity(const baxter_msgs::GripperIdentity::ConstPtr& msg)
{
    identity = *msg;
}

void BaxterGripper::_on_gripper_properties(const baxter_msgs::GripperProperties::ConstPtr& msg)
{
    properties = *msg;
}

void BaxterGripper::_on_gripper_state(const baxter_msgs::GripperState::ConstPtr& msg)
{
    state = *msg;
}

void BaxterGripper::_on_ir_range(const sensor_msgs::Range::ConstPtr& msg)
{
    range_msg = *msg;
    range = range_msg.range;
}

void BaxterGripper::_wait()
{
    ros::Time start = ros::Time::now();
    ros::Duration timeout(2.5);
    while( state.moving && (ros::Time::now() - start < timeout) )
    {
        ros::spinOnce();
    }
}

float BaxterGripper::length()
{
    switch(type)
    {
        case NORMAL:
            gripperLength = 0.0508;
            break;
        case WIDE:
            gripperLength = 0.0508;
            break;
        case LONG:
            gripperLength = 0.1016;
            break;
        case CUSTOM:
            break;
        default:
            gripperLength = 0;
            break;
    }
    return gripperLength;
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
    if(block)
        _wait();
}

void BaxterGripper::go_to(float pos)
{
    std_msgs::Float32 msg;
    msg.data = pos;
    _pub_goto.publish(msg);
    if(block)
        _wait();
}

#endif