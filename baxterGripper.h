#include "ros/ros.h"
#include <iostream>
#include <string>
#include <sstream>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <baxter_core_msgs/EndEffectorCommand.h>
#include <baxter_core_msgs/EndEffectorProperties.h>
#include <baxter_core_msgs/EndEffectorState.h>
#include <sensor_msgs/Range.h>

#ifndef BAXTER_GRIPPER
#define BAXTER_GRIPPER

namespace bcm = baxter_core_msgs;

class BaxterGripper
{
    //interface for gripper on Baxter Research Robot
public:
    BaxterGripper(std::string); //constructor. declare side
    BaxterGripper(std::string, int);
    
    //Expose Publishers
    void calibrate();
    void clear_calibration();
    void reset();
    void go_to(float);
    void command(std::string, bool = false, ros::Duration = 0, std::string = "");
    void open();
    void close();
    
    //Subscriber Data
    bcm::EndEffectorProperties properties;
    bcm::EndEffectorState state;
    float range;                                // ir_range value from message
    sensor_msgs::Range range_msg;               // ir_range message
    std::string type();
    uint hardware_id();
    bool calibrated();
    bool ready();
    bool error();
    
    float gripperLength;
    bool block;                                 // if true, waits until gripper is finished moving before exit
    std::string name;
    
private:
    void _init(); //initializing common to both constructors
    BaxterGripper(); //need to declare a side, so don't reveal default constructor
    ros::NodeHandle _nh;
    int _bufferSize;
    
    bcm::EndEffectorCommand _ee_cmd;
    
    ros::Publisher _pub_cmd;
    ros::Subscriber _sub_properties;
    ros::Subscriber _sub_state;
    ros::Subscriber _sub_ir_range;
    
    void _on_gripper_properties(const bcm::EndEffectorProperties::ConstPtr&);
    void _on_gripper_state(const bcm::EndEffectorState::ConstPtr&);
    void _on_ir_range(const sensor_msgs::Range::ConstPtr&);
    void _wait_for_subscriber(ros::Subscriber&);
    
    void _wait(ros::Duration = 3);                               //  waits until the gripper is no longer moving
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

BaxterGripper::BaxterGripper(std::string side, int bufferSize)
{
    _bufferSize = bufferSize;
    name = side;
    _init();
}

void BaxterGripper::_init()
{
    //Setup Subs/Pubs
    std::string topic = "/robot/end_effector/"+name+"_gripper/";
    this->_pub_cmd = _nh.advertise<bcm::EndEffectorCommand>(topic+"command",_bufferSize);
    this->_sub_properties = _nh.subscribe(topic+"properties",_bufferSize,&BaxterGripper::_on_gripper_properties, this);
    this->_sub_state = _nh.subscribe(topic+"state",_bufferSize,&BaxterGripper::_on_gripper_state, this);
    this->_sub_ir_range = _nh.subscribe("/robot/range/"+name+"_hand_range/state",_bufferSize,&BaxterGripper::_on_ir_range, this);
    this->_wait_for_subscriber(_sub_state);
    this->_wait_for_subscriber(_sub_properties);
    this->_wait_for_subscriber(_sub_ir_range);
    ros::spinOnce();
    std::cout<<"Hardware id: "<<this->hardware_id();
    this->_ee_cmd.id = this->hardware_id();
    
    // Calibrate gripper
    this->calibrate();
    while (!this->ready()) {
        ros::spinOnce();
        //ROS_ERROR(" %s_gripper waiting for ready", name.c_str());
    }
    this->block = false;
    this->gripperLength = 0;
}

void BaxterGripper::_wait_for_subscriber(ros::Subscriber& sub)
{
    ros::Duration timeout(2);
    ros::Time start = ros::Time::now();
    ros::Rate r(10);
    while (ros::Time::now() - start < timeout)
    {
        if(sub.getNumPublishers() > 0)
            return;
        r.sleep();
    }
    ROS_ERROR("Unable to subscribe to gripper topic [%s]", sub.getTopic().c_str());
}

void BaxterGripper::_on_gripper_properties(const bcm::EndEffectorProperties::ConstPtr& msg)
{
    properties = *msg;
}

void BaxterGripper::_on_gripper_state(const bcm::EndEffectorState::ConstPtr& msg)
{
    state = *msg;
}

void BaxterGripper::_on_ir_range(const sensor_msgs::Range::ConstPtr& msg)
{
    range_msg = *msg;
    range = range_msg.range;
}

void BaxterGripper::_wait(ros::Duration timeout)
{
    ros::Rate r(2);
    r.sleep();
    ros::Time start = ros::Time::now();
    while( this->state.moving && !this->ready() && (ros::Time::now() - start < timeout) )
    {
        ros::spinOnce();
    }
}

void BaxterGripper::open()
{
    this->go_to(100);
}

void BaxterGripper::close()
{
    this->go_to(0);
}

void BaxterGripper::clear_calibration()
{
    if (this->type() != "electric")
        return;
    this->block = true;
    std::string cmd = bcm::EndEffectorCommand::CMD_CLEAR_CALIBRATION;
    ros::Duration timeout(3);
    this->command(cmd, true, timeout, "");
    this->block = false;    
}

void BaxterGripper::calibrate()
{
    std::cout<<"type: "<<this->type()<<"\n";
    if (this->type() != "electric")
        return;
    //if (this->calibrated())
    //    this->clear_calibration();
    if (this->error())
        this->reset();
    ros::Duration timeout(5);
    std::string cmd = bcm::EndEffectorCommand::CMD_CALIBRATE;
    this->command(cmd, true, timeout, "");
}

void BaxterGripper::reset()
{
    if (this->type() != "electric")
        return;
    this->block = true;
    ros::Duration timeout(2);
    std::string cmd = bcm::EndEffectorCommand::CMD_RESET;
    this->command(cmd, true, timeout, "");
}

void BaxterGripper::command(std::string cmd, bool _block, ros::Duration timeout, std::string args)
{
    //std::cout<<"Sending command \'"<<cmd<<"\' with arguments \'"<<args<<"\'\n";
    this->_ee_cmd.command = cmd;
    if (args != "")
        this->_ee_cmd.args = args;
    _pub_cmd.publish(this->_ee_cmd);
    if(_block)
        _wait(timeout);
}

void BaxterGripper::go_to(float pos)
{
    std::string cmd = bcm::EndEffectorCommand::CMD_GO;
    std::stringstream args;
    args << "{\"position\": " << pos << "}";
    ros::Duration timeout(5);
    this->command(cmd, true, timeout,  args.str());
}

bool BaxterGripper::calibrated()
{
    return (this->state.calibrated == true);
}

bool BaxterGripper::ready()
{
    return (this->state.ready == true);
}

bool BaxterGripper::error()
{
    return (this->state.error == true);
}

std::string BaxterGripper::type()
{
    char type = this->properties.ui_type;
    std::string val;
    switch (type)
    {
        case (bcm::EndEffectorProperties::SUCTION_CUP_GRIPPER):
            val = "suction";
            break;
        case (bcm::EndEffectorProperties::ELECTRIC_GRIPPER):
            val = "electric";
            break;
        case (bcm::EndEffectorProperties::CUSTOM_GRIPPER):
            val = "custom";
            break;
        default:
            val = "electric"; // should be "none"
            break;
    }
    return val;
}

uint BaxterGripper::hardware_id() {
    // Return what seems like the default if none provided
    if (this->state.id == 0)
        return 65664;
    return this->state.id;
}

#endif