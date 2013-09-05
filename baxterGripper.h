#include "ros/ros.h"
#include <iostream>
#include <string>
#include <sstream>


class BaxterGripper
{
    //interface for gripper on Baxter Research Robot
public:
    BaxterGripper(std::string); //constructor. declare side
    BaxterGripper(ros::NodeHandle, std::string, int);
    
    std::string name;
    
private:
    BaxterGripper(); //need to declare a side, so don't reveal default constructor

};