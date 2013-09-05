#include "ros/ros.h"
#include "iostream"
#include "image_converter.h"
#include "ros_cpp_server/ros_cpp_node.h"
#include <string>

class ExecuteRos
{
public:
    ExecuteRos();
    void test();
    void setFunction(std::string);
    void setRate(uint);
    void setResponse(std::string);
    void setRun(bool);
    
private:
    bool run;
    std::string function;
    uint pubRate;
    std::string response;
    ros::Publisher pub;
  
};

void ExecuteRos::setRate(uint rate )
{
    pubRate = rate;
}

void ExecuteRos::setFunction(std::string msg)
{
    function = msg;
}

void ExecuteRos::setResponse(std::string msg)
{
    response = msg;
}

void ExecuteRos::setRun(bool state)
{
    run = state;
}

ExecuteRos::ExecuteRos()
    {
      run = false;
      function = "";
      pubRate = 10;
      response = "";
    }
    
void ExecuteRos::test()
{
    std::cout << "This test thing worked" << std::endl;
}
