#include "ros/ros.h"
#include <sensor_msgs/PointCloud.h>
#include "baxterLimb.h"


#ifndef BAXTERSONAR_H
#define BAXTERSONAR_H

enum speed { _SLOW, _NORMAL };

// this class exists to slow Baxter down when he detects people
class BaxterSonar {
public:
    BaxterSonar(BaxterLimb*, BaxterLimb*);
    
    // these variables set max velocity for Baxter's limbs
    double slow_speed;          //  when person is near
    double normal_speed;        //  when nobody is near
    
    speed current_speed;
    
    sensor_msgs::PointCloud pointCloud;
    
private:
    BaxterLimb* _arm_a;
    BaxterLimb* _arm_b;
    ros::NodeHandle _nh;
    
    double _slow_distance;
    
    double _get_distance(geometry_msgs::Point32);
    
    BaxterSonar();
    ros::Subscriber _sub_sonar_state;
    
    void _on_sonar_state(const sensor_msgs::PointCloud::ConstPtr&);
};


BaxterSonar::BaxterSonar(BaxterLimb* a, BaxterLimb* b) {
    _arm_a = a;
    _arm_b = b;
    
    // Defaults are very slow
    slow_speed = 0;
    normal_speed = .5;
    current_speed = _NORMAL;
    _slow_distance = 2;
    
    _sub_sonar_state = _nh.subscribe("/robot/sonar/head_sonar/state",1,&BaxterSonar::_on_sonar_state, this);
}

void BaxterSonar::_on_sonar_state(const sensor_msgs::PointCloud::ConstPtr& msg) {
    pointCloud = *msg;
    std::vector< geometry_msgs::Point32 > points = msg->points;
    float x;
    float min = 10;
    for (int i = 0; i < points.size(); i++) {
        x = _get_distance(points[i]);
        if (x < min)
            min = x;
    }
    std::cout<<"\nclosest object: "<<min;
    if (min <= _slow_distance) {
        std::cout<<"\nSomething is too close to me\n";
        if (current_speed) {
            _arm_a->set_max_velocity(slow_speed);
            _arm_b->set_max_velocity(slow_speed);
        }
    }
    else {
        if (!current_speed) {
            _arm_a->set_max_velocity(normal_speed);
            _arm_b->set_max_velocity(normal_speed);
        }
    }
}

double BaxterSonar::_get_distance(geometry_msgs::Point32 point) {
    return sqrt(point.x*point.x + point.y*point.y + point.z*point.z);
}

#endif