#include "opencv2/features2d/features2d.hpp"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#ifndef GEOMETRY_TYPES
#define GEOMETRY_TYPES

/*    Here are the types defined here

struct Point{ double x; double y; double z; };
struct Quaternion{ double x; double y; double z; double w; };
struct Pose{ Point point; Quaternion quaternion; };
struct Twist{ Point linear; Point angular; };
struct Wrench{ Point force; Point torque; };
struct PRY{ double pitch; double roll; double yaw; };
struct PRYPose{ Point point; PRY pry; };

/* Define operators for structs *
*********************************/
namespace gm = geometry_msgs;


//  Point Operators  ------------------------------
namespace geometry_variables{
    //  Geometry Variables
    //  Can also be referred to as gv
    class Point
    {
    public:
        double x, y, z;
        Point() : x(0), y(0), z(0) {}
        Point(double a, double b, double c) : x(a), y(b), z(c) {}
        Point(double a) : x(a),y(a),z(a) {}
        // Casting and Assignment
        operator geometry_msgs::Point()
        {
            geometry_msgs::Point ret;
            ret.x = x;
            ret.y = y;
            ret.z = z;
            return ret;
        }
        Point(geometry_msgs::Point point)
        {
            *this = point;
        }
        Point& operator=(const geometry_msgs::Point & rhs)
        {
            this->x = rhs.x;
            this->y = rhs.y;
            this->z = rhs.z;
            return *this;
        }
        // Printing
        void print()
        {
            std::cout<< "x: " << x;
            std::cout<< "  y: " << y;
            std::cout<< "  z: " << z << std::endl;
        }
        // Arithmetic
        Point& operator+=(const Point &a)
        {
            this->x = this->x + a.x;
            this->y = this->y + a.y;
            this->z = this->z + a.z;
            return *this;
        }
        Point & operator+=(const double &a)
        {
            this->x = this->x + a;
            this->y = this->y + a;
            this->z = this->z + a;
            return *this;
        }
        Point& operator-=(const Point &a)
        {
            this->x = this->x - a.x;
            this->y = this->y - a.y;
            this->z = this->z - a.z;
            return *this;
        }
        Point & operator-=(const double &a)
        {
            this->x = this->x - a;
            this->y = this->y - a;
            this->z = this->z - a;
            return *this;
        }
        Point& operator*=(const Point &a)
        {
            this->x = this->x * a.x;
            this->y = this->y * a.y;
            this->z = this->z * a.z;
            return *this;
        }
        Point & operator*=(const double &a)
        {
            this->x = this->x * a;
            this->y = this->y * a;
            this->z = this->z * a;
            return *this;
        }
        Point& operator/=(const Point &a)
        {
            this->x = this->x / a.x;
            this->y = this->y / a.y;
            this->z = this->z / a.z;
            return *this;
        }
        Point & operator/=(const double &a)
        {
            this->x = this->x / a;
            this->y = this->y / a;
            this->z = this->z / a;
            return *this;
        }
        const Point operator+(const Point& a) const
        {
            return Point(*this) += a;
        }
        const Point operator+(const double& a) const
        {
            return Point(*this) += a;
        }
        const Point operator-(const Point& a) const
        {
            return Point(*this) -= a;
        }
        const Point operator-(const double& a) const
        {
            return Point(*this) -= a;
        }
        const Point operator*(const Point& a) const
        {
            return Point(*this) *= a;
        }
        const Point operator*(const double& a) const
        {
            return Point(*this) *= a;
        }
        const Point operator/(const Point& a) const
        {
            return Point(*this) /= a;
        }
        const Point operator/(const double& a) const
        {
            return Point(*this) /= a;
        }
    };
    Point abs(const Point &a)
    {
        Point ret;
        ret.x = fabs(a.x);
        ret.y = fabs(a.y);
        ret.z = fabs(a.z);
        return ret;
    }
    bool operator<(const Point &a, const Point &b)
    {
        bool ret = true;
        ret = ret && (a.x < b.x);
        ret = ret && (a.y < b.y);
        ret = ret && (a.z < b.z);
        return ret;
    }
    bool operator>(const Point &a, const Point &b)
    {
        bool ret = true;
        ret = ret && (a.x > b.x);
        ret = ret && (a.y > b.y);
        ret = ret && (a.z > b.z);
        return ret;
    }
    bool operator==(const Point &a, const Point &b)
    {
        bool ret = true;
        ret = ret && (a.x == b.x);
        ret = ret && (a.y == b.y);
        ret = ret && (a.z == b.z);
        return ret;
    }
    bool operator !=(const Point &a, const Point &b)
    {
        return !(a == b);   
    }

    struct Quaternion{ 
        double x, y, z, w; 
        operator geometry_msgs::Quaternion()
        {
            geometry_msgs::Quaternion ret;
            ret.x = x;
            ret.y = y;
            ret.z = z;
            ret.w = w;
            return ret;
        }
        Quaternion& operator=(const geometry_msgs::Quaternion & rhs)
        {
            this->x = rhs.x;
            this->y = rhs.y;
            this->z = rhs.z;
            this->w = rhs.w;
            return *this;
        }
        Quaternion(){}
        Quaternion(geometry_msgs::Quaternion quat)
        {
            *this = quat;
        }
    };
    //  Pose Operators  ------------------------------
    struct Pose{ 
        // Member Variables
        Point position; 
        Quaternion orientation; 
        // Operators
        operator geometry_msgs::Pose()
        {
            geometry_msgs::Pose ret;
            ret.position = position;
            ret.orientation = orientation;
            return ret;
        }
        Pose& operator=(const geometry_msgs::Pose &rhs)
        {
            this->position = rhs.position;
            this->orientation = rhs.orientation;
            return *this;
        }
        Pose(geometry_msgs::Pose pose)
        {
            *this = pose;
        }
        Pose(){}
    };
    struct Twist{ Point linear; Point angular; };
    struct Wrench{ Point force; Point torque; };
    class PRY
    { 
    public:
        double pitch; double roll; double yaw; 
        PRY(double a, double b, double c) : pitch(a), roll(b), yaw(c) {}
        PRY(double a) : pitch(a), roll(a), yaw(a) {}
        PRY(Quaternion quat){
            double q0 = quat.w;
            double q1 = quat.x;
            double q2 = quat.y;
            double q3 = quat.z;
            pitch = asin(2*(q0*q2 - q3*q1));
            roll = atan2(2*(q0*q1 + q2*q3),1-2*(q1*q1 + q2*q2));
            yaw = atan2(2*(q0*q3 + q1*q2),1-2*(q2*q2 + q3*q3));
        }
        PRY() {}
        void print()
        {
            std::cout << "pitch: " << pitch;
            std::cout << "  roll: " << roll;
            std::cout << "  yaw: " << yaw << std::endl;
        }
    };
    class PRYPose
    { 
    public:
        Point position; PRY pry; 
        PRYPose(Point a, PRY b) : position(a), pry(b) {}
        PRYPose(double a, double b, double c, double d, double e, double f) :
            position(Point(a,b,c)), pry(PRY(d,e,f)) {}
        PRYPose(double a) : position(a), pry(a) {}
        PRYPose(Pose a) : position(a.position), pry(a.orientation) {}
        PRYPose() : position(0), pry(0) {}
        PRYPose& operator=(const geometry_msgs::Pose &rhs)
        {
            this->position = rhs.position;
            this->pry = PRY(rhs.orientation);
            return *this;
        }
        void print()
        {
            position.print();
            pry.print();
        }
    };

    //  Quaternion Operators  ------------------------------
    Quaternion operator+(const Quaternion &a, const Quaternion &b)
    {
        Quaternion ret;
        ret.x = a.x + b.x;
        ret.y = a.y + b.y;
        ret.z = a.z + b.z;
        ret.w = a.w + b.w;
        return ret;
    }
    Quaternion operator-(const Quaternion &a, const Quaternion &b)
    {
        Quaternion ret;
        ret.x = a.x - b.x;
        ret.y = a.y - b.y;
        ret.z = a.z - b.z;
        ret.w = a.w - b.w;
        return ret;
    }
    bool operator==(const Quaternion &a, const Quaternion &b)
    {
        bool ret = true;
        ret = ret && (a.x == b.x);
        ret = ret && (a.y == b.y);
        ret = ret && (a.z == b.z);
        ret = ret && (a.w == b.w);
        return ret;
    }
        
    Pose operator+(const Pose &a, const Pose &b)
    {
        Pose ret;
        ret.position = a.position + b.position;
        ret.orientation = a.orientation + b.orientation;
        return ret;    
    }
    Pose operator-(const Pose &a, const Pose &b)
    {
        Pose ret;
        ret.position = a.position - b.position;
        ret.orientation = a.orientation - b.orientation;
        return ret;    
    }
    bool operator==(const Pose &a, const Pose &b)
    {
        bool ret = true;
        ret = ret && (a.position == b.position);
        ret = ret && (a.orientation == b.orientation);
        return ret;
    }

    //  Twist Operators  ------------------------------
    Twist operator+(const Twist &a, const Twist &b)
    {
        Twist ret;
        ret.linear = a.linear + b.linear;
        ret.angular = a.angular + b.angular;
        return ret;
    }
    Twist operator-(const Twist &a, const Twist &b)
    {
        Twist ret;
        ret.linear = a.linear - b.linear;
        ret.angular = a.angular - b.angular;
        return ret;
    }
    bool operator==(const Twist &a, const Twist &b)
    {
        bool ret = true;
        ret = ret && (a.linear == b.linear);
        ret = ret && (a.angular == b.angular);
    }

    //  Wrench Operators  ------------------------------
    Wrench operator+(const Wrench &a, const Wrench &b)
    {
        Wrench ret;
        ret.force = a.force + b.force;
        ret.torque = a.torque + b.torque;
        return ret;
    }
    Wrench operator-(const Wrench &a, const Wrench &b)
    {
        Wrench ret;
        ret.force = a.force - b.force;
        ret.torque = a.torque - b.torque;
        return ret;
    }
    bool operator==(const Wrench &a, const Wrench &b)
    {
        bool ret = true;
        ret = ret && (a.force == b.force);
        ret = ret && (a.torque == b.torque);
    }

    //  PRY Operators  ------------------------------
    PRY operator+(const PRY &a, const PRY &b)
    {
        PRY ret;
        ret.pitch = a.pitch + b.pitch;
        ret.roll = a.roll + b.roll;
        ret.yaw = a.yaw + b.yaw;
        return ret;
    }
    PRY operator-(const PRY &a, const PRY &b)
    {
        PRY ret;
        ret.pitch = a.pitch - b.pitch;
        ret.roll = a.roll - b.roll;
        ret.yaw = a.yaw - b.yaw;
        return ret;
    }
    bool operator==(const PRY &a, const PRY &b)
    {
        bool ret = true;
        ret = ret && (a.pitch == b.pitch);
        ret = ret && (a.roll == b.roll);
        ret = ret && (a.yaw == b.yaw);
    }

    //  PRYPose Operators  ------------------------------
    PRYPose operator+(const PRYPose &a, const PRYPose &b)
    {
        PRYPose ret;
        ret.position = a.position + b.position;
        ret.pry = a.pry + b.pry;
        return ret;
    }
    PRYPose operator-(const PRYPose &a, const PRYPose &b)
    {
        PRYPose ret;
        ret.position = a.position - b.position;
        ret.pry = a.pry - b.pry;
        return ret;
    }
    bool operator==(const PRYPose &a, const PRYPose &b)
    {
        bool ret = true;
        ret = ret && (a.position == b.position);
        ret = ret && (a.pry == b.pry);
    }
}


/*       Namespace Shorthand
 * *************************/
namespace gv = geometry_variables;

gv::Quaternion toQuat(gv::PRY pry)
{
    gv::Quaternion quat;
    double p = pry.pitch/2;
    double r = pry.roll/2;
    double y = pry.yaw/2;
    quat.w = cos(r)*cos(p)*cos(y) + sin(r)*sin(p)*sin(y); // q0
    quat.x = sin(r)*cos(p)*cos(y) - cos(r)*sin(p)*sin(y); // q1
    quat.y = cos(r)*sin(p)*cos(y) + sin(r)*cos(p)*sin(y); // q2
    quat.z = cos(r)*cos(p)*sin(y) - sin(r)*sin(p)*cos(y); // q3
    return quat;
}
    
gv::PRY toPRY(gv::Quaternion quat)
{
    gv::PRY pry;
    double q0 = quat.w;
    double q1 = quat.x;
    double q2 = quat.y;
    double q3 = quat.z;
    pry.pitch = asin(2*(q0*q2 - q3*q1));
    pry.roll = atan2(2*(q0*q1 + q2*q3),1-2*(q1*q1 + q2*q2));
    pry.yaw = atan2(2*(q0*q3 + q1*q2),1-2*(q2*q2 + q3*q3));
    return pry;
}

/* *  Custom Overloads of OpenCV Operators  *
* ******************************************/

cv::Rect operator*(const cv::Rect &a, double b)
{
    double height = a.height;
    double width = a.width;
    cv::Point center(a.x+width/2, a.y+height/2);
    height *= b;
    width *= b;
    cv::Point origin(center.x - width/2, center.y - height/2);
    cv::Rect ret(origin, cv::Size(width,height));
    return ret;
}

cv::Point2d operator+(const cv::Point2d &a, double b)
{
    cv::Point ret;
    ret.x = a.x + b;
    ret.y = a.y + b;
    return ret;
}

cv::Point2d operator-(const cv::Point2d &a, double b)
{
    cv::Point ret;
    ret.x = a.x - b;
    ret.y = a.y - b;
    return ret;
}

cv::Point2d constrain(const cv::Point2d &a, const cv::Point2d &min, const cv::Point2d &max)
{
    cv::Point2d ret = a;
    if (a.x > max.x)
        ret.x = max.x;
    else if (a.x < min.x)
        ret.x = min.x;
    
    if (a.y > max.y)
        ret.y = max.y;
    else if (a.y < min.y)
        ret.y = min.y;
    
    return ret;
}

#endif