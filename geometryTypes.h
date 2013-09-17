#include "opencv2/features2d/features2d.hpp"
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

//  Point Operators  ------------------------------

struct Point
{
    double x, y, z;
    Point(double a, double b, double c) : x(a), y(b), z(c) {}
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

struct Quaternion{ double x; double y; double z; double w; };
struct Pose{ Point point; Quaternion quaternion; };
struct Twist{ Point linear; Point angular; };
struct Wrench{ Point force; Point torque; };
struct PRY{ double pitch; double roll; double yaw; };
struct PRYPose{ Point point; PRY pry; };

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
    
    
//  Pose Operators  ------------------------------
Pose operator+(const Pose &a, const Pose &b)
{
    Pose ret;
    ret.point = a.point + b.point;
    ret.quaternion = a.quaternion + b.quaternion;
    return ret;    
}
Pose operator-(const Pose &a, const Pose &b)
{
    Pose ret;
    ret.point = a.point - b.point;
    ret.quaternion = a.quaternion - b.quaternion;
    return ret;    
}
bool operator==(const Pose &a, const Pose &b)
{
    bool ret = true;
    ret = ret && (a.point == b.point);
    ret = ret && (a.quaternion == b.quaternion);
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
    ret.point = a.point + b.point;
    ret.pry = a.pry + b.pry;
    return ret;
}
PRYPose operator-(const PRYPose &a, const PRYPose &b)
{
    PRYPose ret;
    ret.point = a.point - b.point;
    ret.pry = a.pry - b.pry;
    return ret;
}
bool operator==(const PRYPose &a, const PRYPose &b)
{
    bool ret = true;
    ret = ret && (a.point == b.point);
    ret = ret && (a.pry == b.pry);
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