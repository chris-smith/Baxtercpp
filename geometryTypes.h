#ifndef GEOMETRY_TYPES
#define GEOMETRY_TYPES

// SHOULD PUT ALL GEOMETRY TYPES AND OPERATORS INTO
// THEIR OWN HEADER FILE

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
Point operator+(const Point &a, const Point &b)
{
    Point ret;
    ret.x = a.x + b.x;
    ret.y = a.y + b.y;
    ret.z = a.z + b.z;
    return ret;
}
Point operator-(const Point &a, const Point &b)
{
    Point ret;
    ret.x = a.x - b.x;
    ret.y = a.y - b.y;
    ret.z = a.z - b.z;
    return ret;
}
Point operator*(const Point &a, const Point &b)
{
    Point ret;
    ret.x = a.x * b.x;
    ret.y = a.y * b.y;
    ret.z = a.z * b.z;
    return ret;
}
Point operator/(const Point &a, const Point &b)
{
    Point ret;
    ret.x = a.x / b.x;
    ret.y = a.y / b.y;
    ret.z = a.z / b.z;
    return ret;
}
Point operator+(const Point &a, double &b)
{
    Point ret;
    ret.x = a.x + b;
    ret.y = a.y + b;
    ret.z = a.z + b;
    return ret;
}
Point operator-(const Point &a, double &b)
{
    Point ret;
    ret.x = a.x - b;
    ret.y = a.y - b;
    ret.z = a.z - b;
    return ret;
}
Point operator*(const Point &a, double &b)
{
    Point ret;
    ret.x = a.x * b;
    ret.y = a.y * b;
    ret.z = a.z * b;
    return ret;
}
Point operator/(const Point &a, double &b)
{
    Point ret;
    ret.x = a.x / b;
    ret.y = a.y / b;
    ret.z = a.z / b;
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
bool operator=(const Point &a, const Point &b)
{
    bool ret = true;
    ret = ret && (a.x == b.x);
    ret = ret && (a.y == b.y);
    ret = ret && (a.z == b.z);
    return ret;
}

//  Quaternion Operators  ------------------------------
Quaternion operator+(const Quaternion &a, const Quaternion &b)
{
    Quaternion ret;
    ret.x = a.x + b.x;
    ret.y = a.y + b.y;
    ret.z = a.z + b.z;
    ret.w = a.w + b.w;
    return ret
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
bool operator=(const Quaternion &a, const Quaternion &b)
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
Bool operator=(const Pose &a, const Pose &b)
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
bool operator=(const Twist &a, const Twist &b)
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
bool operator=(const Wrench &a, const Wrench &b)
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
    Wrench ret;
    ret.pitch = a.pitch - b.pitch;
    ret.roll = a.roll - b.roll;
    ret.yaw = a.yaw - b.yaw;
    return ret;
}
bool operator=(const PRY &a, const PRY &b)
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
    ret.force = a.point + b.point;
    ret.torque = a.pry + b.pry;
    return ret;
}
PRYPose operator-(const PRYPose &a, const PRYPose &b)
{
    PRYPose ret;
    ret.force = a.point - b.point;
    ret.torque = a.pry - b.pry;
    return ret;
}
bool operator=(const PRYPose &a, const PRYPose &b)
{
    bool ret = true;
    ret = ret && (a.point == b.point);
    ret = ret && (a.pry == b.pry);
}



#endif