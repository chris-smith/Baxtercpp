#include "opencv2/features2d/features2d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include <boost/algorithm/string.hpp>
#include <typeinfo>
#include "geometryTypes.h"

struct BoundingBox {cv::Point2d corners[4];};
struct BoundingBox_Size {cv::Point2d origin; double height; double width;};
struct pt_line {cv::Point2d pt1; cv::Point2d pt2;};

#ifndef SEARCHGEOMETRY_H
#define SEARCHGEOMETRY_H

class SearchGeometry{
    // The purpose of this class is to help define locations where objects
    // may be found, where they may be placed, and to visualize them
public:
    SearchGeometry();    
    ~SearchGeometry();
    
    //Physical locations
    BoundingBox search_border;    //  define space where legos might be from Baxter's perspective
    BoundingBox search_path;//  box to move gripper around while searching -- inner offset of _border
    double table_height;   //  height of table from Baxter's perspective
    double height_offset;  //  height off table for Baxter's gripper when searching for pieces
    gv::PRYPose reset_button;  //  location of button to remove any pieces still in view and place new pieces 
    BoundingBox deposit_border; //  location to place found objects
    std::vector<BoundingBox> deposit_containers;  //  containers in deposit_border
    gv::PRYPose home;               // home locationd
    
    //Search area parameters
    double borderMin(char); //get minimum dimension of _border in 'x' or 'y'
    double borderMax(char); //get maximum dimension of _border in 'x' or 'y'
    gv::PRYPose getNextPosition(gv::PRYPose); //return a new position to move to for searching
    
    //deposit box drawing parameters
    void setPixelScale(int, int);
    void setImageBuffer(int, int);
    
    void BuildDepositBox(); //builds and displays image of deposit_border, deposit_containers in _depositBox
    void ResetDepositBox(); //sets _depositBox to blank image      
    
private:
    void Init();            //  Initializing common to both constructors    
    
    //deposit box image params
     cv::Mat _depositBox;    //  picture representing tray to place legos on
    int _xPixelScale;             //  x scale between image representation of box and dimension in m
    int _yPixelScale;             //  y scale between image representation of box and dimension in m
    int _xBuffer;                 //  x offset on images
    int _yBuffer;                 //  y offset on images
    cv::RNG* _rng; //random number generator for colors
    
    //drawing/geometric functions
    bool _on_border(cv::Point2d, pt_line);         //true if point is on the line
    bool _intersect(cv::Point2d pt, pt_line line); //true if a ray along unit vector {1,0} from pt intersects line
    bool _inside(BoundingBox, BoundingBox);        //checks if second bounding box is inside the first
    bool _inside(BoundingBox, cv::Point2d);        //checks if point is inside the boundingbox
    int _width(BoundingBox);                       //minimum width to contain boundingbox
    int _height(BoundingBox);                      //minimum height to contain boundingbox
    void _drawBox(cv::Mat&, BoundingBox,bool);     //draw box on img
    cv::Scalar _randomColor();                     //return random color
    BoundingBox _pixelAdjust(BoundingBox);         //multiplies boundbox members by pixel scales
    gv::PRYPose _get_next_search_position(gv::PRYPose, int);
    
};

SearchGeometry::SearchGeometry()
{
    Init();
}

SearchGeometry::~SearchGeometry()
{
    delete _rng;
}

void SearchGeometry::Init(){
    //geometry/drawing defaults
    _xPixelScale = 100;
    _yPixelScale = 100;
    _xBuffer = 10;
    _yBuffer = 10;
    _rng = new cv::RNG(0xFFFFFFFF);   
    table_height = 0;
    height_offset = 0;
}


double SearchGeometry::borderMax(char coor)
{
    double max = -2; //beyond physical reach of baxter
    double temp;
    for(int i = 0; i < 4; i++)
    {
        switch(coor)
        {
            case('x'):
                temp = search_border.corners[i].x;
                break;
            case('y'):
                temp = search_border.corners[i].y;
                break;
            default:
                temp = max;
                break;
        }
        if(temp > max)
            max = temp;
    }
    return max;
}

double SearchGeometry::borderMin(char coor)
{
    double min = 2; //beyond physical reach of baxter
    double temp;
    for(int i = 0; i < 4; i++)
    {
        switch(coor)
        {
            case('x'):
                temp = search_border.corners[i].x;
                break;
            case('y'):
                temp = search_border.corners[i].y;
                break;
            default:
                temp = min;
                break;
        }
        if(temp < min)
            min = temp;
    }
    return min;
}

gv::PRYPose SearchGeometry::_get_next_search_position(gv::PRYPose current, int iter)
{
    //iter helps keeps a 'timeout' on this function during recursion
    if(iter > 4)
        return current;
    double xMin = borderMin('x');
    double xMax = borderMax('x');
    double yMin = borderMin('y');
    double yMax = borderMax('y');
    double xPos = _rng->uniform(xMin,xMax);
    double yPos = _rng->uniform(yMin,yMax);
    cv::Point2d pos(xPos,yPos);
    if(_inside(search_border,pos))
    {   
        current.position.x = xPos;
        current.position.y = yPos;
        return current;
    }
    else
        return _get_next_search_position(current, iter+1);
    
}

gv::PRYPose SearchGeometry::getNextPosition(gv::PRYPose current)
{
    return _get_next_search_position(current,0);
}

void SearchGeometry::setPixelScale(int x, int y)
{
    _xPixelScale = x;
    _yPixelScale = y;
}

void SearchGeometry::setImageBuffer(int x, int y)
{
    _xBuffer = x;
    _yBuffer = y;
}

double min(double a, double b)
{
    if (a < b)
        return a;
    return b;
}

double max(double a, double b)
{
    if (a > b)
        return a;
    return b;
}

//checks if pt is on the line connecting c and d
bool SearchGeometry::_on_border(cv::Point2d pt, pt_line a)
{
    double eps = .0001; // point considered on border if distance from point to line < eps
    double m,b,dist;
    if(a.pt1.x != a.pt2.x) //maybe change this to (pt1.x - pt2.x < eps)
    {
        m = (a.pt1.y-a.pt2.y)/(a.pt1.x-a.pt2.x);  // slope
        b = a.pt1.y - m*a.pt1.x;          // y intercept
        dist = fabs(pt.y - m*pt.x - b)/sqrt(m*m + 1);
    }
    else // vertical line
        dist = fabs(pt.x - a.pt1.x);
    
    if (fabs(dist) <= eps)
        return true;
    return false;
}

bool SearchGeometry::_intersect(cv::Point2d pt, pt_line line)
{
    cv::Point2d a = line.pt1;
    cv::Point2d b = line.pt2;
    
    double x = max(a.x-pt.x, b.x-pt.x); //furthest x distance from pt to line
    if (x < 0) // pt is to the right of the line
        return false;
    double yMax, yMin;
    yMax = max(a.y, b.y);
    yMin = min(a.y, b.y);
    if (( pt.y > yMax ) || (pt.y < yMin)) //pt.y must be between y coordinates of line to intersect
        return false;
    return true;
}

bool SearchGeometry::_inside(BoundingBox outer, BoundingBox inner)
{
    //odd num intersections == inside, even == outside
    int intersections = 0; //odd == false
    cv::Point2d pt;
    pt_line line;
    for(int i = 0; i < 4; i++)
    {
        //iterate through every point in inner, counting intersections
        pt = inner.corners[i];
        for(int j = 0; j < 4; j++)
        {
            line.pt1 = outer.corners[j];
            line.pt2 = outer.corners[0];
            if(j < 3)
                line.pt2 = outer.corners[j+1];
            
            if(_on_border(pt, line))
                return false;
            if(_intersect(pt, line))
                intersections = !intersections;
        }
        if (intersections % 2 == 0)
            return false;
        intersections = 0; //reset for next point
    }
    return true;
}

bool SearchGeometry::_inside(BoundingBox outer, cv::Point2d pt)
{
    //odd num intersections == inside, even == outside
    int intersections = 0; //odd == false
    pt_line line;
    
    for(int j = 0; j < 4; j++)
    {
        line.pt1 = outer.corners[j];
        line.pt2 = outer.corners[0];
        if(j < 3)
            line.pt2 = outer.corners[j+1];
        
        if(_on_border(pt, line))
            return false;
        if(_intersect(pt, line))
            intersections = !intersections;
    }
    if (intersections % 2 == 0)
        return false;
    return true;
}

int SearchGeometry::_height(BoundingBox box)
{
    int height = 0;
    cv::Point2d a,b;
    for(int i = 0; i < 4; i++)
    {
        a = box.corners[i];
        b = box.corners[0];
        if (i < 3)
            b = box.corners[i+1];
        height = max(height, fabs(a.y - b.y));
    }
    return height;
}

int SearchGeometry::_width(BoundingBox box)
{
    int width = 0;
    cv::Point2d a,b;
    for(int i = 0; i < 4; i++)
    {
        a = box.corners[i];
        b = box.corners[0];
        if (i < 3)
            b = box.corners[i+1];
        width = max(width, fabs(a.y - b.y));
    }
    return width;
}

BoundingBox SearchGeometry::_pixelAdjust(BoundingBox box)
{
    for(int i = 0; i < 4; i++)
    {
        box.corners[i].x *= _xPixelScale;
        box.corners[i].x += _xBuffer;
        box.corners[i].y *= _yPixelScale;
        box.corners[i].y += _yBuffer;
    }
    return box;
}

cv::Scalar SearchGeometry::_randomColor() 
{
    int icolor = (unsigned)*_rng;
    return cv::Scalar( icolor&255, (icolor>>8)&255, (icolor>>16)&255);
}

void SearchGeometry::_drawBox(cv::Mat& img,BoundingBox box, bool outer)
{
    cv::Point2d a,b;
    box = _pixelAdjust(box);
    cv::Scalar color;
    if(outer)
        color = cv::Scalar(0,0,255);
    else
        color = _randomColor();
    for(int i = 0; i < 4; i++)
    {
        a = box.corners[i];
        b = box.corners[0];
        if (i < 3)
            b = box.corners[i+1];
        line(img, a, b, color, 1, 8);
    }
}

void SearchGeometry::BuildDepositBox()
{
    BoundingBox outer = deposit_border;
    std::vector<BoundingBox> inner = deposit_containers;
    
    for(int i = 0; i < inner.size(); i++)
    {
        if (!_inside(outer, inner[i]))
        {
            //ROS_ERROR("One of boxes is located outside of the border");
            std::cout<<"ERROR: One of the boxes is located outside of the border"<<"\n";
            return;
        }
    }
    int height = _height(outer) * _yPixelScale  + 2*_yBuffer;
    int width = _width(outer) * _xPixelScale + 2*_xBuffer;
    cv::Mat temp = cv::Mat::zeros(height, width, CV_8UC3);
    temp.copyTo(_depositBox);
    _drawBox(_depositBox, outer, true);
    for(int i = 0; i < inner.size(); i++)
    {
        _drawBox(_depositBox, inner[i], false);
    }
    cv::imshow("deposit box", _depositBox);
    cv::waitKey(5000);
}

void SearchGeometry::ResetDepositBox()
{
    _depositBox.setTo(0);
}

#endif