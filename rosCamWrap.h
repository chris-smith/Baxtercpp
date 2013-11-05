#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <iostream>
#include <baxter_msgs/CloseCamera.h>
#include <baxter_msgs/OpenCamera.h>
#include <baxter_msgs/ListCameras.h>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"

namespace enc = sensor_msgs::image_encodings;

cv::Mat scene;
cv::Mat templ;

#ifndef BAXTER_CAMERA_H
#define BAXTER_CAMERA_H

struct Resolution
{
    uint width;
    uint height;
    Resolution(uint a, uint b) : width(a), height(b) {}
};

class BaxterCamera
{
public:
    //default constructor subscribes to left_hand_camera
    BaxterCamera(); 
    
    //subscribes through provided node handle, to the camera name provided
    //with the perscribed buffer
    BaxterCamera(ros::NodeHandle,std::string,uint);
    BaxterCamera(std::string);
    ~BaxterCamera();
    
    //performs initialization common to both constructors
    void Init(); 
    
    // The following return their respective parts of the ROS image message
    std::string encoding();
    std_msgs::Header header();
    uint height();
    uint width();
    unsigned char is_bigendian();
    std::vector<unsigned char> data();
    std::string name();
    Resolution resolution();
    
    //returns the entire ROS image message
    sensor_msgs::Image image();
    
    void display();        // _show = true
    void display(bool);    // sets _show to true || false
    //void TimerCallback(const ros::TimerEvent &);  // custom timer callback for this object. Edit as required
    
    std::vector<std::string> list_cameras();
    int close();
    int open(Resolution);
    int resize(Resolution);               //closes then opens the camera with resolution
    /*returns an opencv formatted image
    *   Check that image exists before working with it
    *           Mat image = cam.cvImage();
    *           if(!image.empty()){...}
    */
    cv::Mat cvImage();
    
private:
    sensor_msgs::Image _img;
    std::string WINDOW; //name of window when displaying image through opencv
    unsigned int _bufferSize; //size of subscriber buffer
    ros::Subscriber _sub; //subscriber
    bool _show; //when true, _callback will display the image
    std::string _name; //name of camera
    ros::NodeHandle _nh; //node handle used to setup subscriber if one is not provided
    cv_bridge::CvImagePtr cv_ptr;

    //Subscription callback that allocates message data to 
    //the corresponding private variables
    void _callback(const sensor_msgs::Image::ConstPtr&);
    
    /*performs conversion from ROS to opencv image.
    *  CvImagePtr{ std_msgs::Header header;
    *              std::string encoding; 
    *              cv::Mat image}
    */
    cv_bridge::CvImagePtr _getCvImage();
};

BaxterCamera::BaxterCamera()
{
    _name = "left_hand_camera";
    _bufferSize = 1;
    Init();
}

BaxterCamera::~BaxterCamera()
{
    cv::destroyWindow(WINDOW);
    //cv::destroyAllWindows();
}

BaxterCamera::BaxterCamera(ros::NodeHandle nh, std::string name, uint bufferSize)
{
    //Name must be full name, e.g left_hand_camera, right_hand_camera
    _name = name;
    _nh = nh;
    _bufferSize = bufferSize;
    Init();    
}

BaxterCamera::BaxterCamera(std::string name)
{
    _name = name;
    _bufferSize = 1;
    Init();
}

void BaxterCamera::Init()
{
    WINDOW = _name;
    std::string topic = "/cameras/"+_name+"/image";
    _show = false;
    cv::namedWindow(WINDOW, CV_WINDOW_AUTOSIZE);
    _img.height = 0;
    _sub = _nh.subscribe(topic, _bufferSize, &BaxterCamera::_callback, this);
    ros::Duration timeout(2.0);
    ros::Time begin = ros::Time::now();
    //wait for an image or timeout
    while ((ros::Time::now() - begin < timeout) && (_img.height == 0))
    {
        ros::Duration(0.1).sleep();
        ros::spinOnce();
    }
    
    if (_img.height == 0)
        ROS_ERROR("Timed Out: Unable to start subscription to [%s]", topic.c_str());
}

void BaxterCamera::_callback(const sensor_msgs::Image::ConstPtr& msg)
{   
    _img.header = msg->header;
    _img.height = msg->height;
    _img.width = msg->width;
    _img.encoding = msg->encoding;
    _img.is_bigendian = msg->is_bigendian;
    _img.step = msg->step;
    _img.data = msg->data;
    std::string x = msg->encoding;
    //std::cout<<"encoding: "<< x <<std::endl;
    if (_show){
        cv_ptr = _getCvImage();
        //std::cout<<"cvencod: "<<cv_ptr->encoding<<std::endl;
        if(!cv_ptr)
            return;
        
        cv::imshow(WINDOW, cv_ptr->image);
        cv::waitKey(10);
    }
    
}

cv_bridge::CvImagePtr BaxterCamera::_getCvImage()
{
    cv_bridge::CvImagePtr temp;
    try
    {
        //std::cout<<"succesful conversion"<<std::endl;
        temp = cv_bridge::toCvCopy(_img, "");
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        
    }
    return temp;
}

cv::Mat BaxterCamera::cvImage()
{
    cv::Mat temp;
    cv_ptr = _getCvImage();
    if(!cv_ptr)
        return temp;
    
    return cv_ptr->image;
}

std::vector<std::string> BaxterCamera::list_cameras()
{
    std::string serv = "/cameras/list";
    ros::ServiceClient client = _nh.serviceClient<baxter_msgs::ListCameras>(serv);
    baxter_msgs::ListCameras srv;
    if(client.call(srv))
        return srv.response.cameras;
}

int BaxterCamera::close()
{
    std::string serv = "/cameras/close";
    ros::ServiceClient client = _nh.serviceClient<baxter_msgs::CloseCamera>(serv);
    baxter_msgs::CloseCamera srv;
    srv.request.name = _name;
    if(client.call(srv))
        return srv.response.err;
}

int BaxterCamera::open(Resolution res)
{
    std::string serv = "/cameras/open";
    ros::ServiceClient client = _nh.serviceClient<baxter_msgs::OpenCamera>(serv);
    baxter_msgs::OpenCamera srv;
    srv.request.name = _name;
    srv.request.settings.width = res.width;
    srv.request.settings.height = res.height;
    if(client.call(srv))
        return srv.response.err;
    
}

int BaxterCamera::resize(Resolution res)
{
    this->close();
    return this->open(res);
}

void BaxterCamera::display()
{
    _show = true;
}

void BaxterCamera::display(bool show)
{
    _show = show;
    //std::cout<<"will display image"<<std::endl;
}

Resolution BaxterCamera::resolution()
{
    return Resolution(_img.width,_img.height);
}

sensor_msgs::Image BaxterCamera::image()
{
    return _img;
}

std::vector<unsigned char> BaxterCamera::data()
{
   //unsigned char* x;
   //return x;
   return _img.data;
}

unsigned char BaxterCamera::is_bigendian()
{
    return _img.is_bigendian;
}

uint BaxterCamera::width()
{
    return _img.width;
}

uint BaxterCamera::height()
{
    return _img.height;
}

std_msgs::Header BaxterCamera::header()
{
    return _img.header;
}

std::string BaxterCamera::encoding()
{
    return _img.encoding;
}

std::string BaxterCamera::name()
{
    return _name;
}

/*void BaxterCamera::TimerCallback(const ros::TimerEvent &e)
{
    //--------------Put your code here--------------//
    std::string impath = "/home/ceeostud2/Pictures/";
    templ = cv::imread(impath+"gear.jpg",1);
    cv::resize(templ, templ, cv::Size(), 0.1, 0.1, cv::INTER_LINEAR);
    scene = cvImage();
    surfTest();
}*/

#endif

/*----------------------------------------------------------------------
 * Everything after this line is not a part of the BaxterCamera Class
 * ---------------------------------------------------------------------*/

void surfTest()
{
    cv::Mat img_object; cv::Mat img_scene;
    templ.copyTo(img_object);
    scene.copyTo(img_scene);
    cv::namedWindow("original", CV_WINDOW_AUTOSIZE);
    
    int minHessian = 400;
    
    /*or could use FeatureDetector::Create
     * Supports these detector types:
     *  FAST
     *  STAR
     *  SIFT
     *  SURF
     *  ORB
     *  BRISK
     *  MSER
     *  GFTT
     *  HARRIS
     *  Dense
     *  SimpleBlob
     *********************************************/
    cv::SiftFeatureDetector detector( minHessian );
    std::vector<cv::KeyPoint> keypoints_object, keypoints_scene;
    
    detector.detect( img_object, keypoints_object);
    detector.detect( img_scene, keypoints_scene);
    
    cv::SiftDescriptorExtractor extractor;
    cv::Mat descriptors_object, descriptors_scene;
    
    extractor.compute(img_object, keypoints_object, descriptors_object);
    extractor.compute(img_scene, keypoints_scene, descriptors_scene);
    
    cv::FlannBasedMatcher matcher;
    std::vector< cv::DMatch > matches;
    matcher.match( descriptors_object, descriptors_scene, matches);
    
    double max_dist = 0; double min_dist = 100;
    
    for(int i = 0; i < descriptors_object.rows; i++)
    {
        double dist = matches[i].distance;
        if(dist < min_dist) min_dist = dist;
        if(dist > max_dist) max_dist = dist;
    }
    
    //printf("-- Max dist : %f \n",max_dist);
    //printf("-- Min dist : %f \n", min_dist);
    
    std::vector< cv::DMatch > good_matches;
    
    for(int i = 0; i < descriptors_object.rows; i++)
    {
        if(matches[i].distance < 3*min_dist)
        { good_matches.push_back(matches[i]); }
    }
    cv::imshow("original", img_scene);
    cv::Mat img_matches;
    cv::drawMatches( img_object, keypoints_object, img_scene, keypoints_scene,
                 good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
                 std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    
    cv::waitKey(30);
    //imshow("Good Matches and object detection", img_matches);
    
    std::vector<cv::Point2f> obj;
    std::vector<cv::Point2f> scene;
    //printf("-- Num good matches: %d \n", good_matches.size());
    
    for(int i = 0; i < good_matches.size(); i++)
    {
        obj.push_back(keypoints_object[good_matches[i].queryIdx].pt);
        scene.push_back(keypoints_scene[good_matches[i].trainIdx].pt);
    }
    //printf("-- Num obj keypoints: %d \n", obj.size());
    //printf("-- Num scene keypoints: %d \n", scene.size());
    
    // Mat mask;
    if(obj.size() < 4)
    {
        cv::imshow("Good Matches and object detection", img_matches);
        ROS_ERROR("No matches found for the template");
        return;
    }
        
    cv::Mat H = findHomography(obj, scene, CV_RANSAC);//CV_RANSAC,CV_LMEDS
    cv::namedWindow("Good Matches and object detection", CV_WINDOW_AUTOSIZE);
    
    //printf("-- obj size: %d \n",obj.size());
    
    std::vector<cv::Point2f> obj_corners(4);
    obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( img_object.cols,0);
    obj_corners[2] = cvPoint(img_object.cols, img_object.rows); obj_corners[3] = cvPoint(0, img_object.rows);
    std::vector<cv::Point2f> scene_corners(4);
    
    cv::perspectiveTransform(obj_corners, scene_corners, H);
    
    /*for(int i = 0;i<4;i++){
        printf("-- Scene corner %d: %f %f\n",i,scene_corners[i].x,scene_corners[i].y);
        printf("-- Object corner %d: %f %f\n",i,obj_corners[i].x, obj_corners[i].y);
    }
    printf("--Img columns: %i\n",img_object.cols);
    printf("--Img rows: %i\n",img_object.rows);*/
    
    
    cv::line(img_matches, scene_corners[0] + cv::Point2f( img_object.cols,0), scene_corners[1] + cv::Point2f(img_object.cols,0), cv::Scalar(0,255,0),4);
    cv::line(img_matches, scene_corners[1] + cv::Point2f( img_object.cols,0), scene_corners[2] + cv::Point2f(img_object.cols,0), cv::Scalar(0,255,0),4);
    cv::line(img_matches, scene_corners[2] + cv::Point2f( img_object.cols,0), scene_corners[3] + cv::Point2f(img_object.cols,0), cv::Scalar(0,255,0),4);
    cv::line(img_matches, scene_corners[3] + cv::Point2f( img_object.cols,0), scene_corners[0] + cv::Point2f(img_object.cols,0), cv::Scalar(0,255,0),4);
    //circle(img_matches, Point2f(536,408), 20, (255,0,0), 2, 8,0);
    
    cv::imshow("Good Matches and object detection", img_matches);
    cv::waitKey(10);
    //imwrite(impath+"objdetect.jpg", img_matches);
    
} 
