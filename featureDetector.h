#include "rosCamWrap.h"
#include <geometry_msgs/Point.h>

class BaxterFeatureDetector{
    
public:
    BaxterFeatureDetector();
    BaxterFeatureDetector(BaxterCamera*);
    BaxterFeatureDetector(ros::NodeHandle,BaxterCamera*,uint);
    ~BaxterFeatureDetector();
    
    void displayTemplate();
    void displayTemplate(bool);
    void displayScene();
    void displayScene(bool);
    void displayMatch();
    void displayMatch(bool);
    cv::Mat getScene();
    void setTimer(ros::Duration period); // change frequency of location publish
    void startTimer(); // starts _detect_timer if it's been stopped. implements ros::timer::start()
    void stopTimer();  // stops _detect_timer. implements ros::timer::stop()
    
private:
    BaxterCamera* _scene; // holds camera object being searched for _templ
    cv::Mat _templ;
    cv::Mat _scene_img;
    void Init();
    
    std::string _name;
    unsigned int _bufferSize; //size of subscriber buffer
    ros::NodeHandle _nh; //node handle used to setup subscriber if one is not provided
    ros::Subscriber _sub; //subscriber receives new image to load for template
    ros::Publisher _pub; //publisher pushes out coordinates of detected feature
    cv_bridge::CvImagePtr cv_ptr;
    std::string TEMPL_WINDOW;
    std::string MATCH_WINDOW;
    ros::Timer _detect_timer;
    bool _showTempl;
    bool _showMatch;
    
    cv_bridge::CvImagePtr _getCvImage(const sensor_msgs::Image::ConstPtr&);
    void _callback(const sensor_msgs::Image::ConstPtr&); //loads template
    geometry_msgs::Point _getCentroid(std::vector<cv::Point2f>&); //centroid of points
    geometry_msgs::Point _detect(); //feature detect algorithm. returns centroid
    void _timer_callback(const ros::TimerEvent&);
    
};

BaxterFeatureDetector::BaxterFeatureDetector()
{
    _name = "left_hand_camera";
    _scene = new BaxterCamera(_name);
    _bufferSize = 1;
    Init();
}

BaxterFeatureDetector::BaxterFeatureDetector(BaxterCamera* cam)
{
    _scene = cam;
    _name = cam->name();
    _bufferSize = 1;
    Init();
}

BaxterFeatureDetector::BaxterFeatureDetector(ros::NodeHandle nh, BaxterCamera* cam, uint bufferSize)
{
    //Name is full name, e.g left_hand_camera, right_hand_camera
    _name = cam->name();
    _nh = nh;
    _bufferSize = bufferSize;
    _scene = cam;
    Init();    
}

BaxterFeatureDetector::~BaxterFeatureDetector()
{
    //delete _scene;   //don't need to do this since the camera was passed into feature detector
    cv::destroyWindow(TEMPL_WINDOW);
    cv::destroyWindow(MATCH_WINDOW);
}

void BaxterFeatureDetector::Init()
{
    cv::initModule_nonfree(); //initialize nonfree opencv modules
    TEMPL_WINDOW = _name + " template";
    MATCH_WINDOW = _name + " matches";
    std::string topic = "/cameras/"+_name+"/feature_detector";
    cv::namedWindow(TEMPL_WINDOW, CV_WINDOW_AUTOSIZE);
    cv::namedWindow(MATCH_WINDOW, CV_WINDOW_AUTOSIZE);
    _showMatch = false;
    _showTempl = false;
    _detect_timer = _nh.createTimer(ros::Duration(0.1), &BaxterFeatureDetector::_timer_callback, this);
    _sub = _nh.subscribe(topic, _bufferSize, &BaxterFeatureDetector::_callback, this);
    _pub = _nh.advertise<geometry_msgs::Point>(topic+"/location",_bufferSize);
}

void BaxterFeatureDetector::_callback(const sensor_msgs::Image::ConstPtr& msg)
{
    _templ =  _getCvImage(msg)->image;
    if(_showTempl)
        cv::imshow(TEMPL_WINDOW, _templ);
}

cv_bridge::CvImagePtr BaxterFeatureDetector::_getCvImage(const sensor_msgs::Image::ConstPtr& img)
{
    cv_bridge::CvImagePtr temp;
    try
    {
        //std::cout<<"succesful conversion"<<std::endl;
        temp = cv_bridge::toCvCopy(img , "");
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        
    }
    return temp; 
}

cv::Mat BaxterFeatureDetector::getScene()
{
    return _scene->cvImage();
}

void BaxterFeatureDetector::displayTemplate()
{
    _showTempl = true;
}

void BaxterFeatureDetector::displayTemplate(bool show)
{
    _showTempl = show;
}

void BaxterFeatureDetector::displayScene()
{
    _scene->display();
}

void BaxterFeatureDetector::displayScene(bool show)
{
    _scene->display(show);
}

void BaxterFeatureDetector::displayMatch()
{
    _showMatch = true;
}

void BaxterFeatureDetector::displayMatch(bool show)
{
    _showMatch = show;
}

void BaxterFeatureDetector::setTimer(ros::Duration period)
{
    stopTimer();
    _detect_timer.setPeriod(period);
    startTimer();
}

void BaxterFeatureDetector::startTimer()
{
    _detect_timer.start();
}

void BaxterFeatureDetector::stopTimer()
{
    _detect_timer.stop();
}

void BaxterFeatureDetector::_timer_callback(const ros::TimerEvent&)
{
    _scene_img = _scene->cvImage(); //get the most recent scene image
    if(_templ.empty() || _scene_img.empty())
    {
        ROS_ERROR("A template needs to be loaded before detection can be performed");
        return;
    }
    
    geometry_msgs::Point loc = _detect(); //run detection algorithm
    _pub.publish(loc);
}

geometry_msgs::Point BaxterFeatureDetector::_getCentroid(std::vector<cv::Point2f>& cv_point)
{
    geometry_msgs::Point point;
    point.z = -1; //only looking at 2d -- no depth
    if(cv_point.size() <= 0)
    {
        point.x = -1;
        point.y = -1;
        return point;
    }
    point.x = 0;
    point.y = 0;
    for(int i = 0; i < cv_point.size(); i++)
    {
        point.x += cv_point[i].x;
        point.y += cv_point[i].y;
    }
    point.x /= cv_point.size(); // get average of point.x
    point.y /= cv_point.size();
    
    return point;
}

geometry_msgs::Point BaxterFeatureDetector::_detect()
{
    cv::Mat img_object; cv::Mat img_scene;
    _templ.copyTo(img_object);
    _scene_img.copyTo(img_scene);
    //cv::namedWindow("original", CV_WINDOW_AUTOSIZE);
    
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
    //cv::imshow("original", img_scene);
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
        //cv::imshow("Good Matches and object detection", img_matches);
        ROS_ERROR("No matches found for the template");
        geometry_msgs::Point p;
        p.x = -1; p.y = -1; p.z = -1;
        return p;
    }
        
    cv::Mat H = findHomography(obj, scene, CV_RANSAC);//CV_RANSAC,CV_LMEDS
    //cv::namedWindow("Good Matches and object detection", CV_WINDOW_AUTOSIZE);
    
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
    
    //cv::imshow("Good Matches and object detection", img_matches);
    if(_showMatch)
        cv::imshow(MATCH_WINDOW, img_matches);
    cv::waitKey(10);
    return _getCentroid(scene_corners);
    //imwrite(impath+"objdetect.jpg", img_matches);
   
}