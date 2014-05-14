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

#ifndef BAXTER_CAMERA_CONTROLS
#define BAXTER_CAMERA_CONTROLS

#define CAMERA_CONTROL_EXPOSURE baxter_msgs::CameraControl::CAMERA_CONTROL_EXPOSURE
#define CAMERA_CONTROL_GAIN baxter_msgs::CameraControl::CAMERA_CONTROL_GAIN
#define CAMERA_CONTROL_WHITE_BALANCE_R baxter_msgs::CameraControl::CAMERA_CONTROL_WHITE_BALANCE_R
#define CAMERA_CONTROL_WHITE_BALANCE_G baxter_msgs::CameraControl::CAMERA_CONTROL_WHITE_BALANCE_G
#define CAMERA_CONTROL_WHITE_BALANCE_B baxter_msgs::CameraControl::CAMERA_CONTROL_WHITE_BALANCE_B
#define CAMERA_CONTROL_WINDOW_X baxter_msgs::CameraControl::CAMERA_CONTROL_WINDOW_X
#define CAMERA_CONTROL_WINDOW_Y baxter_msgs::CameraControl::CAMERA_CONTROL_WINDOW_Y
#define CAMERA_CONTROL_FLIP baxter_msgs::CameraControl::CAMERA_CONTROL_FLIP
#define CAMERA_CONTROL_MIRROR baxter_msgs::CameraControl::CAMERA_CONTROL_MIRROR
#define CAMERA_CONTROL_RESOLUTION_HALF baxter_msgs::CameraControl::CAMERA_CONTROL_RESOLUTION_HALF

#endif

#ifndef BAXTER_CAMERA_H
#define BAXTER_CAMERA_H

struct Resolution {
    uint width;
    uint height;
    Resolution(uint a, uint b) : width(a), height(b) {}
};

class BaxterCamera {
public:
    //default constructor subscribes to left_hand_camera
    BaxterCamera(); 
    
    // possible provide name, buffer size
    BaxterCamera(std::string,uint);
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
    
    //Adjust CameraSettings
    Resolution resolution();
    void resolution(Resolution);
    int fps();
    void fps(int);
    int exposure();
    void exposure(int);
    int gain();
    void gain(int);
    int white_balance_red();
    void white_balance_red(int);
    int white_balance_green();
    void white_balance_green(int);
    int white_balance_blue();
    void white_balance_blue(int);
    Resolution window();
    void window(Resolution);
    bool flip();
    void flip(bool);
    bool mirror();
    void mirror(bool);
    bool half_resolution();
    void half_resolution(bool);
    void reset_controls();
    
    //returns the entire ROS image message
    sensor_msgs::Image image();
    
    void display();        // _show = true
    void display(bool);    // sets _show to true || false
    
    // Camera Services
    std::vector<std::string> list_cameras();
    int close();
    int open();
    
    /*returns an opencv formatted image
    *   Check that image exists before working with it
    *           Mat image = cam.cvImage();
    *           if(!image.empty()){...}
    */
    cv::Mat cvImage();
    
private:
    sensor_msgs::Image _img;
    baxter_msgs::CameraSettings _settings;
    std::string WINDOW;                 //name of window when displaying image through opencv
    unsigned int _bufferSize;           //size of subscriber buffer
    ros::Subscriber _sub;               //image subscriber
    ros::ServiceClient _list_client;
    ros::ServiceClient _close_client;
    ros::ServiceClient _open_client;
    bool _show;                         //when true, _callback will display the image
    std::string _name;                  //name of camera
    ros::NodeHandle _nh;                //node handle used to setup subscriber if one is not provided
    cv_bridge::CvImagePtr cv_ptr;

    //Subscription callback that allocates message data to 
    //the corresponding private variables
    void _callback(const sensor_msgs::Image::ConstPtr&);
    
    int _reload();                      //closes then opens the camera with _settings
    
    int _get_control_value(int, int);   // control id, default
    void _set_control_value(int, int);  // control id, value
    
    /*performs conversion from ROS to opencv image.
    *  CvImagePtr{ std_msgs::Header header;
    *              std::string encoding; 
    *              cv::Mat image}
    */
    cv_bridge::CvImagePtr _getCvImage();
};

BaxterCamera::BaxterCamera() {
    _name = "left_hand_camera";
    _bufferSize = 1;
    Init();
}

BaxterCamera::BaxterCamera(std::string name, uint bufferSize) {
    //Name must be full name, e.g left_hand_camera, right_hand_camera
    _name = name;
    _bufferSize = bufferSize;
    Init();    
}

BaxterCamera::BaxterCamera(std::string name) {
    _name = name;
    _bufferSize = 1;
    Init();
}

BaxterCamera::~BaxterCamera() {
    cv::destroyWindow(WINDOW);
}

void BaxterCamera::Init() {
    // initialization common to constructors
    WINDOW = _name;
    std::string topic = "/cameras/"+_name+"/image";
    _show = false;
    _img.height = 0;
    
    // setup subs, services
    _sub = _nh.subscribe(topic, _bufferSize, &BaxterCamera::_callback, this);
    _list_client = _nh.serviceClient<baxter_msgs::ListCameras>("/cameras/list");
    _close_client = _nh.serviceClient<baxter_msgs::CloseCamera>("/cameras/close");
    _open_client = _nh.serviceClient<baxter_msgs::OpenCamera>("/cameras/open");
    // default settings
    _settings.width = 320;
    _settings.height = 200;
    _settings.fps = 20;
    
    //wait for an image or timeout
    ros::Duration timeout(2.0);
    ros::Time begin = ros::Time::now();
    while ( (ros::Time::now() - begin < timeout) && (_img.height == 0)) {
        ros::Duration(0.1).sleep();
        ros::spinOnce();
    } 
    if (_img.height == 0)
        ROS_ERROR("Timed Out: Unable to start subscription to [%s]", topic.c_str());
}

void BaxterCamera::_callback(const sensor_msgs::Image::ConstPtr& msg) {   
    // image subscriber callback
    
    // copy data over
    _img.header = msg->header;
    _img.height = msg->height;
    _img.width = msg->width;
    _img.encoding = msg->encoding;
    _img.is_bigendian = msg->is_bigendian;
    _img.step = msg->step;
    _img.data = msg->data;
    
    // show image if we're supposed to
    if (_show) {
        cv_ptr = _getCvImage();
        if(!cv_ptr)
            return;    
        cv::imshow(WINDOW, cv_ptr->image);
        cv::waitKey(10);
    }   
}

int BaxterCamera::_get_control_value(int control_id, int defaul) {
    // return value for specified control 
    // control ids from BAXTER_CAMERA_CONTROLS at top
    int index = -1;
    // see if control has been set
    for(int i = 0; i < _settings.controls.size(); i++) {
        if (_settings.controls[i].id == control_id) {
            index = i; break;
        }
    }
    // if set, return value
    if (index > -1)
        return _settings.controls[index].value;
    return defaul;
}

void BaxterCamera::_set_control_value(int control_id, int val) {
    // set value for specified camera control
    // control_ids from BAXTER_CAMERA_CONTROLS at top
    int index = -1;
    // see if control has been added to list yet
    for(int i = 0; i < _settings.controls.size(); i++) {
        if(_settings.controls[i].id == control_id) {
            index = i; break;
        }
    }
    // if it has, change its value to new one
    if(index > -1) {
        _settings.controls[index].value = val;
        return;
    }
    // else add new control value
    baxter_msgs::CameraControl ctl;
    ctl.id = control_id;
    ctl.value = val;
    _settings.controls.push_back(ctl);
}

cv_bridge::CvImagePtr BaxterCamera::_getCvImage() {
    // used internally to convert ROS message to openCV
    
    cv_bridge::CvImagePtr temp;
    try {
        temp = cv_bridge::toCvCopy(_img, "");
    }
    catch(cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());   
    }
    return temp;
}

cv::Mat BaxterCamera::cvImage() {
    // return openCV formatted image
    
    cv::Mat temp;
    cv_ptr = _getCvImage();
    if(!cv_ptr)
        return temp;
    
    return cv_ptr->image;
}

std::vector<std::string> BaxterCamera::list_cameras() {
    // return a list of available cameras on baxter
    baxter_msgs::ListCameras srv;
    if(_list_client.call(srv))
        return srv.response.cameras;
}

int BaxterCamera::close() {
    // close this camera using provided service
    
    baxter_msgs::CloseCamera srv;
    srv.request.name = _name;
    if(this->_close_client.call(srv))
        return srv.response.err;
    return 1;
}

int BaxterCamera::open() {
    // open this camera using provided service
    
    baxter_msgs::OpenCamera srv;
    srv.request.name = this->_name;
    // head camera has special default settings
    if (this->_name == "head_camera") {
        this->_set_control_value(CAMERA_CONTROL_FLIP, true);
        this->_set_control_value(CAMERA_CONTROL_MIRROR, true);
    }
    srv.request.settings = this->_settings;
    if(this->_open_client.call(srv))
        return srv.response.err;
    return 1;
}

int BaxterCamera::_reload() {
    // used internally to reload camera after adjusting settings
    
    bool close = false;
    bool open = false;
    if (this->close() != 0)
        close = true;
    if( this->open() != 0)
        open = true;
    // check if camera closed correctly
    if (close && open) {
        throw("Unable to reload camera");
        return 1;
    }
    else if(close)
        throw("Unable to close camera");
    else if(open)
        throw("Unable to open camera");
    return 0;
}

void BaxterCamera::display() {
    // tell this object to automatically display all images to the screen
    _show = true;
    cv::namedWindow(WINDOW, CV_WINDOW_AUTOSIZE);
}

void BaxterCamera::display(bool show) {
    // start, stop automatically displaying images to screen
    if(show)
        cv::namedWindow(WINDOW, CV_WINDOW_AUTOSIZE);
    else
        cv::destroyWindow(WINDOW);
    _show = show;
}

Resolution BaxterCamera::resolution() {
    return Resolution(_img.width,_img.height);
}

void BaxterCamera::resolution(Resolution res) {
    this->_settings.width = res.width;
    this->_settings.height = res.height;
    this->_reload();
}

int BaxterCamera::fps() {
    return this->_settings.fps;
}

void BaxterCamera::fps(int val) {
    this->_settings.fps = val;
}

int BaxterCamera::exposure() {
    return _get_control_value(CAMERA_CONTROL_EXPOSURE, -1);
}

void BaxterCamera::exposure(int val) {
    if ((val < 0 || val > 100) && val != -1){
        throw(std::out_of_range("Invalid exposure value"));
        return;
    }
    this->_set_control_value(CAMERA_CONTROL_EXPOSURE, val);
    this->_reload();
}

int BaxterCamera::gain() {
    return _get_control_value(CAMERA_CONTROL_GAIN, -1);
}

void BaxterCamera::gain(int val) {
    if ((val < 0 || val > 79) && val != -1){
        throw(std::out_of_range("Invalid gain value"));
        return;
    }
    this->_set_control_value(CAMERA_CONTROL_GAIN, val);
    this->_reload();
}

int BaxterCamera::white_balance_red() {
    return this->_get_control_value(CAMERA_CONTROL_WHITE_BALANCE_R, -1);
}

void BaxterCamera::white_balance_red(int val) {
    if ((val < 0 || val > 4095) && val != -1){
        throw(std::out_of_range("Invalid white balance value"));
        return;
    }
    this->_set_control_value(CAMERA_CONTROL_WHITE_BALANCE_R, val);
    this->_reload();
}

int BaxterCamera::white_balance_green() {
    return this->_get_control_value(CAMERA_CONTROL_WHITE_BALANCE_G, -1);
}

void BaxterCamera::white_balance_green(int val) {
    if ((val < 0 || val > 4095) && val != -1){
        throw(std::out_of_range("Invalid white balance value"));
        return;
    }
    this->_set_control_value(CAMERA_CONTROL_WHITE_BALANCE_G, val);
    this->_reload();
}

int BaxterCamera::white_balance_blue() {
    return this->_get_control_value(CAMERA_CONTROL_WHITE_BALANCE_B, -1);
}

void BaxterCamera::white_balance_blue(int val) {
    if ((val < 0 || val > 4095) && val != -1){
        throw(std::out_of_range("Invalid white balance value"));
        return;
    }
    this->_set_control_value(CAMERA_CONTROL_WHITE_BALANCE_B, val);
    this->_reload();
}

Resolution BaxterCamera::window() {
    int x = this->_get_control_value(CAMERA_CONTROL_WINDOW_X, -1);
    if (x == -1){
        if(this->half_resolution())
            return Resolution(x/2, this->resolution().height);
        else
            return this->resolution();
    }
    else
        return Resolution(x,this->_get_control_value(CAMERA_CONTROL_WINDOW_Y, -1));    
}

void _coerce(int& val, const int& min, const int& max) {
    // coerce value to within range
    if (val > max)
        val = max;
    else if (val < min)
        val = min;
}

void BaxterCamera::window(Resolution res) {
    /*
     * Set Camera window. The max size is a function of the current camera
     * resolution and if half_resolution is enabled or not.
     */
    int x = res.width;
    int y = res.height;
    int limit_x = 1280 - this->resolution().width;
    int limit_y = 800 - this->resolution().height;
    
    if (this->half_resolution()){
        limit_x /= 2;
        limit_y /= 2;
    }
    
    if (x < 0 || x > limit_x){
        ROS_ERROR("Tried to set X window to %d. Coercing value to within range [%d,%d]", x,0,limit_x);
        _coerce(x,0,limit_x);        
    }
    if (y < 0 || y > limit_y){
        ROS_ERROR("Tried to set Y window to %d. Coercing value to within range [%d,%d]", x,0,limit_y);
        _coerce(y,0,limit_y);
    }
    
    this->_set_control_value(CAMERA_CONTROL_WINDOW_X, x);
    this->_set_control_value(CAMERA_CONTROL_WINDOW_Y, y);
    this->_reload();
}

bool BaxterCamera::flip() {
    return this->_get_control_value(CAMERA_CONTROL_FLIP, true);
}

void BaxterCamera::flip(bool val) {
    this->_set_control_value(CAMERA_CONTROL_FLIP, val);
    this->_reload();
}

bool BaxterCamera::mirror() {
    return this->_get_control_value(CAMERA_CONTROL_MIRROR, true);
}

void BaxterCamera::mirror(bool val) {
    this->_set_control_value(CAMERA_CONTROL_MIRROR, val);
    this->_reload();
}

bool BaxterCamera::half_resolution() {
    return this->_get_control_value(CAMERA_CONTROL_RESOLUTION_HALF, true);
}

void BaxterCamera::half_resolution(bool val) {
    this->_set_control_value(CAMERA_CONTROL_RESOLUTION_HALF, val);
    this->_reload();
}

void BaxterCamera::reset_controls() {
    this->_settings.controls.clear();
}

sensor_msgs::Image BaxterCamera::image() {
    return this->_img;
}

std::vector<unsigned char> BaxterCamera::data() {
   return _img.data;
}

unsigned char BaxterCamera::is_bigendian() {
    return _img.is_bigendian;
}

uint BaxterCamera::width() {
    return _img.width;
}

uint BaxterCamera::height() {
    return _img.height;
}

std_msgs::Header BaxterCamera::header() {
    return _img.header;
}

std::string BaxterCamera::encoding() {
    return _img.encoding;
}

std::string BaxterCamera::name() {
    return _name;
}

#endif
