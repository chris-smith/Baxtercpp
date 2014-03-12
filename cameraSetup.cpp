#include "baxterCam.h"

int thresh_val = 0;
int max_value = 255;

cv::Mat left, right;
char* trackbar_value = "Value";
char* window_name = "HSV Conversion";

void threshold(int, void*);

int main(int argc, char** argv) {
    ros::init(argc, argv, "Cam_Setup");
    //cv::initModule_nonfree(); //initialize nonfree opencv modules
    
    BaxterCamera right_cam("right_hand_camera");
    right_cam.display();
    ros::spinOnce();
    std::cout<<"Setting up right cam...\n";
    right_cam.resolution(Resolution(320,200));
    right_cam.half_resolution(false);
    right_cam.window(Resolution(560,190)); // (570, 150)
    right_cam.gain(15);
    right_cam.exposure(10);
    
    BaxterCamera left_cam("left_hand_camera");
    left_cam.display();
    ros::spinOnce();
    /*std::cout<<"Setting up left_cam...\n";
    left_cam.resolution(Resolution(320,200));
    left_cam.half_resolution(false);
    left_cam.window(Resolution(555,220)); // (570, 150)
    left_cam.gain(15);
    */
    std::cout<<"Starting trackbar...\n";
    cv::namedWindow(window_name, CV_WINDOW_AUTOSIZE);
    
    cv::createTrackbar("Threshold", window_name, &thresh_val, max_value, threshold);
    threshold(0,0);
    
    while(ros::ok()) {
        left = left_cam.cvImage();
        right = right_cam.cvImage();
        ros::spinOnce();
    }
    
    return 0;
}

void threshold(int, void*) {
    if (right.empty())
        return;
    cv::Mat thresh;
    try {
        cv::cvtColor(right, thresh, CV_RGB2GRAY);
    }
    catch (cv::Exception) {
        ROS_ERROR("Unable to convert image to greyscale");
        //return temp;
    }
    cv::threshold(thresh, thresh,100,255,cv::THRESH_BINARY_INV);
    //thresh = thresh - _gripper_mask;
    int type = cv::MORPH_ELLIPSE;
    int sz = 1;
    cv::Mat element = getStructuringElement( type,
                                             cv::Size(2*sz+1, 2*sz+1),
                                             cv::Point(sz, sz) );
    //cv::erode(thresh, thresh, element); 
    
    std::vector< std::vector< cv::Point > > contours;
    std::vector< cv::Vec4i > hierarchy;
    int largest_area = 0;
    int largest_contour_index = 0;
    
    //cv::Canny(thresh, canny, 5, 250, 3);
    cv::imshow(window_name, thresh);
    
    cv::waitKey(30);
    
}