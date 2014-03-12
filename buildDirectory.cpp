#include "objectClassifier.h"
#include "nxtKit.h"

cv::Mat gripper_mask;
cv::Mat image;
cv::Mat object;
BaxterCamera *right_cam;
ObjectClassifier *classifier;

void showHelp();
void remove_grippers();
void setClassifierImage(int&);
std::vector<cv::Point> getBlob(cv::Mat);
cv::Rect get_bounds(std::vector<cv::Point>);
void setup_pid(BaxterLimb&);

int main(int argc, char** argv) {
    ros::init(argc, argv, "Cam_Test");
    cv::initModule_nonfree(); //initialize nonfree opencv modules
    
    right_cam = new BaxterCamera("right_hand_camera");
    classifier = new ObjectClassifier("/home/ceeostud2/Pictures/ObjectTemplates/", 300);
    NxtKit kit( cv::RotatedRect(), 0 );
    
    std::string part_name;
    remove_grippers();
    right_cam->display();
    gv::Point pos;
    JointPositions jp;
    
    std::cout << "Press 'h' to see command options\n";
    
    int key;
    int padding = 0;  // padding on subimage rectangle for object
    ros::Duration tout(.5);
    ros::spinOnce();
    while ( ros::ok() ){
        key = cv::waitKey(50);
        switch(key) {
            case 'h':
                //std::cout<<"key is h\n";
                showHelp();
                break;
            case 's':
                //save image
                if (part_name != "")
                    classifier->save_scene(part_name);
                else
                    std::cout<<"Specify which part you're adding\n";
                break;
            case 'p':
                kit.show_parts();
                break;
            case 'n':
                std::cout<<"What part are you adding?\n";
                std::cin.clear();
                std::cin>>part_name;
                if ( kit.contains(part_name) )
                    std::cout<<"Adding part "<<part_name<<"\n";
                else {
                    std::cout<<"That part is not contained within the provided kit\n";
                    part_name = "";
                }
                break;
            case 'b':
                std::cout<<"Part named "<<part_name<<"\n";
                break;
            case 'm':
                //if (!object.)
                classifier->match();
                break;
            case 45:
                // '-' --> decrease padding around object
                if (padding > 0)
                    padding -= 2;
                std::cout<<"new padding "<<padding<<"\n";
                break;
            case 61:
                // '+' --> increase padding around object
                padding += 2;
                std::cout<<"new padding "<<padding<<"\n";
                break;
            case 'r':
                remove_grippers();
                break;
                
            default:
                setClassifierImage(padding);
                if (!object.empty()) {
                    classifier->scene(object);
                    cv::imshow("object", object);
                }
                else {
                    cv::imshow("object", image);
                    classifier->scene( image );
                }
                //pos = limb.endpoint_pose().position;
                if (key > 0)
                    std::cout<<"unknown key: "<<key<<"\n";
                break;
        }
        ros::spinOnce();
    }
    
    return 1;
}

void showHelp() {
    std::string msg;
    msg = "Key Codes\n\th --\tHelp\n\ts --\tAdd Image to Database\n";
    msg += "\tp --\tShow Objects in Kit\n\tn --\tSet Part Name\n";
    msg += "\tb --\tShow Current Part Name\n\tm --\tMatch Current View\n";
    msg += "\t- --\tDecrease Padding\n\t+ --\tIncrease Padding\n";
    msg += "\tr --\tReset Mask\n";
    std::cout<<msg;
}

void setClassifierImage(int& padding) {
    image = right_cam->cvImage();
    std::vector<cv::Point> blob = getBlob( image );
    cv::Rect rect;
    
    if (!blob.empty()) {
        rect = get_bounds(blob);
        if (padding > 0) {
            //std::cout<<"padding...\n";
            cv::Size size(rect.width+padding, rect.height+padding);
            cv::Point2d org(rect.x-padding/2, rect.y-padding/2);
            cv::Rect padded(org, size);
            //std::cout<<padded.x <<" "<<padded.y<<" "<<padded.width<<" "<<padded.height<<"\n";
            // if padded doesn't contain the scene's top-left or 
            //  bottom-right corners, padded is contained within scene
            if ( !padded.contains(cv::Point2d(0,0)) && !padded.contains(cv::Point2d( right_cam->width(), right_cam->height() ) ) )
                rect = padded;
        }
        try {
            object = image(rect);
        }
        catch (cv::Exception) {
            ROS_ERROR("Padding was probably too large. Padding reset to 0");
            padding = 0;
        }
    }
    else
        object = image;   // if no object is found, pass it scene
}

void remove_grippers() {
    std::cout<<"Removing grippers from scene\n";
    //if (!_object())
    //    return;
    std::vector<cv::Point> blob;
    cv::Mat scene = right_cam->cvImage();
    if (scene.empty()){
        ROS_ERROR("Empty scene :/");
        remove_grippers();
    }
    //std::cout<<"scene type "<<scene.type()<<"\n";
    cv::Mat mask;
    int type = cv::MORPH_ELLIPSE;
    int sz = 2;
    cv::Mat element = getStructuringElement( type,
                                             cv::Size(2*sz+1, 2*sz+1),
                                             cv::Point(sz, sz) );
    cv::Mat thresh;
    try {
        cv::cvtColor(scene, thresh, CV_BGR2GRAY);     
    }
    catch (cv::Exception) {
        ROS_ERROR("Unable to convert image to greyscale");
        return;
    }
    cv::threshold(thresh, thresh,50,255,cv::THRESH_BINARY_INV); 
    cv::dilate(thresh, mask, element);   
    cv::dilate(mask, mask, element);
    
    gripper_mask = mask;
}

std::vector<cv::Point> getBlob(cv::Mat scene)
{
    // finds outline of the largest non-white object
    std::vector<cv::Point> temp(0,cv::Point(0,0));
    if (scene.empty())
    {
        //ROS_ERROR("Image to search is empty");
        return temp;
    }
    cv::Mat thresh;
    try {
        cv::cvtColor(scene, thresh, CV_BGR2GRAY);
    }
    catch (cv::Exception) {
        ROS_ERROR("Unable to convert image to greyscale");
        return temp;
    }
    cv::threshold(thresh, thresh,100,255,cv::THRESH_BINARY_INV);
    thresh = thresh - gripper_mask;
    int type = cv::MORPH_ELLIPSE;
    int sz = 1;
    cv::Mat element = getStructuringElement( type,
                                             cv::Size(2*sz+1, 2*sz+1),
                                             cv::Point(sz, sz) );
    //cv::erode(thresh, thresh, element); 
    
    std::vector< std::vector< cv::Point > > contours;
    std::vector< cv::Vec4i > hierarchy;
    int min_area = 75;
    int largest_area = 0;
    int largest_contour_index = 0;
    
    cv::imshow("thresholded", thresh);
    cv::findContours(thresh, contours, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
    
    int iters = contours.size();
    for(int i = 0; i < iters; i++)
    {
        double a = cv::contourArea(contours[i], false);
        if ( a > largest_area)
        {
            largest_area = a;
            largest_contour_index = i;
        }
    }
    //std::cout<<"blob area: "<<largest_area<<"\n";
    cv::imshow("blob", thresh);
    if (largest_area < min_area)
    {
        return temp;
    }
    
    return contours[largest_contour_index];
}

std::vector<cv::Point> _increase_bounds(std::vector<cv::Point> rect, cv::Point new_point)
{
    //std::vector<cv::Point> corners = _corners(rect);
    double ymin, ymax, xmin, xmax, x, y;
//     std::cout<<"new point: "<<new_point.x <<" "<< new_point.y<<std::endl;
//     std::cout<<"xmin: "<<xmin<<" xmax: "<<xmax<<" ymin: "<<ymin<<" ymax: "<<ymax<<std::endl;
    x = new_point.x;
    y = new_point.y;
    if (x > rect[1].x)
        rect[1].x = x;
    else if(x < rect[0].x)
        rect[0].x = x;
    
    if (y > rect[1].y)
        rect[1].y = y;
    else if(y < rect[0].y)
        rect[0].y = y;
    
    
    /*std::cout<<"new origin: "<<ret.x<<" "<<ret.y<<std::endl;
    std::cout<<"new size: "<<ret.width<<" "<<ret.height<<std::endl;*/
    return rect;
}

cv::Rect get_bounds(std::vector<cv::Point> contour)
{
//     double height = _search_cam->height();
//     double width = _search_cam->width();
    std::vector<cv::Point> bounds(2,cv::Point(0,0));
    bounds[0].x = right_cam->width();  // xmin
    bounds[0].y = right_cam->height(); // ymin
    bounds[1].x = 0;      // xmax
    bounds[1].y = 0;      // ymax
    cv::Point current_point;
    //std::cout<<"num points: "<<contour.size()<<std::endl;
    for(int i = 0; i < contour.size(); i++)
    {
        current_point = contour[i];
//         std::cout<<"origin: "<<bounds.x<<" "<<bounds.y<<std::endl;
//         std::cout<<"size: "<<bounds.width<<" "<<bounds.height<<std::endl;
        bounds = _increase_bounds(bounds, current_point);
    }
    return cv::Rect(bounds[0], bounds[1]);
    //return _restrict(bounds, scene_height, scene_width);
}

void setup_pid(BaxterLimb& limb)
{
    limb.set_joint_pid("right_w2", (Gains){3.5, .5, 0.1}); //6
    limb.set_joint_pid("right_w1", (Gains){4, .5, 0.05});
    limb.set_joint_pid("right_w0", (Gains){3.5, .1, 0.002});
    limb.set_joint_pid("right_e1", (Gains){3.5, .1, 0.15});
    limb.set_joint_pid("right_e0", (Gains){3.25, .1, 0.002});
    limb.set_joint_pid("right_s1", (Gains){2, .03, 0.05});
    limb.set_joint_pid("right_s0", (Gains){2.25, .4, 0.015}); //0
    limb.set_allowable_error(0.008);
    limb.set_max_velocity(1.5);
    limb.set_max_acceleration(30);
    Gains epGains(.6,.25,.05);
    //limb.set_endpoint_pid(epGains);
}