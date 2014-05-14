#include "baxterCam.h"
#include "baxterLimb.h"
#include <geometry_msgs/Point.h>
#include <string>
#include <dirent.h>
#include <stdlib.h>
#include <sstream>

#define OBJ_CLS_DO_NOT_LOAD false


#ifndef OBJECT_CLASSIFIER_H
#define OBJECT_CLASSIFIER_H

enum ObjectShape { LINE, TRIANGLE, CIRCLE, RECTANGLE, UNKNOWN };

class ObjectModel {
public:
    // build an object of the model as new information about it is discovered
    // this allows a decision to be made at the final stage of part deposition
    // about what part this actually is
    double aspect_ratio;        // width : height
    float grip_width;
    std::vector< cv::Point > minimalBlob;
    std::vector< double > angles;
    bool convex, isSquare;
    ObjectShape shape;
    // these quantities are scaled by the height offset of the camera
    double length, width;       // of minimum bounding rect. assert length > width
    double area, area_ratio;    // area of outer bounds
    double internal_area;        // counts white pixels in view
    ObjectModel() : aspect_ratio(-1), grip_width(-1), convex(false), shape(UNKNOWN) {}
    
    void print() {
        std::cout << "\nObject Model--\n";
        std::cout << "Aspect Ratio: " << aspect_ratio;
        std::cout << "\nGrip Width: " << grip_width;
        std::cout << "\nConvex: " << (convex ? "true" : "FALSE");
        std::cout << "\nMinimal Blob Size: " << minimalBlob.size();
        std::cout << "\nLength: " << length;
        std::cout << "\nWidth: " << width;
        std::cout << "\nArea: " << area;
        std::cout << "\nArea Ratio: " << area_ratio;
        std::cout << "\nInternal Area: " << internal_area;
        std::cout << "\nShape: ";
        switch(shape) {
            case 0:
                std::cout << "Line";
                break;
            case 1:
                std::cout << "Triangle";
                break;
            case 2:
                std::cout << "Circle";
                break;
            case 3:
                std::cout << "Rectangle";
                break;
            default:
                std::cout << "Unknown";
                break;
        }
        std::cout << "\n";
    }
};

class Line {
public:
    Line() : pt1(), pt2() {}
    Line(cv::Point a, cv::Point b) : pt1(a), pt2(b) {}
    cv::Point pt1, pt2;
    
    double length() {
        // return length of this line
        double dx = this->pt1.x - this->pt2.x;
        double dy = this->pt1.y - this->pt2.y;
        return sqrt( dx*dx + dy*dy );
    };
    int dot(Line b) {
        double dx1, dx2, dy1, dy2, m1, m2;
        dx1 = this->pt1.x - this->pt2.x;
        dy1 = this->pt1.y - this->pt2.y;
        dx2 = b.pt1.x - b.pt2.x;
        dy2 = b.pt1.y - b.pt2.y;
        return dx1*dx2 + dy1*dy2;
        
    }
    double angle(Line b, int units=0) {
        // return angle between this line and line b
        // AdotB = len(A)*len(B)*cos(theta);
        if (units)
            return 180*acos( this->dot(b) / ( this->length() * b.length() ) ) / PI;
        return acos( this->dot(b) / ( this->length() * b.length() ) );
    };
private:
};

class ObjectClassifier
{
public:
    ObjectClassifier(std::string);              // provide path to db images
    ObjectClassifier(std::string, bool);        // load images into db?
    ObjectClassifier(std::string, int);         // _min_hessian
    
    void scene(const cv::Mat&);       //set the image to search through
    std::string match();
    std::string match(const cv::Mat&);
    std::string getFinalDecision(ObjectModel, std::string);
    MatchParams* conditionalMatch(const cv::Mat&);
    void save_scene();                // prompts user to save scene in specified directory
    void save_scene(std::string);
    void save_scene(std::string, const cv::Mat&);
    void reload();                      // reload db
    
    bool is_circle( std::vector< cv::Point > );
    bool is_rectangular();
    bool is_rectangular(const cv::Mat&);
    bool is_rectangular(std::vector<cv::Point>);
    bool is_rectangular(std::vector< cv::Point >, ObjectModel&);
    float aspect_ratio();
    float aspect_ratio(std::vector<cv::Point>);
    double angle(cv::Point, cv::Point, cv::Point, uint);
    std::vector<cv::RotatedRect> rectangles(std::vector<cv::Point>);
    std::vector<Line> interiorLines(std::vector<cv::Point>);
    void drawLine(Line, int delay=1000);
    bool inBlob(Line, std::vector<cv::Point>);
    bool intersect(Line a, Line b);
    std::vector<cv::RotatedRect> rectanglesFromDiagonals(std::vector<Line>);
    std::vector<cv::Point> corners(const std::vector<cv::Point>);
    
private:
    ObjectClassifier();
    
    // variables
    cv::Mat _scene;
    DIR* _db_dir;
    bool _valid_path;
    std::vector<cv::Mat> _db_images;
    std::vector<std::string> _db_names;
    std::vector<std::string> _db_image_types;
    uint _num_types;
    std::vector<std::vector<cv::KeyPoint> > _db_keypoints;
    std::vector<cv::KeyPoint> _query_keypoints;
    std::vector<cv::Mat> _db_descriptors;
    cv::Mat _query_descriptors;
    cv::FlannBasedMatcher _matcher;
    int _min_hessian;
    std::vector<cv::DMatch> _matches;
    std::vector<std::string> _valid_types;      // valid image file types to be placed in db
    std::string _dir_path;
    
    // Load images
    void _check_path(std::string&);
    void _load_images();
    void _load_images_recurse(std::string);
    void _set_valid_types();
    
    // Save new templates
    void _save_image();
    void _get_name_for_image();                         // used to get the file name for image saving
    int _get_num_files(std::string);
    std::vector<std::string> _get_files_names();        // returns a vector with the names of all files located in the directory path
    
    // Matcher setup
    void _train_matcher();      // extracts descriptors for each object in provided path

    // Try to match specific shapes
    double _dist(cv::Point2f, cv::Point2f);
    std::vector<cv::Point> _getBlob(const cv::Mat&);
    //bool _is_circle(std::vector< cv::Point >);
    
    // check if part is of a certain kind
    bool _is_type(std::string, std::string);
    void _handleMatch(const std::string, MatchParams*, double);
};

ObjectClassifier::ObjectClassifier(std::string path) {
    std::cout<<"OpenCV Version: "<<CV_VERSION<<"\n";
    std::cout<<"--------------------------\n";
    _min_hessian = 300;
    _db_images.clear();
    _valid_path = false;
    _num_types = 0;
    _set_valid_types();
    _check_path(path);
    _load_images();
    _train_matcher();
}

ObjectClassifier::ObjectClassifier(std::string path, int min_hessian) {
    std::cout<<"OpenCV Version: "<<CV_VERSION<<"\n";
    std::cout<<"--------------------------\n";
    _min_hessian = min_hessian;
    _db_images.clear();
    _valid_path = false;
    _num_types = 0;
    _set_valid_types();
    _check_path(path);
    _load_images();
    _train_matcher();
}

ObjectClassifier::ObjectClassifier(std::string path, bool load) {
    _min_hessian = 300;
    _db_images.clear();
    _valid_path = false;
    _num_types = 0;
    _set_valid_types();
    _check_path(path);
    if (load){
        _load_images();
        _train_matcher();
    }
}

void ObjectClassifier::_set_valid_types() {
    // image types to load
    _valid_types.push_back("jpg");
    _valid_types.push_back("jpeg");
    _valid_types.push_back("png");
}

void ObjectClassifier::_check_path(std::string& path) {
    // ensure path provides a valid directory
    unsigned tilda = path.find("~");
    if (tilda != std::string::npos) {
        ROS_ERROR("Object Classifier needs to be passed a complete path. Can not include [~]");
        return;
    }
    unsigned loc = path.find_last_of(".");
    if (loc != std::string::npos) {
        char prev = path[loc-1];
        if (prev != '\\' || prev != '/') {
            // path provided is a file rather than a directory
            unsigned dir = path.find_last_of("/");
            if (dir != std::string::npos) {
                if (dir < loc)
                    path.erase(dir, path.length()-dir);
                ROS_WARN("You have provided the ObjectClassifier with a file path rather than a location. Image database will be built from the folder containing the file");
            }
        }
    }
    _db_dir = opendir(path.c_str());
    if (_db_dir == NULL) {
        ROS_ERROR("The directory [%s] cannot be accessed", path.c_str());
        return;
    }
    _valid_path = true;
    _dir_path = path;
    if (_dir_path.find_last_of("/") == _dir_path.size()-1)
        _dir_path = _dir_path.substr(0,_dir_path.size()-1);
    
    ROS_INFO("Will search path [%s] for images", _dir_path.c_str());
}

void ObjectClassifier::_load_images() {
    /* 
     * This should become a public function. 
     * This clears the db's and
     * calls _load_images_recurse on _dir_path
     * 
     * ********************************************************************/
    _db_images.clear();
    _db_names.clear();
    _load_images_recurse(_dir_path);
    std::cout<<"Image Database Size: "<<_db_images.size()<<"\n\n";
    cv::destroyWindow("loading image");
}

void ObjectClassifier::_load_images_recurse(std::string path) {
    // recursively move through folder structure to build image database
    if (!_valid_path || (path.find_last_of(".") != std::string::npos))
        return;
        
    cv::Mat temp;
    dirent* dp;
    unsigned file_type_loc;
    bool _is_home(true);
    DIR* dir;
    std::string file_name;
    std::string extension;
    std::vector<std::string>::iterator it;
    std::string folder_name("");
    int iter(0);
    
    // trim any trailing /'s
    file_type_loc = path.find_last_of("/");
    if (file_type_loc == path.size()-1)
        path = path.substr(0, file_type_loc);
        
    if (path != _dir_path){
        // if the current path is not the main path for the directory
        // add a new image type
        file_type_loc = path.find_last_of("/");
        // this check is probably unnecessary -- already trimmed trailing /'s
        if (file_type_loc == path.size()-1) {
            file_type_loc = path.find_last_of("/", path.size()-2);
        }
        // get folder name, add to list of image types
        folder_name = path.substr(file_type_loc+1);
        _db_image_types.push_back(folder_name);
        _is_home = false;
    }
    
    // open directory, read while it still has any contents
    dir = opendir(path.c_str());
    while ((dp = readdir(dir)) != NULL) {
        file_name = dp->d_name;
        if(_is_home) {
            // if in home directory, recursively load files from
            // each folder -- don't load files from home directory
            file_type_loc = file_name.find_last_of(".");
            if (file_type_loc != std::string::npos)
                continue;
            // add '/' to end of path if necessary
            //   should always be necessary...
            if (path.find_last_of("/") != path.size()-1)
                path += '/';
                
            // don't load images whose name starts with '_'
            if (file_name[0] != '_') {
                 _num_types++;
                _load_images_recurse(path+file_name+"/");
            }
        }
        else{
            // if not in home directory, load all images
            // of correct file types
            
            // get current file type
            file_type_loc = file_name.find_last_of(".");
            if (file_type_loc == std::string::npos)
                continue;
            extension = file_name.substr(file_type_loc);
            // check that file type is valid
            it = std::find(_valid_types.begin(), _valid_types.end(), extension.substr(1));
            if (it == _valid_types.end())
                continue;
            
            // open image at the path, add to database
            temp = cv::imread(path+"/"+file_name);
            if(temp.size() == cv::Size(0,0))
                continue;
            std::stringstream ss;
            ss<<folder_name<<"_"<<iter;
            iter++;
            _db_images.push_back(temp);
            _db_names.push_back(ss.str());
        }
    }
    if (!_is_home)
        std::cout <<"Loaded "<< iter <<" images of type "<< folder_name <<"\n";
    else
        std::cout<<"Loaded "<<_db_image_types.size()<<" types\n";
}

void ObjectClassifier::scene(const cv::Mat& scene) {
    // sets a new image to match against
    // extracts descriptors, shows view
    _scene = scene;
    cv::SiftFeatureDetector _detector(_min_hessian);
    cv::SiftDescriptorExtractor _extractor;
    _detector.detect(_scene, _query_keypoints);
    _extractor.compute(_scene, _query_keypoints, _query_descriptors);
    
    cv::imshow("Classifier View",_scene);
}

void ObjectClassifier::_train_matcher() {
    // trains the matcher on the image database
    
    cv::SiftFeatureDetector _detector(_min_hessian);
    cv::SiftDescriptorExtractor _extractor;
    std::cout<<"Detecting keypoints...\n";
    if(_db_images.size() == 0) {
        ROS_ERROR("Image Database is empty. Unable to train matcher");
        return;
    }
    _detector.detect(_db_images, _db_keypoints);
    
    std::cout<<"Extracting descriptors...\n";
    _extractor.compute(_db_images, _db_keypoints, _db_descriptors);
    for(int i = 0; i < _db_descriptors.size(); i++) {
        if(_db_descriptors[i].empty())
            ROS_ERROR("Missing descriptors");
    }
    
    _matcher.add(_db_descriptors);
    std::cout<<"Training matcher...\n";
    _matcher.train();
    std::cout<<"Training Complete!\n";
}

std::string ObjectClassifier::getFinalDecision(ObjectModel model, std::string part) {
    // tries to make a final decision about what the object actually is
    // the model is built from geometric data gathered about the object
    // similar to a classification tree -- not really one though
    //   ideally this wouldn't be hardcoded...
    std::string finalDecision;
    finalDecision = part;
    if ( _is_type(part, "gear") ) {
        if (model.grip_width < 20)
            finalDecision = "8_tooth_gear";
        else if (model.grip_width < 30)
            finalDecision = "12_tooth_gear";
        else if (model.grip_width < 40)
            finalDecision = "16_tooth_gear";
        else if (model.grip_width < 52)
            finalDecision = "20_tooth_gear";
        else if (model.grip_width < 70)
            finalDecision = "24_tooth_gear";
        else if ( !_is_type(part, "40_tooth") ) {
            if (model.grip_width > 70)
                finalDecision = "36_tooth_gear";
        }
    }
    else if ( _is_type(part, "tire") || _is_type(part, "wheel") )
        finalDecision = part;
    else if ( (model.grip_width < 12) && !_is_type(part, "beam") ) {
        // very skinny parts
        if (model.length < 4) {
            if (model.area < 20)
                finalDecision = "2_axle";
            else if (model.aspect_ratio < 0.35)
                finalDecision = "peg";
            else
                finalDecision = "2_1_plate";
        }
        else if (model.length < 6) {
            if (model.area_ratio > 0.76)
                finalDecision = "3_axle";
            else if (model.aspect_ratio < 0.23)
                finalDecision = "extended_peg";
            else
                finalDecision = "extended_axle_peg";
        }
        else {
            // part is an axle -- need to choose correct one based on length
            if (model.length < 7)
                finalDecision = "4_axle";
            else if (model.length < 9)
                finalDecision = "5_axle";
            else if (model.length < 11.2)
                finalDecision = "6_axle";
            else if (model.length < 14.5)
                finalDecision = "8_axle";
            else if (model.length < 18)
                finalDecision = "10_axle";
            else
                finalDecision = "12_axle"; 
        }
    }
    else if ( _is_type(part, "beam") && model.area_ratio < .7 && !model.convex ) {
        // bent parts
        if (model.area < 150)
            finalDecision = "2_4_beam";
        else if (model.area < 195)
            finalDecision = "3_5_beam";
        else if (model.area < 225)
            finalDecision = "4_6_beam";
        else
            finalDecision = "3_4_7_beam";
    }
    else if ( _is_type(part, "peg_brick") || _is_type(part, "axle_brick") ||
        ( !_is_type(part, "beam") && !model.convex && fabs(model.grip_width-22) < 3 ) ) {
        if (model.length < 4.25) {
            finalDecision = _is_type(part, "axle_brick") ? "2_1_axle_brick" : "2_1_peg_brick";
            if (model.internal_area - model.area > 3)
                finalDecision = "2_1_brick";
            else if (model.area - model.internal_area < 0)
                finalDecision = "2_1_peg_brick";
                
        }
        else if (model.length < 8)
            finalDecision = "4_1_peg_brick";
        else if (model.length < 12)
            finalDecision = "6_1_peg_brick";
        else if (model.length < 15)
            finalDecision = "8_1_peg_brick";
        else
            finalDecision = "16_1_peg_brick";
    }
    else if ( _is_type(part, "plate") ) {
        if (model.grip_width > 30) {
            // is two wide
            if (model.length < 8)
                finalDecision = "4_2_plate";
            else if (model.length < 12)
                finalDecision = "6_2_plate";
            else
                finalDecision = "8_2_plate";
        }
        else if (model.grip_width < 20) {
            if (model.length < 6)
                finalDecision = "2_1_plate";
            else
                finalDecision = "4_1_plate";
        }
    } 
    else if (( _is_type(part, "beam") && model.area_ratio > .7) ||
        ( (model.shape == RECTANGLE  || model.area_ratio > .7 || model.length > 18 )
        && ( model.grip_width > 12 && model.grip_width < 20) ) ) {
        if ( !_is_type(part, "beam") && model.length < 4.5 && model.convex)
            finalDecision = "2_1_plate";
        else if (model.length < 6)
            finalDecision = "3_beam";
        else if (model.length < 7.5 && model.convex)
            finalDecision = "4_1_plate";
        else if (model.length < 10)
            finalDecision = "5_beam";
        else if (model.length < 13)
            finalDecision = "7_beam";
        else if (model.length < 17)
            finalDecision = "9_beam";
        else if (model.length < 20)
            finalDecision = "11_beam";
        else
            finalDecision = "15_beam";  // 
    }
    
    else
        finalDecision = "4512363";
    
    return finalDecision;
}

std::string ObjectClassifier::match(const cv::Mat& scene) {
    // tries to match provided image to the database
    this->scene(scene);
    std::string resp = this->match();
    return resp;
}

std::string ObjectClassifier::match() {
    // matches this object's _scene image against the database
    int max = 0;
    int maxIndex;
    if (_query_descriptors.empty())
        ROS_ERROR("Unable to find any descriptors in the image");
    else{
        // match
        _matcher.match(_query_descriptors, _matches);
        std::vector<cv::DMatch> good_matches; 
        cv::Mat match_img;
        std::vector<uint> _num_matches(_db_image_types.size(), 0);
        std::stringstream window("match");
        
        // find min, max dists for all matches
        double max_dist = 0; double min_dist = 100;
        for(int i = 0; i < _matches.size(); i++) {
            double dist = _matches[i].distance;
            if (dist < min_dist) min_dist = dist;
            if (dist > max_dist) max_dist = dist;
        }
        // only keep some matches -- the good ones
        for(int i = 0; i < _matches.size(); i++)
        {
            if(_matches[i].distance < 3*min_dist)
                good_matches.push_back(_matches[i]);
        }
        std::string type;
        std::cout<<"Total number of matches: "<<good_matches.size()<<"\n";
        std::vector<std::string>::iterator it;
        int ind;
        // get counts on the good matches by image type
        for(int i = 0; i < good_matches.size(); i++) {
            // get the database index of this match
            ind = good_matches[i].imgIdx;
            type = "";
            // get the image name from the database index
            if (ind < _db_names.size())
                type = _db_names[ind];
            // all images are named "type_iter" where iter is a number
            if (type != "") {
                type = type.substr(0, type.find_last_of("_"));
                int j;
                // iterate through directory image types to get index
                for(j = 0; j < _db_image_types.size(); j++) {
                    if (_db_image_types[j] == type)
                        break;
                }
                // increment number of matches for that image type
                if (j < _num_matches.size())
                    _num_matches[j]++;
            }
        }
        // find the image type with the most matches
        std::cout<<"\nNumber of matches";
        maxIndex = _num_matches.size();
        for(int i = 0; i < _num_matches.size(); i++) {
            if (_num_matches[i] > 0)
                std::cout<<"\n\t"<<_db_image_types[i]<<": "<<_num_matches[i];
            if(_num_matches[i] > max) {
                max = _num_matches[i];
                maxIndex = i;
            }
        }
    }
    // return name of image with the most matches, if there are at least a few
    if (max > 2){
        std::cout<<"\nI think this piece matches with "<<_db_image_types[maxIndex]<<"\n";
        return _db_image_types[maxIndex];
    }
    else{
        std::cout<<"I'm not sure what I'm seeing in this picture right now...\n";
        return "";  
    }
    
}

void ObjectClassifier::save_scene() {
    // save the current _scene to the database
    _save_image();
}

void ObjectClassifier::save_scene(std::string name, const cv::Mat& img) {
    // save the provided image to the database under the given name
    
    // set scene
    this->scene(img);
    // save scene
    this->save_scene(name);
}

void ObjectClassifier::save_scene(std::string resp) {
    // save the current scene at the provided name response
    bool invalid = true;
    std::string adjusted;
    std::size_t found;
    unsigned file_type_loc;
    std::string file_name;
    std::string extension;
    std::string valid_chars("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_.");
    // check that file name is valid
    while(invalid && ros::ok()) {
        // see if there are any invalid characters
        found = resp.find_first_not_of(valid_chars);
        if (found != std::string::npos) {
            std::cout<<"There is an invalid character in your response.";
            std::cout<<" Acceptable characters are [A-Za-z0-9_]\n";
            std::cout<<"Please enter a new file name:\n";
        }
        else {
            // Get file extension, remove from file name
            // it's jpg if not specified
            file_type_loc = resp.find_last_of(".");
            file_name = resp;
            if (file_type_loc == std::string::npos) {
                extension = ".jpg";
                file_name.substr(0, file_type_loc-1);
            }
            else {
                extension = file_name.substr(file_type_loc);
            }
            // set invalid flag
            invalid = false;
        }
    }
    
    bool success;
    std::stringstream ss;
    ss << _dir_path << "/" << file_name;
    // get number of files at that path already
    int num_files = _get_num_files(ss.str());
    if (num_files == 0) {
        // if there is nothing there, create the folder
        std::string createFolderCommand = "mkdir " + ss.str();
        system(createFolderCommand.c_str());
    }    
    
    try{
        // add file to database
        // files saved as path/name_int.jpg
        ss <<"/"<< num_files << extension;
        cv::imwrite(ss.str(), _scene);
    }
    catch(std::runtime_error& ex) {
        fprintf(stderr, "Exception converting image to %s format: %s\n", extension.substr(1).c_str(), ex.what());
        return;
    }
    std::cout<<"File saved as "<<ss.str()<<"!\n";
}


void ObjectClassifier::reload() {
    // reloads the classifier, rebuilds everything
    _db_images.clear();
    _db_names.clear();
    _db_image_types.clear();
    _num_types = 0;
    _db_keypoints.clear();
    _query_keypoints.clear();
    _db_descriptors.clear();
    _load_images();
    _train_matcher();
}

void ObjectClassifier::_save_image(){
    // saves the current scene, requests user input for name
    cv::imshow("Current Scene",_scene);
    cv::waitKey(500);
    std::cout<<"Would you like to save this picture as a template?  [y/n]\n";
    std::string response = "BLANK";
    std::cin >> response;
    std::transform(response.begin(), response.end(), response.begin(), tolower);
    if (response != "y" && response != "yes")
        return;
    else
        _get_name_for_image();
}

void ObjectClassifier::_get_name_for_image() {
    // request user input in order to name image
    // very similar to save_scene() except it queries the user directly
    bool invalid = true;
    std::string resp;
    std::string adjusted;
    std::size_t found;
    unsigned file_type_loc;
    std::string file_name;
    std::string extension;
    std::string valid_chars("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_.");
    std::cout<<"What type is this?\n";
    while(invalid && ros::ok()) {
        std::cin >> resp;
        found = resp.find_first_not_of(valid_chars);
        if (found != std::string::npos) {
            std::cout<<"There is an invalid character in your response.";
            std::cout<<" Acceptable characters are [A-Za-z0-9_]\n";
            std::cout<<"Please enter a new file name:\n";
        }
        else {
            file_type_loc = resp.find_last_of(".");
            file_name = resp;
            if (file_type_loc == std::string::npos)
            {
                extension = ".jpg";
                file_name.substr(0, file_type_loc-1);
            }
            else
            {
                extension = file_name.substr(file_type_loc);
            }
            //file_name +=extension;
            std::cout<<"trying to name file "<<file_name<<extension<<"\n";
            invalid = false;
        }
    }
    bool success;
    
    std::stringstream ss;
    ss << _dir_path << "/" << file_name;
    std::cout<<"getting files at path "<<ss.str()<<"\n";
    int num_files = _get_num_files(ss.str());
    if (num_files == 0) {
        std::string createFolderCommand = "mkdir " + ss.str();
        system(createFolderCommand.c_str());
    }    
    std::cout<<"There are currently "<<num_files<<" of that type\n";
    try{
        ss <<"/"<< num_files << extension;
        //std::cout<<"saving file as " << ss.str()<<"\n";
        cv::imwrite(ss.str(), _scene);
    }
    catch(std::runtime_error& ex) {
        fprintf(stderr, "Exception convertion image to %s format: %s\n", extension.substr(1).c_str(), ex.what());
        return;
    }
    std::cout<<"File saved as "<<ss.str()<<"!\n";
    
}

int ObjectClassifier::_get_num_files(std::string path) {
    // get the number of files at the specified path
    
    // open the directory
    DIR* dir;
    dir = opendir(path.c_str());
    dirent* dp;
    unsigned file_type_loc;
    std::string file_name;
    std::string extension;
    std::vector<std::string>::iterator it;
    // if the directory doesn't exist, return 0
    if (dir == NULL)
        return 0;
    
    int num_files = 0;
    // while there are still files in the directory
    while ((dp = readdir(dir)) != NULL) {
        // get file name and extension
        file_name = dp->d_name;
        file_type_loc = file_name.find_last_of(".");
        if (file_type_loc == std::string::npos)
            continue;
        extension = file_name.substr(file_type_loc);
        // if this is a valid image type, increment the number of files
        it = std::find(_valid_types.begin(), _valid_types.end(), extension.substr(1));
        if (it == _valid_types.end())
            continue;
        num_files++;
    }
    return num_files;
}

std::vector<std::string> ObjectClassifier::_get_files_names() {
    // DEPRECATED??
    std::vector<std::string> names;
    if (!_valid_path)
        return names;
    dirent* dp;
    unsigned file_type_loc;
    std::string file_name;
    std::string extension;
    std::vector<std::string>::iterator it;
    std::cout<<"searching directory "<<_dir_path<<"\n";
    _db_dir = opendir(_dir_path.c_str());
    while ((dp = readdir(_db_dir)) != NULL)
    {
        file_name = dp->d_name;
        file_type_loc = file_name.find_last_of(".");
        if (file_type_loc != std::string::npos)
            continue;
        names.push_back(file_name);
    }
    return names;
}

bool ObjectClassifier::_is_type(std::string part, std::string type) {
    /*
     * checks if the part is of the specified type
     *  e.g part is "3_axle"
     *     _is_type("3_axle", "axle") is true
     *     _is_type("3_axle", "beam") is false
     */
    std::size_t found = part.find(type);
    if ( found != std::string::npos)
        return true;
    return false;
}

/* EVERYTHING AFTER THIS IS A LOT OF EXPERIMENTING
***************************************************/

std::vector<cv::Point> ObjectClassifier::_getBlob(const cv::Mat& scene) {
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
    int type = cv::MORPH_ELLIPSE;
    int sz = 1;
    cv::Mat element = getStructuringElement( type,
                                             cv::Size(2*sz+1, 2*sz+1),
                                             cv::Point(sz, sz) );
    cv::erode(thresh, thresh, element); 
    //element = getStructuringElement( type, (3,3) );
    cv::morphologyEx(thresh, thresh, cv::MORPH_CLOSE, cv::Mat() );
    cv::morphologyEx(thresh, thresh, cv::MORPH_CLOSE, cv::Mat() );
    
    std::vector< std::vector< cv::Point > > contours;
    std::vector< cv::Vec4i > hierarchy;
    int largest_area = 0;
    int largest_contour_index = 0;
    
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
    //cv::imshow("blob", thresh);
    double min_area = 75;
    if (largest_area < min_area)
    {
        return temp;
    }
    
    //std::vector<cv::Point> hull;
    //convexHull(contours[largest_contour_index], hull, false, true);
    //return hull;
    return contours[largest_contour_index];
}

double ObjectClassifier::angle( cv::Point pt1, cv::Point pt2, cv::Point pt0, uint units=0 )
{
    double a = _dist(pt1, pt2);
    double b = _dist(pt1, pt0);
    double c = _dist(pt2, pt0);
    // a2 = b2 + c2 - 2bc*cos(theta)
    return acos((b*b + c*c - a*a)/(2*b*c));
}

bool pointsOppositeLine(Line line, cv::Point a, cv::Point b) {
    // returns true if the points a and b are on opposite sides of line 
    // from "How to determine if 2 points are on opposite sides of a lines"
    //     on math.stackexchange.com
    cv::Point pt1, pt2;
    pt1 = line.pt1;
    pt2 = line.pt2;
    int first = (pt1.y-pt2.y)*(a.x-pt1.x) + (pt2.x-pt1.x)*(a.y-pt1.y);
    int second = (pt1.y-pt2.y)*(b.x-pt1.x) + (pt2.x-pt1.x)*(b.y-pt1.y);
    return ( first * second < 0 );
}

bool pointInBlob(cv::Point pt, std::vector<cv::Point> blob) {
    std::vector<cv::Point>::iterator it;
    it = std::find(blob.begin(), blob.end(), pt);
    if (it == blob.end())
        return false;
    return true;    
}

bool ObjectClassifier::intersect(Line a, Line b) {
    // return true if distinct line segments a and b intersect along their length
    
    // check that two lines don't share a common endpoint
    if (a.pt1 == b.pt1 || a.pt1 == b.pt2 || a.pt2 == b.pt1 || a.pt2 == b.pt2)
        return false;
    // else if endpoints of one line are on opposite sides
    // of the other line, check for an intersection
    if ( pointsOppositeLine(a, b.pt1, b.pt2) ) {
        // Solve line equations y-y1 = m(x-x1) for intersection point
        float y, x, m1, m2;
        cv::Point2f pt1, pt2, pt3, pt4;
        pt1 = a.pt1;
        pt2 = a.pt2;
        pt3 = b.pt1;
        pt4 = b.pt2;
        // get slopes of each line
        m1 = (pt1.y - pt2.y) / (pt1.x - pt2.x);
        m2 = (pt3.y - pt4.y) / (pt3.x - pt4.x);
        // if lines are parallel, they don't intersect or are the same line
        // this should never throw due to pointOppositeLine check
        if (m1 == m2)
            return false;
        
        // from setting y-y1=m(x-x1) equal for each line
        x = (m1*pt1.x - pt1.y - m2*pt3.x + pt3.y) / (m1 - m2);
        y = m1*(x-pt1.x) + pt1.y;
        
        // check that x, y are between the endpoints of one of the lines
        double xMax = max(pt1.x, pt2.x);
        double xMin = min(pt1.x, pt2.x);
        if ( (x > xMax) || (x < xMin) )
            return false;
        double yMax = max(pt1.y, pt2.y);
        double yMin = min(pt1.y, pt2.y);
        if ( (y > yMax) || (y < yMin) )
            return false;
        return true;
    }
    // else if points are on the same side of the line
    return false;
}

bool ObjectClassifier::inBlob(Line line, std::vector< cv::Point > blob) {
    int len = blob.size();
    cv::Point pt1, pt2, a, b;
    pt1 = line.pt1;
    pt2 = line.pt2;
    // check every edge segment of blob to see if the line intersects it
    //std::cout << "line " << line.pt1 << " " << line.pt2 << "\n";
    for (int i = 1; i <= len; i++) {
        a = blob[i%len];
        b = blob[i-1];
        //std::cout << "edge " << a << " " << b << "\n";
        // check if the two points are consecutive edge points
        if ( (pt1 == a && pt2 == b) || (pt1 == b && pt2 == a) )
            return false;
        //std::cout << "not an edge\n";
        // else check for intersection
//         cv::Mat img(_scene);
//         Line temp(a,b);
//         cv::line(img, temp.pt1, temp.pt2, cv::Scalar(255));
//         cv::line(img, line.pt1, line.pt2, cv::Scalar(255,255,255));
//         cv::imshow("lines", img);
//         cv::waitKey(30);
        if ( intersect(line, Line(a,b)) )
            return false;
        //std::cout << " not an intersection\n";   
    }
    //drawLine(line);
    return true;
}

std::vector< Line > ObjectClassifier::interiorLines(std::vector< cv::Point > blob) {
    std::vector< Line > chords;
    int len = blob.size();
    Line temp;
    for (int i = 0; i < len; i++) {
        // build list of interior lines between all points
        // must be separated by at least 5 points
        for (int j = i+5; j < len; j++) {
            temp = Line(blob[i], blob[j]);
            // check that line connecting points stays within object
            // boundary and that line is not an edge
            if ( inBlob(temp, blob) )
                chords.push_back(temp);
            //std::cout << "---new line\n";
        }
    }
    return chords;
}

void ObjectClassifier::drawLine(Line a, int delay) {
    cv::Mat img(_scene);
    cv::RNG rng(ros::Time::now().nsec);
    cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0, 255));
    cv::line(img, a.pt1, a.pt2, color);
    cv::imshow("lines", img);
    cv::waitKey(delay);
}

bool isUnique(cv::RotatedRect rect, std::vector<cv::RotatedRect> rects) {
    // checks that the new rectangle is unique
    // unique implies that at most 1 endpoint from each rectangle is within
    
}

cv::RotatedRect fromDiagonals(std::vector<Line> sides) {
    // return a rectangle from the sides
    if (sides.size() != 4)
        return cv::RotatedRect(cv::Point(0,0), cv::Size(0,0), 0);
    // else
    double maxLen, minLen, len;
    int maxInd, minInd;
    maxLen = sides[0].length();
    minLen = sides[0].length();
    cv::Point2f center(0,0);
    for (int i = 0; i < 4; i++) {
        len = sides[i].length();
        if (len < minLen) {
            minLen = len;
            minInd = i;
        }
        if (len > maxLen) {
            maxLen = len;
            maxInd = i;
        }
        center.x += sides[i].pt1.x;
        center.y += sides[i].pt1.y;
    }
    // call height maxlen, width minLen
    cv::Size2f size(minLen, maxLen);
    // center is the average position of the corners
    center.x /= 4;
    center.y /= 4;
    // angle is the angle between the length of the rectangle and the vertical
    float angle = sides[maxInd].angle(Line(cv::Point(0,0),cv::Point(0,1)));
    return cv::RotatedRect(center, size, angle);
}

std::vector< cv::RotatedRect > ObjectClassifier::rectanglesFromDiagonals(std::vector< Line > lines) {
    // go through all sets of lines
    int len = lines.size();
    std::vector<cv::RotatedRect> rects;
    std::vector<Line> sides(4, Line());
    double angle;
    bool isRect;
    
    for (int i = 0; i < len; i++) {
        for (int j = i+1; j < len; j++) {
            // check that these two lines intersect
            cv::Mat img(_scene);
            cv::RNG rng(ros::Time::now().nsec);
            cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0, 255));
            //line(img, lines[i].pt1, lines[i].pt2, color);
            //line(img, lines[j].pt1, lines[j].pt2, color);
            //cv::imshow("lines", img);
            //cv::waitKey(1000);
            if ( intersect(lines[i], lines[j]) ) {
                // form four sides from lines 
                sides[0] = Line(lines[i].pt1, lines[j].pt1);
                sides[1] = Line(lines[j].pt1, lines[i].pt2);
                sides[2] = Line(lines[i].pt2, lines[j].pt2);
                sides[3] = Line(lines[j].pt2, lines[i].pt1);
                isRect = true;
                //std::cout <<"\n new rectangle\n";
                for (int k = 1; k <= 4; k++) {
                    // get angle between successive sides
                    angle = sides[k%4].angle(sides[k-1], 1);
                    //std::cout << " angle " << angle;
                    // all angles should be within 10 degrees of perpendicular
                    if (isnan(angle)) 
                        isRect = false;
                    if (fabs(angle-90) > 5)
                        isRect = false;
                }
                if (isRect) {
                    
                    color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0, 255));
                    for (int i = 0; i < 4; i++)                    
                        line(img, sides[i].pt1, sides[i].pt2, color);
                    cv::RotatedRect rect = fromDiagonals(sides);
                    cv::imshow("rectangles", img);
                    cv::waitKey(1000);
                    rects.push_back( rect );
                    std::cout << "\n--above is a rectangle\n";
                }
            }
        }
    }
    std::cout << "\nI found " << rects.size() << "rectangles\n";
    return rects;
}

std::vector< cv::RotatedRect > ObjectClassifier::rectangles(std::vector< cv::Point > blob) {
    // returns vector of rectangles found in blob
    std::vector< Line > chords;
    std::cout << "-- Points in Blob -- \n";
    for (int i = 0; i < blob.size(); i++)
        std::cout << blob[i] << "\n";
    std::cout << "-- End of Blob --\n";
    chords = interiorLines(blob);
    
    std::cout << "num interior lines " << chords.size() << "\n";
    double minLen = chords[0].length();
    double maxLen = minLen;
    double len;
    for (int i = 1; i < chords.size(); i++) {
        len = chords[i].length();
        std::cout << len << "\n";
        if (len < minLen)
            minLen = len;
        else if (len > maxLen)
            maxLen = len;
    }
    std::cout << " min len " << minLen << "\n";
    std::cout << " max len " << maxLen << "\n";
    std::vector<Line> goodLines;
    double thresh = sqrt(maxLen/minLen); // 5*minLen
    for (int i = 0; i < chords.size(); i++) {
        if (chords[i].length() > thresh) {
            goodLines.push_back(chords[i]);
            //drawLine(chords[i], 100);
        }
    }
    std::cout << "num good interior lines " << goodLines.size() << "\n";
    std::vector<cv::RotatedRect> rects = rectanglesFromDiagonals(goodLines);
    
    return rects;
}

std::vector<int> quickSort(const std::vector<int>& arr) {
    // sort array of distinct indices
    int len = arr.size();
    if (len < 2)
        return arr;
    // else
    std::vector<int> a, b;
    int p = arr[0];
    for (int i = 1; i < len; i++) {
        if (arr[i] < p)
            a.push_back(arr[i]);
        else if (arr[i] > p)
            b.push_back(arr[i]);
    }
    // sort sub arrays
    a = quickSort(a);
    b = quickSort(b);
    // combine arrays
    a.push_back(p);
    a.insert(a.end(), b.begin(), b.end());
    
    return a;
}

std::vector<cv::Point> ObjectClassifier::corners(const std::vector<cv::Point> blob) {
    // get points near corners from blob
    std::vector<int> hull;
    std::vector<cv::Point> corners;
    convexHull(blob, hull, false, true);
    hull = quickSort(hull);
    int hullLen = hull.size();
    int blobLen = blob.size();
    std::cout << "blobLen " << blobLen;
    std::cout << "\nhullLen " << hullLen <<"\n";
    int thresh = 10;
    if (blobLen > 100)
        thresh = sqrt(blobLen);
    std::cout << "threshold " << thresh << "\n";
    int index;
    int lastIndex = 0;
    int diff;
    cv::Mat img(_scene);
    cv::Scalar hullColor(255,0,0);
    cv::Scalar pointColor(0, 255, 0);
    // go through convex hull -- look for 
    for(int i = 0; i < hullLen; i++) {
        index = hull[i];
        corners.push_back(blob[index]);
        //circle(img, blob[index], 1, hullColor);
        std::cout << "index " << index << "\n";
        // if this index is far from last index, add some points
        diff = index - lastIndex;
        if ( abs(diff) > thresh ) {
            for (int j = 1; j <= abs(diff)/thresh+1; j++) {
                corners.push_back( blob[(lastIndex+(thresh*j)/3)%blobLen] );
                corners.push_back( blob[(lastIndex+(thresh*j)/2)%blobLen] );
                corners.push_back( blob[(lastIndex+2*thresh*j/3)%blobLen] );
                //circle(img, blob[(lastIndex+thresh*j/3)%blobLen], 1, pointColor);
                //circle(img, blob[(lastIndex+thresh*j/2)%blobLen], 1, pointColor);
                //circle(img, blob[(lastIndex+2*thresh*j/3)%blobLen], 1, pointColor);
            }
        }
        //cv::imshow("points", img);
        //cv::waitKey(800);
        lastIndex = index;
    }
    
    // remove clustered points
    int len = corners.size();
    for(int i = 0; i < corners.size(); i++) {
        int j = i+1;
        while (_dist(corners[i], corners[j]) < 5)
            j++;
        if (j >= len)
            j = len-1;
        diff = j - i;
        if (diff > 1) {
            for (int k = i+1; k < j; k++) {
                corners.erase( corners.begin() + k );
            }
        }
    }
    
    return corners;
}

float ObjectClassifier::aspect_ratio(std::vector< cv::Point > blob) {
    cv::RotatedRect rect = cv::minAreaRect(blob);
    return min (rect.size.width/rect.size.height, rect.size.height/rect.size.width );
}

float ObjectClassifier::aspect_ratio() {
    std::vector< cv::Point > blob = _getBlob(_scene);
    return aspect_ratio(blob);
}

bool ObjectClassifier::is_rectangular(std::vector< cv::Point > blob) {
    ObjectModel temp;
    return is_rectangular(blob, temp);
}

bool ObjectClassifier::is_rectangular(std::vector< cv::Point > blob, ObjectModel& model) {
    //std::cout << blob << "\n";
    
    std::vector< cv::Point > approx;
    approxPolyDP(cv::Mat(blob), approx, cv::arcLength(cv::Mat(blob), true)*0.015, true);
    bool isIt = false;
    
    model.angles.clear();
    model.minimalBlob = approx;
    model.convex = cv::isContourConvex(approx);
    
    std::cout << "Convex " << (model.convex ? "true" : "false") << "\n";
    
    int len = approx.size();
//     cv::Mat img(_scene);
//     for (int i = 0; i < len; i++) {
//         circle(img, approx[i], 1, cv::Scalar(255));
//     }
//     cv::imshow("approx points", img);
//     cv::waitKey(1000);
    std::cout << "blob size " << len << "\n";
    
    double maxCosine = 0;
    for (int j = 2; j <= len; j++) {
        double cosine = fabs(angle(approx[j%len], approx[j-2], approx[j-1]));
        model.angles.push_back(cosine);
        maxCosine = max(maxCosine, cosine);
    }
    if (len == 2)
        model.shape = LINE;
    else if (len == 3)
        model.shape = TRIANGLE;
    else if (len == 4)
        model.shape = RECTANGLE;
    else if (len <= 6) {
        if (model.convex)
            model.shape = RECTANGLE;
    }
    else if (len > 6){
        // it's probably a circle, but I can recognize all circular shapes
        //   gears, wheels, etc
        // so don't worry about it
    }
    else
        model.shape = UNKNOWN;
    // if minimum bounding rectangle area is the same as the blob area, it's a rectangle
    // closest shape to satisfy this but not be rectangle is a circle
    // circle takes up 78.5% of the area of it's bounding rectangle
    /*
    if (ratio > 0.85){
        if (model.convex)
            model.shape = RECTANGLE;
    }*/
    
    
    if (fabs(maxCosine-PI/2) < 0.3)
        isIt = true;
    std::cout << "max cosine " << maxCosine << "\n";
        
    std::cout << (isIt ? "IS rectangular\n" : "is NOT rectangular\n");
    return isIt;
}

bool ObjectClassifier::is_rectangular(const cv::Mat &img) {
    std::vector< cv::Point > blob = _getBlob(img);
    return is_rectangular(blob);
}

bool ObjectClassifier::is_rectangular() {
    // tries to verify that the object in scene is rectangular
    return is_rectangular(_scene);
}

bool ObjectClassifier::is_circle(std::vector< cv::Point > blob) {
    int size = blob.size();
    // get center of blob
    int x = 0;
    int y = 0;
    for( int i = 0; i < size; i++ )
    {
        x += blob[i].x;
        y += blob[i].y;
    }
    x /= size;
    y /= size;
    cv::Point2f pt(x,y);
    // get distances from blob to center
    std::vector< double > dist(size, 0);
    for (int i = 0; i < size; i++)
    {
        dist[i] = _dist(pt, blob[i]);
    }
    // statistics on distances
    double sum = std::accumulate( dist.begin(), dist.end(), 0.0 );
    double mean = sum / size;
    
    std::vector<double> diff(size);
    std::transform( dist.begin(), dist.end(), diff.begin(),
                    std::bind2nd( std::minus<double>(), mean ) );
    double sq_sum = std::inner_product( diff.begin(), diff.end(), diff.begin(), 0.0 );
    double stdev = std::sqrt( sq_sum / (size - 1) );
    std::cout<<"mean: "<<mean<<"  stddev: "<< stdev<<"\n";
    return false;
}

double ObjectClassifier::_dist(cv::Point2f b, cv::Point2f a) {
    // get distance between 2 points
    double val = sqrt( pow((b.x - a.x), 2) + pow((b.y - a.y), 2) );
    //std::cout << "distance " << val << "\n";
    return val;
}

#endif