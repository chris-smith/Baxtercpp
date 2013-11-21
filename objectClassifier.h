#include "rosCamWrap.h"
#include <geometry_msgs/Point.h>
#include <string>
#include <dirent.h>
#include <stdlib.h>
#include <sstream>

#define OBJ_CLS_DO_NOT_LOAD false

class ObjectClassifier
{
public:
    ObjectClassifier(std::string);              // provide path to db images
    ObjectClassifier(std::string, bool);        // load images into db?
    ObjectClassifier(std::string, int);         // _min_hessian
    
    void scene(const cv::Mat&);       //set the image to search through
    void match();
    void save_scene();                // prompts user to save scene in specified directory
private:
    ObjectClassifier();
    
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
    cv::FlannBasedMatcher _flannMatcher;
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
};

ObjectClassifier::ObjectClassifier(std::string path)
{
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

ObjectClassifier::ObjectClassifier(std::string path, int min_hessian)
{
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

ObjectClassifier::ObjectClassifier(std::string path, bool load)
{
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

void ObjectClassifier::_set_valid_types()
{
    _valid_types.push_back("jpg");
    _valid_types.push_back("jpeg");
    _valid_types.push_back("png");
}

void ObjectClassifier::_check_path(std::string& path)
{
    unsigned tilda = path.find("~");
    if (tilda != std::string::npos)
    {
        ROS_ERROR("Object Classifier needs to be passed a complete path. Can not include [~]");
        return;
    }
    unsigned loc = path.find_last_of(".");
    if (loc != std::string::npos)
    {
        char prev = path[loc-1];
        if (prev != '\\' || prev != '/')
        {
            // path provided is a file rather than a directory
            unsigned dir = path.find_last_of("/");
            if (dir != std::string::npos)
            {
                if (dir < loc)
                    path.erase(dir, path.length()-dir);
                ROS_WARN("You have provided the ObjectClassifier with a file path rather than a location. Image database will be built from the folder containing the file");
            }
        }
    }
    _db_dir = opendir(path.c_str());
    if (_db_dir == NULL)
    {
        ROS_ERROR("The directory [%s] cannot be accessed", path.c_str());
        return;
    }
    _valid_path = true;
    _dir_path = path;
    if (_dir_path.find_last_of("/") == _dir_path.size()-1)
        _dir_path = _dir_path.substr(0,_dir_path.size()-1);
    
    ROS_INFO("Will search path [%s] for images", _dir_path.c_str());
}

void ObjectClassifier::_load_images()
{
    /* 
     * This should become a public function. It should clear the db's and
     * call _load_images_recurse on _dir_path
     * 
     * ********************************************************************/
    if (!_valid_path)
        return;
    cv::Mat temp;
    dirent* dp;
    unsigned file_type_loc;
    std::string file_name;
    std::string extension;
    std::vector<std::string>::iterator it;
    while ((dp = readdir(_db_dir)) != NULL)
    {
        file_name = dp->d_name;
        file_type_loc = file_name.find_last_of(".");
        if (file_type_loc == std::string::npos)
            continue;
        extension = file_name.substr(file_type_loc);
        
        it = std::find(_valid_types.begin(), _valid_types.end(), extension.substr(1));
        if (it == _valid_types.end())
            continue;
        std::cout<<"file name: "<<file_name<<"\n";
        //if (_dir_path.find_last_of("/") != _dir_path.size()-1)
        //    _dir_path += '/';
        temp = cv::imread(_dir_path+"/"+file_name);
        //cv::imshow("loading image", temp);
        //cv::waitKey(500);
        _db_images.push_back(temp);
        _db_names.push_back(file_name);
    }
    _db_images.clear();
    _db_names.clear();
    _load_images_recurse(_dir_path);
    std::cout<<"Image Database Size: "<<_db_images.size()<<"\n\n";
    cv::destroyWindow("loading image");
}

void ObjectClassifier::_load_images_recurse(std::string path)
{
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
    
    file_type_loc = path.find_last_of("/");
    if (file_type_loc == path.size()-1)
        path = path.substr(0, file_type_loc);
    if (path != _dir_path){
        file_type_loc = path.find_last_of("/");
        if (file_type_loc == path.size()-1){
            file_type_loc = path.find_last_of("/", path.size()-2);
        }
        folder_name = path.substr(file_type_loc+1);
        _db_image_types.push_back(folder_name);
        _is_home = false;
    }
    dir = opendir(path.c_str());
    while ((dp = readdir(dir)) != NULL)
    {
        file_name = dp->d_name;
        if(_is_home){
            file_type_loc = file_name.find_last_of(".");
            if (file_type_loc != std::string::npos)
                continue;
            if (path.find_last_of("/") != path.size()-1)
                path += '/';
            _num_types++;
            _load_images_recurse(path+file_name+"/");
        }
        else{
            file_type_loc = file_name.find_last_of(".");
            if (file_type_loc == std::string::npos)
                continue;
            extension = file_name.substr(file_type_loc);
            
            it = std::find(_valid_types.begin(), _valid_types.end(), extension.substr(1));
            if (it == _valid_types.end())
                continue;
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

void ObjectClassifier::scene(const cv::Mat& scene)
{
    _scene = scene;
    cv::SurfFeatureDetector _detector(_min_hessian);
    cv::SurfDescriptorExtractor _extractor;
    _detector.detect(_scene, _query_keypoints);
    _extractor.compute(_scene, _query_keypoints, _query_descriptors);
    //std::cout<<"scene descriptors: "<<_query_descriptors.size()<<"\n";
    cv::imshow("scene",_scene);
}

void ObjectClassifier::_train_matcher()
{
    cv::SurfFeatureDetector _detector(_min_hessian);
    cv::SurfDescriptorExtractor _extractor;
    std::cout<<"Detecting keypoints...\n";
    if(_db_images.size() == 0){
        ROS_ERROR("Image Database is empty. Unable to train matcher");
        return;
    }
    _detector.detect(_db_images, _db_keypoints);
    std::cout<<"Extracting descriptors...\n";
    _extractor.compute(_db_images, _db_keypoints, _db_descriptors);
    for(int i = 0; i < _db_descriptors.size(); i++)
    {
        if(_db_descriptors[i].empty())
            cvError(0, "Train Matcher","Descriptor Empty -- Check file path", __FILE__,__LINE__);
    }
    _flannMatcher.add(_db_descriptors);
    std::cout<<"Training matcher...\n";
    _flannMatcher.train();
    std::cout<<"Training Complete!\n";
}

void ObjectClassifier::match()
{
    int max = 0;
    int maxIndex;
    if (_query_descriptors.empty())
        ROS_ERROR("Unable to find any descriptors in the image");
    else{
        _flannMatcher.match(_query_descriptors, _matches);
        std::vector<cv::DMatch> good_matches; //distance should be less than .5???
        cv::Mat match_img;
        std::vector<uint> _num_matches(_db_image_types.size(), 0);
        std::stringstream window("match");
        //<< "match ";
        double max_dist = 0; double min_dist = 100;
        for(int i = 0; i < _matches.size(); i++)
        {
            double dist = _matches[i].distance;
            if (dist < min_dist) min_dist = dist;
            if (dist > max_dist) max_dist = dist;
        }
        for(int i = 0; i < _matches.size(); i++)
        {
            if(_matches[i].distance < 3*min_dist)
                good_matches.push_back(_matches[i]);
        }
        std::string type;
        std::cout<<"Total number of matches: "<<good_matches.size()<<"\n";
        std::vector<std::string>::iterator it;
        for(int i = 0; i < good_matches.size(); i++)
        {
            //std::cout<<"Match "<<i<<"\n\tqueryIdx: "<<window[i].queryIdx<<"\n\t";
            //std::cout<<"trainIdx: "<<window[i].trainIdx<<"\n\timgIdx: ";
            //std::cout<<window[i].imgIdx<<"\n\tdistance: "<<window[i].distance<<"\n";
            //if (_matches[i].distance < 0.5)
            type = _db_names[good_matches[i].imgIdx];
            type = type.substr(0, type.find_last_of("_"));
            int j;
            for(j = 0; j < _db_image_types.size(); j++)
                if (_db_image_types[j] == type)
                    break;
            //std::cout<<"match found at index "<< j << " for "<<_db_image_types[j]<<"\n";
            if (j < _num_matches.size())
                _num_matches[j]++;
            //_num_matches[good_matches[i].imgIdx]++;
            /*cv::namedWindow(window.str(), CV_WINDOW_AUTOSIZE);
            cv::drawMatches(_db_images[i], _db_keypoints[i], _scene, _query_keypoints,
                            good_matches, match_img, cv::Scalar::all(-1), cv::Scalar::all(-1),
                            std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
            cv::imshow(window.str(), match_img);
            cv::waitKey(1000);*/
        }
        std::cout<<"\nNumber of matches";
        maxIndex = _num_matches.size();
        for(int i = 0; i < _num_matches.size(); i++)
        {
            std::cout<<"\n\t"<<_db_image_types[i]<<": "<<_num_matches[i];
            if(_num_matches[i] > max)
            {
                max = _num_matches[i];
                maxIndex = i;
            }
        }
    }
    if (max > 2){
        cv::Mat temp;
        std::cout<<"\nI think this piece matches with "<<_db_image_types[maxIndex]<<"\n";
        //cv::imshow("Matching image",_db_images[maxIndex]);
        //cv::drawMatches(_db_images[maxIndex], _db_keypoints[maxIndex], _scene, _query_keypoints,
        //                   _matches, temp, cv::Scalar::all(-1), cv::Scalar::all(-1),
        //                     std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
        //     cv::imshow("Match", temp);
        //cv::waitKey(1000);
        //cv::destroyWindow("Match");
    }
    else{
        std::cout<<"I'm not sure what I'm seeing in this picture right now...\n";
        ;//_save_image();   
    }
    
}

void ObjectClassifier::save_scene()
{
    _save_image();
}

void ObjectClassifier::_save_image()
{
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

void ObjectClassifier::_get_name_for_image()
{
    bool invalid = true;
    std::string resp;
    std::string adjusted;
    std::size_t found;
    unsigned file_type_loc;
    std::string file_name;
    std::string extension;
    std::string valid_chars("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_.");
    std::cout<<"What folder should this go in?\n";
    while(invalid && ros::ok())
    {
        std::cin >> resp;
        found = resp.find_first_not_of(valid_chars);
        if (found != std::string::npos)
        {
            std::cout<<"There is an invalid character in your response.";
            std::cout<<" Acceptable characters are [A-Za-z0-9_]\n";
            std::cout<<"Please enter a new file name:\n";
        }
        else
        {
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
    //file_name = file_name.substr(0, file_type_loc);
    std::stringstream ss;
    ss << _dir_path << "/" << file_name;
    std::cout<<"getting files at path "<<ss.str()<<"\n";
    int num_files = _get_num_files(ss.str());
    if (num_files == 0)
    {
        std::string createFolderCommand = "mkdir " + ss.str();
        system(createFolderCommand.c_str());
    }    
    std::cout<<"There are currently "<<num_files<<" of that type\n";
    try{
        ss <<"/"<< num_files << extension;
        //std::cout<<"saving file as " << ss.str()<<"\n";
        cv::imwrite(ss.str(), _scene);
    }
    catch(std::runtime_error& ex)
    {
        fprintf(stderr, "Exception convertion image to %s format: %s\n", extension.substr(1).c_str(), ex.what());
        return;
    }
    std::cout<<"File saved as "<<ss.str()<<"!\n";
    
}

int ObjectClassifier::_get_num_files(std::string path)
{
    DIR* dir;
    std::cout<<"Getting number of files at path "<<path<<"\n";
    dir = opendir(path.c_str());
    dirent* dp;
    unsigned file_type_loc;
    std::string file_name;
    std::string extension;
    std::vector<std::string>::iterator it;
    if (dir == NULL)
        return 0;
    int num_files = 0;
    while ((dp = readdir(dir)) != NULL)
    {
        file_name = dp->d_name;
        std::cout<<file_name<<"\n";
        file_type_loc = file_name.find_last_of(".");
        if (file_type_loc == std::string::npos)
            continue;
        extension = file_name.substr(file_type_loc);
        
        it = std::find(_valid_types.begin(), _valid_types.end(), extension.substr(1));
        if (it == _valid_types.end())
            continue;
        num_files++;
    }
    return num_files;
}

std::vector<std::string> ObjectClassifier::_get_files_names()
{
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



