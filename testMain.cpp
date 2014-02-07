#include "searchControl.h"

void setup_pid(BaxterLimb&, BaxterLimb&);

int main(int argc, char** argv) {
    ros::init(argc, argv, "Lego_Sort_Implementation");
    //cv::initModule_nonfree(); //initialize nonfree opencv modules
    
    ros::NodeHandle nh;    
    KDL::Tree my_tree;
    std::string robot_desc_string;
    nh.param( "robot_description", robot_desc_string, std::string() );
    if (!kdl_parser::treeFromString(robot_desc_string, my_tree)){
        ROS_ERROR("Failed to construct kdl tree from string");
        if (!kdl_parser::treeFromFile("~/git/sdk-examples/baxter/baxter_description/urdf/baxter.urdf", my_tree));
        {
            ROS_ERROR("Failed to construct kdl tree from file");
            ROS_ERROR("Unable to construct kdl tree.\nProgram will now exit.");
            return 0;
        }
    }
    BaxterCamera left_cam("left_hand_camera");
    BaxterCamera right_cam("right_hand_camera");
    left_cam.display();
    right_cam.display();
    ros::spinOnce();
//     BaxterLimb left_arm("left");
//     BaxterLimb right_arm("right");
    BaxterLimb left_arm(my_tree, "left", 1);
    BaxterLimb right_arm(my_tree, "right",1);
    left_arm.set_command_timeout(0.1);
    right_arm.set_command_timeout(0.1);
    
    //BaxterGripper right_gripper("right");
    //BaxterGripper left_gripper("left");
    setup_pid(left_arm, right_arm);
    //right_arm.set_simple_positions( gv::RPYPose(right_arm.endpoint_pose()).position, 0 );
    //return 0;
    
    //  Low Level Publisher Test
    ros::Publisher _pub_joint_cmd;
    std::string topic = "/robot/limb/left";
    _pub_joint_cmd = nh.advertise<baxter_core_msgs::JointCommand>(topic+"/joint_command", 10);
    baxter_core_msgs::JointCommand msg;
    JointPositions position;
    position.names.push_back("left_s1");
    position.angles.push_back(-PI/8);
    msg.mode = baxter_core_msgs::JointCommand::POSITION_MODE;
    msg.names = position.names;
    msg.command = position.angles;
    ros::Duration timeout(1);
    ros::Time start = ros::Time::now();
    ros::spinOnce();
    /*timeout = ros::Duration(10);
    right_arm.set_simple_positions(right_arm.endpoint_pose().position, 0, timeout);
    JointPositions calcPos = right_arm.get_simple_positions(right_arm.endpoint_pose().position, 0);
    calcPos.print("Calculated Joint Angles");
    right_arm.joint_positions().print("Actual joint angles");
    return 1;
    /*right_gripper.open();
    right_gripper.go_to(50);
    left_gripper.open();
    timeout.sleep();
    right_gripper.open();
    left_gripper.go_to(50);
    timeout.sleep();
    right_gripper.close();
    left_gripper.close();
    /*while(ros::Time::now() - start < timeout && ros::ok())
        _pub_joint_cmd.publish(msg);
    //ros::Duration timeout(5);
    timeout = ros::Duration(5);
    JointPositions jp = left_arm.joint_positions();
    jp.print();
    gv::Point pt(.6, .5, .1);
    gv::RPYPose endpt;
    pt.print("Moving to position");
    
    left_arm.set_simple_positions(pt, 0, timeout);
    pt -= .05;
    left_arm.set_simple_positions(pt, 0, timeout);
    ros::spinOnce();
    endpt = left_arm.endpoint_pose();
    endpt.print("left final position");
    jp = right_arm.joint_positions();
    jp.print("right arm");
    endpt = right_arm.endpoint_pose();
    endpt.print("right endpoint");*/
    SearchControl search_control(&right_arm, &left_arm, &right_cam, &left_cam);
    search_control.geometry.table_height = -.072;
    search_control.geometry.height_offset = .12;
    search_control.geometry.home = right_arm.endpoint_pose();
    while( ros::ok() )
        search_control.search();
    return 0;
}

void setup_pid(BaxterLimb& left, BaxterLimb& right)
{
    left.set_joint_pid("left_w2", (Gains){3.25, 0.2, 0.004}); //6
    left.set_joint_pid("left_w1", (Gains){2.75, 0.08, 0.002});
    left.set_joint_pid("left_w0", (Gains){2.25, 0.08, 0.002});
    left.set_joint_pid("left_e1", (Gains){1.75, 0.08, 0.002});
    left.set_joint_pid("left_e0", (Gains){1.75, 0.08, 0.002});
    left.set_joint_pid("left_s1", (Gains){1.5, 0.05, 0.002});
    left.set_joint_pid("left_s0", (Gains){1.5, 0.05, 0.002}); //0
    left.set_allowable_error(0.015);
    left.set_max_velocity(.5);
    left.set_max_acceleration(70);
    
    // taken from BCTests.cpp
    right.set_joint_pid("right_w2", (Gains){3.5, .5, 0.1}); //6
    right.set_joint_pid("right_w1", (Gains){4, .5, 0.05});
    right.set_joint_pid("right_w0", (Gains){3.5, .1, 0.002});
    right.set_joint_pid("right_e1", (Gains){3.75, .1, 0.15});
    right.set_joint_pid("right_e0", (Gains){3.25, .1, 0.002});
    right.set_joint_pid("right_s1", (Gains){1.75, .03, 0.02});
    right.set_joint_pid("right_s0", (Gains){2.25, .4, 0.015}); //0
    right.set_allowable_error(0.008);
    right.set_max_velocity(.2);
    right.set_max_acceleration(20);
    Gains epGains(.6,.25,.05);
    //right.set_endpoint_pid(epGains);
}