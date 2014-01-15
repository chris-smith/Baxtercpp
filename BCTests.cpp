#include "backgroundController.h"

void setup_pid(BaxterLimb&);

int main(int argc, char** argv) {
    ros::init(argc, argv, "Background_Controller_Test");
    
    BaxterLimb limb("right");
    BackgroundController bc(&limb);
    
    setup_pid(limb);
    
    JointPositions jp = limb.joint_positions();
    std::vector<double> pos(NUMJOINTS, 0);
    pos[0] = PI/4;
    jp.angles = pos;
    bc.setBackgroundController(jp);
    
    while( ros::ok() )
        ros::spinOnce();
    
    limb.joint_positions().print("Final Positions");
    
    return 1;
}

void setup_pid(BaxterLimb& limb)
{
    limb.set_joint_pid("right_w2", (Gains){4.5, .2, 0.08}); //6
    limb.set_joint_pid("right_w1", (Gains){3.5, .02, 0.007});
    limb.set_joint_pid("right_w0", (Gains){3.5, .1, 0.002});
    limb.set_joint_pid("right_e1", (Gains){3.5, .4, 0.007});
    limb.set_joint_pid("right_e0", (Gains){3.25, .1, 0.002});
    limb.set_joint_pid("right_s1", (Gains){2.5, .05, 0.015});
    limb.set_joint_pid("right_s0", (Gains){2.5, .1, 0.015}); //0
    limb.set_allowable_error(0.008);
    limb.set_max_velocity(.15);
    limb.set_max_acceleration(10);
    Gains epGains(.6,.25,.05);
    //limb.set_endpoint_pid(epGains);
}