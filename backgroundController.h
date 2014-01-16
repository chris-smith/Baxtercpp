#include "baxterLimb.h"

/*
 *      This class is intended to function as a more active velocity controller
 *      for Baxter's limbs when intense processing is being done
 * 
 *      Upon setting a new desired position, controller will actively command
 *      Baxter's arm towards its goal
 *
 */

#ifndef BACKGROUND_CONTROLLER_H
#define BACKGROUND_CONTROLLER_H

class BackgroundController 
{
public:
    BackgroundController(BaxterLimb*);
    ~BackgroundController();
    
    void setBackgroundController(JointPositions);
    void runBackgroundController();
    void stopBackgroundController();
    void resetBackgroundController();
    
    void setGainRatio(double);
    
private:
    BackgroundController();
    std::vector<double> _compute_gains();
    
    BaxterLimb* _limb;
    
    bool _stop;
    bool _running;

    double _gainRatio;
    
    // used to calculate, set joint velocities
    JointPositions _desired;  // desired position
    ros::Time _last;     // = ros::Time::now();
    double _hz;        // (hz); 
    ros::Duration _dt;
    std::vector<double> _current;
    std::vector<double> _position;
    std::vector<double> _error;
    std::vector<double> _previous_error;//(position.size(),0);
    std::vector<double> _integral;//(position.size(),0);
    std::vector<double> _derivative;//(position.size(),0);
    std::vector<double> _last_vel;// = joint_velocities();
    std::vector<Gains> _gains;
    JointVelocities _output;
};

BackgroundController::BackgroundController()
{
    // this shouldn't happen, needs the limb
}

BackgroundController::BackgroundController(BaxterLimb* limb)
{
    _limb = limb;
    _hz = 100;
    // size vectors appropriately
    int num = limb->joint_angles().size();
    _previous_error.resize(num, 0);
    _integral.resize(num, 0);
    _derivative.resize(num, 0);
    _last_vel.resize(num, 0);
    _stop = false;
    _running = false;
    _gainRatio = .8;
}

BackgroundController::~BackgroundController()
{
    _limb->exit_control_mode();
}

void BackgroundController::setBackgroundController(JointPositions newGoal)
{
    // sets new goal, starts running controller if not running
    int num = newGoal.angles.size();
    if ( num != _integral.size() )
    {
        // resize if necessary
        _previous_error.resize(num, 0);
        _integral.resize(num, 0);
        _derivative.resize(num, 0);
        _last_vel.resize(num, 0);
    }
    // set new goal
    _desired = newGoal;
    _desired.print("desired position");
    // run if not running
    if (!_running) {
        _running = true;
        this->runBackgroundController();
    }
}

void printGains(std::vector<Gains> gains)
{
    for(int i = 0; i < gains.size(); i++)
    {
        std::cout<<"kp "<<gains[i].kp<<" ki "<<gains[i].ki<<" kd "<<gains[i].kd<<"\n";
    }
}

void BackgroundController::runBackgroundController()
{
    //
    _running = true;
    ros::spinOnce();
    _current = _limb->joint_angles();
    _position = _desired.angles;
    _error = v_difference(_position, _current);
    //v_print(_error,"error");
    _gains = _limb->get_joint_pids();
    //printGains(_gains);
    
    _output.names = _desired.names;
    
    // get dt
    _dt = ros::Time::now() - _last;
    //_dt.nsec = (toSec(_dt.nsec) < 1/hz ? toNsec(1/hz) : _dt.nsec);
    _last = ros::Time::now();
    // calculate integral, derivative error
    _integral = v_sum(_integral, product(_error, toSec(_dt.nsec)));
    _saturate(_integral, .1);
    //_reset_integral(_integral, _error, _previous_error);
    _derivative = quotient(v_difference(_error, _previous_error), toSec(_dt.nsec));
    // compute velocities from error, integral, derivative
    //v_print(_error, "error");
    _output.velocities = _compute_gains();
    // limit acceleration
    _limb->limit_acceleration(_output.velocities, _last_vel, toSec(_dt.nsec));
    _limb->set_joint_velocities(_output);
    //v_print(_output.velocities);
    _last_vel = _output.velocities;
    _previous_error = _error;        
    
    // sleep to keep rate constant
    ros::Rate(_hz).sleep();
    //_rate.sleep();
    // if not asked to stop, loop
    if ( !_stop && ros::ok() )
        runBackgroundController();
    else 
    {
        // stop running, reset stop command
        _running = false;
        _stop = false;
        _limb->exit_control_mode();
    }
}

void BackgroundController::stopBackgroundController()
{
    _stop = true;
}

void BackgroundController::resetBackgroundController()
{
    int num = _desired.angles.size();
    _previous_error.resize(num, 0);
    _integral.resize(num, 0);
    _derivative.resize(num, 0);
    _last_vel.resize(num, 0);
}

std::vector<double> BackgroundController::_compute_gains()
{
    std::vector<double> temp(_error.size(),0);
    for(int i = 0; i < _error.size(); i++)
    {
        temp[i] = _error[i] * _gains[i].kp * _gainRatio;
        temp[i] += _integral[i] * _gains[i].ki * _gainRatio;
        temp[i] += _derivative[i] * _gains[i].kd * _gainRatio;
    }
    return temp;
}

void BackgroundController::setGainRatio(double newRatio)
{
    _gainRatio = newRatio;
}


#endif