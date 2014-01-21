#include "baxterLimb.h"
#include "thread.h"

/*
 *      This class is intended to function as a more active velocity controller
 *      for Baxter's limbs when intense processing is being done
 * 
 *      Upon setting a new desired position, controller will actively command
 *      Baxter's arm towards its goal in the background
 *
 */

#ifndef BACKGROUND_CONTROLLER_H
#define BACKGROUND_CONTROLLER_H

class BackgroundController : public Thread
{
public:
    BackgroundController(BaxterLimb*);
    ~BackgroundController();
    
    virtual void *run() {
        _running = true;
        ros::Rate r(_hz);
        while( _running && ros::ok() )
        {
            //std::cout<<"running thread "<<this->self()<<" ";
            if ( _run )
                runBackgroundController();
            r.sleep();
            ros::spinOnce();
        }
        _running = false;
        _run = false;
    }
    
    void set(JointPositions);
    void stop();
    void reset();
    virtual int join();
    virtual int detach();
    
    void setGainRatio(double);
    
private:
    BackgroundController();
    
    BaxterLimb* _limb;
    
    void runBackgroundController();
    
    // helps determine what thread functions should do
    bool _running;
    bool _run;

    // multiply all limb gains by this
    double _gainRatio;
    
    // threading
    /*std::thread* _thread;
    void _resetThread();
    void _deleteThread();
    void _runThread();*/
    
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
    std::vector<double> _compute_output();    
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
    // we don't start running immediately
    _running = false;
    _run = false;
    _gainRatio = .2;
    //std::cout<<"basic setup complete\n";
    // ensure thread starts at NULL
    //_thread = NULL;
    //this->start();
}

BackgroundController::~BackgroundController()
{
    // tells limb to exit gracefully, deletes thread
    _limb->exit_control_mode();
    this->join();
    //_deleteThread();
}

int BackgroundController::join() 
{
    // stops thread from running loop, joins thread
    _running = false;
    _run = false;
    return Thread::join();
}

int BackgroundController::detach()
{
    // allows thread to run
    _running = true;
    return Thread::detach();
}

void BackgroundController::set(JointPositions newGoal)
{
    // sets new goal, starts running controller on thread if not running
    if ( ros::ok() ) {
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
        //_desired.print("desired position");
        
        // tell thread to run
        _run = true;
        //if ( !_running )
          //this->start();
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
    //  calculates and sets joint velocities for _limb
    
    ros::spinOnce();
    //  setup
    _current = _limb->joint_angles();
    _position = _desired.angles;
    _error = v_difference(_position, _current);
    _gains = _limb->get_joint_pids();
    _output.names = _desired.names;
    
    //  get dt, reset _last
    _dt = ros::Time::now() - _last;
    _last = ros::Time::now();
    
    //  calculate integral, derivative error
    _integral = v_sum(_integral, product(_error, toSec(_dt.nsec)));
    _saturate(_integral, .1);
    //_reset_integral(_integral, _error, _previous_error);
    _derivative = quotient(v_difference(_error, _previous_error), toSec(_dt.nsec));
    
    //  compute velocities from error, integral, derivative
    _output.velocities = _compute_output();
   
    //  limit acceleration, velocity
    _limb->limit_acceleration(_output.velocities, _last_vel, toSec(_dt.nsec));
    
    //  set output
    _limb->set_joint_velocities(_output);
    //v_print(_output.velocities);
    
    //  update for next call
    _last_vel = _output.velocities;
    _previous_error = _error;        
}

void BackgroundController::stop()
{
    // gracefully stop the controller
    if ( ros::ok() ) {
        //_running = false;
        _run = false;
        _limb->exit_control_mode();
        reset();
    }
}

void BackgroundController::reset()
{
    // resets error states tracked for pid controller
    if ( ros::ok() ) {
        int num = _desired.angles.size();
        _previous_error.resize(num, 0);
        _integral.resize(num, 0);
        _derivative.resize(num, 0);
        _last_vel.resize(num, 0);
    }
}

std::vector<double> BackgroundController::_compute_output()
{
    // calculates output signal from errors, gains
    int len = _error.size();
    std::vector<double> temp(len,0);
    for(int i = 0; i < len; i++)
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
/*
void BackgroundController::_resetThread()
{
    _deleteThread();
    _thread = new std::thread(&BackgroundController::_runThread, this);
}

void BackgroundController::_deleteThread()
{
    // deletes thread, sets to null
    if (_thread == NULL)
        return;
    // else
    try{
        _thread->join();
    }
    catch (std::system_error e) {
        std::cout<<"Error joining thread on reset: "<<e.what()<<"\n";
        //_thread->detach();
    }
    delete _thread;
    _thread = NULL;
}

void BackgroundController::_runThread()
{
    _running = true;
    while( _running && ros::ok() )
    {
        //std::cout<<"running thread";
        if ( _run )
            runBackgroundController();
    }
    _running = false;
    _run = false;
}*/

#endif