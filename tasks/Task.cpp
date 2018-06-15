#include "Task.hpp"
#include <cmath>

const double DEG2RAD = M_PI/180;

using namespace fdir;

Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
}

Task::~Task()
{
}

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;
    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();

    double slip_ratio;
    if (_slip_ratio.read(slip_ratio) == RTT::NewData)
    {
        if (slip_ratio > _max_slip.value())
        {
            state(EXCEPTION_SLIPPAGE);
        }
        else if (state() == EXCEPTION_SLIPPAGE)
        {
            state(NOMINAL);
        }
    }

    base::samples::RigidBodyState attitude;
    if (_attitude.read(attitude) == RTT::NewData)
    {
        if (
                std::fabs(attitude.getRoll()) > (_max_roll.value()*DEG2RAD) ||
                std::fabs(attitude.getPitch()) > (_max_pitch.value()*DEG2RAD)
           )
        {
            state(EXCEPTION_ATTITUDE);
        }
        else if (state() == EXCEPTION_ATTITUDE)
        {
            state(NOMINAL);
        }
    }

    int error_in_motor;
    if (_error_in_motor.read(error_in_motor) == RTT::NewData)
    {
        if (error_in_motor) // Any number different than 0 is an error in that motor number. If it is 0 then all motors are ok.
        {
            state(EXCEPTION_MOTORS);
        }
        else if (state() == EXCEPTION_MOTORS)
        {
            state(NOMINAL);
        }
    }

    int trajectory_status;
    if (_trajectory_status.read(trajectory_status) == RTT::NewData)
    {
        //if (trajectory_status == waypoint_navigation_lib::OUT_OF_BOUNDARIES)
        if (trajectory_status == 3) //TODO: let waypoint navigation output enum instead of integer
        {
            state(EXCEPTION_TRAJECTORY);
        }
        else if (state() == EXCEPTION_TRAJECTORY)
        {
            state(NOMINAL);
        }
    }

    bool hazard_detected;
    if (_hazard_detected.read(hazard_detected) == RTT::NewData)
    {
        if (hazard_detected)
        {
            state(EXCEPTION_HAZARD);
        }
        else if (state() == EXCEPTION_HAZARD)
        {
            state(NOMINAL);
        }
    }

    switch(state())
    {
        case NOMINAL:
            _fault_detected.write(false);
            _fdir_state.write(FDIR_NOMINAL);
            break;
        case EXCEPTION_MOTORS:
            _fault_detected.write(true);
            _fdir_state.write(FDIR_EXCEPTION_MOTORS);
            break;
        case EXCEPTION_ATTITUDE:
            _fault_detected.write(true);
            _fdir_state.write(FDIR_EXCEPTION_ATTITUDE);
            break;
        case EXCEPTION_SLIPPAGE:
            _fault_detected.write(true);
            _fdir_state.write(FDIR_EXCEPTION_SLIPPAGE);
            break;
        case EXCEPTION_TRAJECTORY:
            _fault_detected.write(true);
            _fdir_state.write(FDIR_EXCEPTION_TRAJECTORY);
            break;
        case EXCEPTION_HAZARD:
            _fault_detected.write(true);
            _fdir_state.write(FDIR_EXCEPTION_HAZARD);
            break;
        case RUNNING:
            state(NOMINAL);
            break;
        default:
            std::cerr << "FDIR: Should not reach this point. State is: " << state() << std::endl;
            break;
    }
}
void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}
