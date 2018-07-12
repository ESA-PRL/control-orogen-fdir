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

    bool state_switched = false;

    double slip_ratio;
    if (_slip_ratio.read(slip_ratio) == RTT::NewData)
    {
        state_switched |= switchState(slip_ratio > _max_slip.value(), EXCEPTION_SLIPPAGE);
    }

    base::samples::RigidBodyState attitude;
    if (_attitude.read(attitude) == RTT::NewData)
    {
        bool fault_detected =
                std::fabs(attitude.getRoll()) > (_max_roll.value()*DEG2RAD) ||
                std::fabs(attitude.getPitch()) > (_max_pitch.value()*DEG2RAD);
        state_switched |= switchState(fault_detected, EXCEPTION_ATTITUDE);
    }

    int error_in_motor;
    if (_error_in_motor.read(error_in_motor) == RTT::NewData)
    {
        // Any number different than 0 is an error in that motor number. If it is 0 then all motors are ok.
        state_switched |= switchState(error_in_motor, EXCEPTION_MOTORS);
    }

    int trajectory_status;
    if (_trajectory_status.read(trajectory_status) == RTT::NewData)
    {
        //if (trajectory_status == waypoint_navigation_lib::OUT_OF_BOUNDARIES)
        //TODO: let waypoint navigation output enum instead of integer
        state_switched |= switchState(trajectory_status == 3, EXCEPTION_TRAJECTORY);
    }

    bool hazard_detected;
    if (_hazard_detected.read(hazard_detected) == RTT::NewData)
    {
        state_switched |= switchState(hazard_detected, EXCEPTION_HAZARD);
    }

    if (state_switched)
    {
        writeToPorts();
    }
}

bool Task::switchState(bool fault_detected, TaskBase::States fault_state)
{
    bool state_switched = false;
    if (fault_detected && state() != fault_state)
    {
        state(fault_state);
        state_switched = true;
    }
    else if (!fault_detected && state() == fault_state)
    {
        state(NOMINAL);
        state_switched = true;
    }
    return state_switched;
}

void Task::writeToPorts()
{
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
