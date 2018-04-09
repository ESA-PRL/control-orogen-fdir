#include "Task.hpp"
#include <cmath>

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
            state(fdir::TaskBase::States::EXCEPTION_SLIPPAGE);
        }
        else if (state() == fdir::TaskBase::States::EXCEPTION_SLIPPAGE)
        {
            state(fdir::TaskBase::States::NOMINAL);
        }
    }

    base::samples::RigidBodyState attitude;
    if (_attitude.read(attitude) == RTT::NewData)
    {
        if (
                std::fabs(attitude.getRoll()) > _max_roll.value() ||
                std::fabs(attitude.getPitch()) > _max_pitch.value()
           )
        {
            state(fdir::TaskBase::States::EXCEPTION_ATTITUDE);
        }
        else if (state() == fdir::TaskBase::States::EXCEPTION_ATTITUDE)
        {
            state(fdir::TaskBase::States::NOMINAL);
        }
    }

    int trajectory_status;
    if (_trajectory_status.read(trajectory_status) == RTT::NewData)
    {
        //if (trajectory_status == waypoint_navigation_lib::OUT_OF_BOUNDARIES)
        if (trajectory_status == 3) //TODO: let waypoint navigation output enum instead of integer
        {
            state(fdir::TaskBase::States::EXCEPTION_TRAJECTORY);
        }
        else if (state() == fdir::TaskBase::States::EXCEPTION_TRAJECTORY)
        {
            state(fdir::TaskBase::States::NOMINAL);
        }
    }

    switch(state())
    {
        case fdir::TaskBase::States::NOMINAL:
            _fault_detected.write(false);
            _fdir_state.write(fdir::FdirState::NOMINAL);
            break;
        case fdir::TaskBase::States::EXCEPTION_ATTITUDE:
            _fault_detected.write(true);
            _fdir_state.write(fdir::FdirState::EXCEPTION_ATTITUDE);
            break;
        case fdir::TaskBase::States::EXCEPTION_SLIPPAGE:
            _fault_detected.write(true);
            _fdir_state.write(fdir::FdirState::EXCEPTION_SLIPPAGE);
            break;
        case fdir::TaskBase::States::EXCEPTION_TRAJECTORY:
            _fault_detected.write(true);
            _fdir_state.write(fdir::FdirState::EXCEPTION_TRAJECTORY);
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
