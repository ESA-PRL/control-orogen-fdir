#ifndef FDIR_TASK_TASK_HPP
#define FDIR_TASK_TASK_HPP

#include "fdir/TaskBase.hpp"

namespace fdir{

    class Task : public TaskBase
    {
	friend class TaskBase;
    protected:
        // Switches to fault_state or NOMINAL state depending on current state
        // and fault_detected.
        // Returns whether the state had to be switched.
        bool switchState(bool fault_detected, TaskBase::States fault_state);

    public:
        Task(std::string const& name = "fdir::Task");
        Task(std::string const& name, RTT::ExecutionEngine* engine);
        ~Task();

        bool configureHook();
        bool startHook();
        void updateHook();
        void errorHook();
        void stopHook();
        void cleanupHook();
    };
}

#endif
