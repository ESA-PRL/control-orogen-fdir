#ifndef FDIR_TASK_TASK_HPP
#define FDIR_TASK_TASK_HPP

#include "fdir/TaskBase.hpp"

namespace fdir{

    class Task : public TaskBase
    {
	friend class TaskBase;
    protected:

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

