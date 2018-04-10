#ifndef FDIR_TYPES_HPP
#define FDIR_TYPES_HPP
namespace fdir{
    enum FdirState
    {
        FDIR_NOMINAL,
        FDIR_EXCEPTION_ATTITUDE,
        FDIR_EXCEPTION_TRAJECTORY,
        FDIR_EXCEPTION_MOTORS,
        FDIR_EXCEPTION_SLIPPAGE,
        FDIR_EXCEPTION_HAZARD
    };
}
#endif
