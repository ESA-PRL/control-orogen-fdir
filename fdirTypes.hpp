#ifndef FDIR_TYPES_HPP
#define FDIR_TYPES_HPP
namespace fdir{
    enum FdirState
    {
        NOMINAL,
        EXCEPTION_ATTITUDE,
        EXCEPTION_TRAJECTORY,
        EXCEPTION_MOTORS,
        EXCEPTION_SLIPPAGE
    };
}
#endif
