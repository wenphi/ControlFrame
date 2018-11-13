/*三次多项式*/
#pragma once
#include <math.h>
namespace MOTION
{
struct CubicParam_t
{
    CubicParam_t()
    {
        posit[0] = posit[1] = 0.0;
        posit[0] = posit[1] = 0.0;
    }
    double posit[2];
    double vel[2];
};

struct CubicFactor_t
{
    double a3, a2, a1, a0;
};

//cubic 计算参数
int cubicComputeFactor(const CubicParam_t &para, CubicFactor_t &factor, double t);
//cubic 计算结果
void cubicComputeResult(CubicFactor_t &factor, double t, double &posit, double &vel, double &acc);
} // namespace MOTION
