#include "cubic.hpp"

namespace MOTION
{

//cubic 计算参数
int cubicComputeFactor(const CubicParam_t &para, CubicFactor_t &factor, double t)
{
    double tt = t * t;
    double ttt = tt * t;
    factor.a0 = para.posit[0];
    factor.a1 = para.vel[0];
    factor.a2 = (-3 * (para.posit[0] - para.posit[1]) - (2 * para.vel[0] + para.vel[1]) * t) / tt;
    factor.a3 = (2 * (para.posit[0] - para.posit[1]) + (para.vel[0] + para.vel[1]) * t) / ttt;
    return 0;
}

//cubic 计算结果
void cubicComputeResult(CubicFactor_t &factor, double t, double &posit, double &vel, double &acc)
{
    double tt = t * t;
    double ttt = tt * t;
    posit = factor.a3 * ttt + factor.a2 * tt + factor.a1 * t + factor.a0;
    vel = 3 * factor.a3 * tt + 2 * factor.a2 * t + factor.a1;
    acc = 6 * factor.a3 * t + 2 * factor.a2;
    return;
}

} // namespace MOTION