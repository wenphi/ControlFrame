#include "quintic.hpp"
namespace MOTION
{
void quinticComputeFactor(const QuinticParam &param, QuinticFactor &factor, double t)
{
    double t2 = t * t;
    double t3 = t2 * t;
    double t4 = t3 * t;
    double t5 = t4 * t;
    factor.c1 = param.p0;
    factor.c2 = param.dp0;
    factor.c3 = param.ddp0 * 0.5;
    factor.c4 = (20.0 * (param.p1 - param.p0) - (8.0 * param.dp1 + 12.0 * param.dp0) * t - (3.0 * param.ddp0 - param.ddp1) * t2) / (2.0 * t3);
    factor.c5 = (30.0 * (param.p0 - param.p1) + (14.0 * param.dp1 + 16.0 * param.dp0) * t + (3.0 * param.ddp0 - 2.0 * param.ddp1) * t2) / (2.0 * t4);
    factor.c6 = (12.0 * (param.p1 - param.p0) - (6.0 * param.dp1 + 6.0 * param.dp0) * t - (param.ddp0 - param.ddp1) * t2) / (2.0 * t5);
    return;
}

void quinticComputeResult(const QuinticFactor &factor, double t, double &posit, double &vel, double &acc)
{
    double t2 = t * t;
    double t3 = t2 * t;
    double t4 = t3 * t;
    double t5 = t4 * t;
    posit = factor.c6 * t5 + factor.c5 * t4 + factor.c4 * t3 + factor.c3 * t2 + factor.c2 * t + factor.c1;
    vel = 5 * factor.c6 * t4 + 4 * factor.c5 * t3 + 3 * factor.c4 * t2 + 2 * factor.c3 * t + factor.c2;
    acc = 20 * factor.c6 * t3 + 12 * factor.c5 * t2 + 6 * factor.c4 * t + 2 * factor.c3;
    return;
}
} // namespace MOTION