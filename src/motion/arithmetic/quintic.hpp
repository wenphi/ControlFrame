/*五次多项式*/
#pragma once
namespace MOTION
{
struct QuinticParam
{
    double p0, dp0, ddp0;
    double p1, dp1, ddp1;
};

struct QuinticFactor
{
    double c1, c2, c3, c4, c5, c6;
};
void quinticComputeFactor(const QuinticParam &param, QuinticFactor &factor, double t);
void quinticComputeResult(const QuinticFactor &factor, double t, double &posit, double &vel, double &acc);

} // namespace MOTION
