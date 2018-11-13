#include "liborocosKdl/include/frames_io.hpp"
#include "motion/tp/tc.hpp"
#include <csignal>
using namespace MOTION;
bool stopFlag = false;

void stop(int sig)
{
    if (sig)
        stopFlag = true;
}

int main()
{
    signal(SIGINT, stop);
    TrajectoryCalc *tc = new TrajectoryCalc();
    tc->initialTc(0.02, 0, 0, 10, 10, 10, 0, 100);
    TcRunParam param;
    param.limit_acc = 10;
    param.limit_ve = 0;
    param.limit_vm = 10;

    while (!stopFlag)
    {
        tc->tcRunCycleQuinticS(param);
        std::cout << tc->progress << std::endl;
        usleep(1000);
    }
    delete tc;
}