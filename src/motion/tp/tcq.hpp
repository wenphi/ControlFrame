#pragma once
#include "tc.hpp"

namespace MOTION
{
//轨迹规划队列
struct TrajectoryCalcQueue
{
    TrajectoryCalc *queue; //用于存储每个队列的参数
    int size;              //队列的大小
    int length;            //当前队列的个数
    int start;             //队列开始的id，一般为0
    int end;               //队列末尾的id
    bool tcqCreate(const int set_size, TrajectoryCalc *const tcSpace);
    bool tcqCheck();
    void tcqClear();
    bool tcqPushBack(const TrajectoryCalc &tc);
    bool tcqPopBack();
    bool tcqRemove(const int num);
    int tcqLen();
    TrajectoryCalc *tcqItem(const int posit);
    bool tcqIsFull();
    bool tcqIsEmpty();
};

} // namespace MOTION
