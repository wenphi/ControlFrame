#include "tcq.hpp"

using namespace MOTION;

bool TrajectoryCalcQueue::tcqCheck()
{
    if ((nullptr == queue) || (0 == size))
    {
        //判断队列是否有效
        return false;
    }
    return true;
}

bool TrajectoryCalcQueue::tcqCreate(const int set_size, TrajectoryCalc *const tcSpace)
{
    if (set_size <= 0 || tcSpace == 0)
    {
        return false;
    }
    queue = tcSpace;
    size = set_size;
    tcqClear();
    return 0;
}

void TrajectoryCalcQueue::tcqClear()
{
    //清理tcq
    length = 0;
    start = 0;
    end = 0;
    return;
}

bool TrajectoryCalcQueue::tcqPushBack(const TrajectoryCalc &tc)
{
    if (!tcqCheck())
        return false;
    if (tcqIsFull())
        return false;
    queue[end] = tc;
    length++;
    end = (end + 1) % size;
    return true;
}

bool TrajectoryCalcQueue::tcqPopBack()
{
    if (!tcqCheck())
        return false;
    int n = end - 1 + size;
    end = n % size;
    length--;
    return true;
}

bool TrajectoryCalcQueue::tcqRemove(const int num)
{
    if (num <= 0 || num > length)
        return false;
    if (!tcqCheck())
        return false;
    start = (start + num) % size;
    length -= num;
    return true;
}

int TrajectoryCalcQueue::tcqLen()
{
    if (!tcqCheck())
        return 0;
    return length;
}

TrajectoryCalc *TrajectoryCalcQueue::tcqItem(const int posit)
{
    if (!tcqCheck())
        return nullptr;
    if ((posit < 0) || (posit >= length))
        return nullptr;
    return &(queue[(start + posit) % size]);
}

bool TrajectoryCalcQueue::tcqIsFull()
{
    //如果队列不存在，认为是满的
    if (!tcqCheck())
        return true;
    if (length < size)
        return false;
    return true;
}

bool TrajectoryCalcQueue::tcqIsEmpty()
{
    //如果队列不存在，认为是空的
    if (!tcqCheck())
        return true;
    if (length == 0)
        return true;
    return false;
}