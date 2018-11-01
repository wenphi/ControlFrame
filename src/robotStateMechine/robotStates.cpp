#include "robotStates.hpp"
#include "robotStateMechine.hpp"
#include <iostream>
//Init
void stateInit::filterHook(msgHolder_t &msgHolder)
{
    msgHolder.msgIsValid = true;
}

void stateInit::updateHook(robotStateMechine *rsm)
{
    return;
}
stateBase *stateInit::updateState(const stateHolder_t &state)
{
    if (1)
        return new stateRunning();
}

//running
void stateRunning::filterHook(msgHolder_t &msgHolder)
{
    msgHolder.msgIsValid = true;
}
void stateRunning::updateHook(robotStateMechine *rsm)
{
    return;
}

stateBase *stateRunning::updateState(const stateHolder_t &state)
{
    if (1)
        return nullptr;
}
