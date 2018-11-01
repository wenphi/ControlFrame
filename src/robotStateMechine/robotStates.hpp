#pragma once
#include "../base/stateBase.hpp"

//状态机的各状态
class stateInit : public stateBase
{
    void filterHook(msgHolder_t &msgHolder);
    void updateHook(robotStateMechine *rsm);
    stateBase *updateState(const stateHolder_t &state);
};

class stateRunning : public stateBase
{
    void filterHook(msgHolder_t &msgHolder);
    void updateHook(robotStateMechine *rsm);
    stateBase *updateState(const stateHolder_t &state);
};