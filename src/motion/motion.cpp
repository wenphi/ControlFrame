#include "motion.hpp"
using namespace MOTION;
actionBase *motion::praseCmdtoAction(Json::Value jsonCmd)
{
    // std::vector<autoValue> params;
    // MsgToParams(jsonCmd, params);
    Json::Value root;
    actionBase *cmdbase_;
    switch (jsonCmd["cmd"].asInt())
    {
    case motionCmd_t::MOTION_CMD_ADD_LINE:
        cmdbase_ = new motionCmdAddLine;
        std::cout << "motion:Get Cmd add line!" << std::endl;
        break;
    default:
        cmdbase_ = nullptr;
        std::cout << "motion: Unkow command!" << std::endl;
    }
    if (cmdbase_)
    {
        root = jsonCmd["params"];
        cmdbase_->setParams(root);
    }
    return cmdbase_;
};

/*--------------motion---------------*/
void motion::motionInit()
{
    tcqQueue.tcqCreate(100, tcQueue);
}