#include "motionCommands.hpp"
#include "motion.hpp"
#include "../robotStateMechine/robotStateMechine.hpp"
using namespace MOTION;
bool motionCmdAddLine::run(Json::Value &jsonData)
{
    motion *motionptr;

    //指向hello模块
    motionptr = dynamic_cast<motion *>(robotStateMechine::getModuleBasePtr());
    //调用hello中的方法
    // motionptr->writeData(jsonData["param1"].asInt());
    //设置返回值
    Json::Value jsonReply;
    jsonReply["resultMsg"]["param1"] = jsonData["param1"].asInt();
    motionptr->setReply(jsonReply);
    return true;
};
