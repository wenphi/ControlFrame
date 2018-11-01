#include "robotStateMechine.hpp"
#include <cstdlib>
#include <cstring>
#include <unistd.h>

//注册模块
bool robotStateMechine::registerMessageServer(messageServer *msgServer_)
{
    if (msgServer_ == nullptr)
        return false;
    zmqServer = msgServer_;
    return true;
}

bool robotStateMechine::registerHello(hello *hello_)
{
    if (hello_ == nullptr)
        return false;
    ptrhello = hello_;
    return true;
}

//
moduleBase *robotStateMechine::getModuleBasePtr()
{
    return baseInstance;
}
moduleBase *robotStateMechine::praseCmdToModule()
{
    //根据参数进行选取子类
    //指向子类
    switch (msgHolder.modules)
    {
    case msgModule_t::MSG_MODULE_HELLO:
        baseInstance = ptrhello;
        break;
    default:
        baseInstance = nullptr;
    }
    if (baseInstance != nullptr)
        baseInstance->setMessage(msgHolder.msgData);
    return baseInstance;
}

void robotStateMechine::updateHook()
{
    while (!stopFlag)
    {
        if (recvMessage())
        {
            if (msgHolder.msgType == msgtype_t::MSG_STATE)
                updateStateHolder(msgHolder.msgData);
            else
            {
                preStateBase->filterHook(msgHolder);
                if (msgHolder.msgIsValid)
                {
                    baseInstance = praseCmdToModule();
                    if (baseInstance != nullptr)
                    {
                        (*baseInstance)();
                        replyHolder.moduleReply = baseInstance->outPutReply();
                        updateStateHolder(replyHolder.moduleReply);
                    }
                }
            }
            if (msgHolder.needReply)
                replyMessage();
        }
        preStateBase->updateHook(this);
        nextStateBase = preStateBase->updateState(stateHolder);
        if (nextStateBase != nullptr)
        {
            delete preStateBase;
            preStateBase = nextStateBase;
        }
    }
}

bool robotStateMechine::replyMessage()
{
    Json::Value msgReply_JSON;
    msgReply_JSON["replyMessage"] = replyHolder.moduleReply;
    if (zmqServer->sendJson(msgReply_JSON) < 0)
        std::cout << "reply message failed!" << std::endl;
    return true;
}

//test
//获取消息
bool robotStateMechine::recvMessage()
{
    Json::Value data = zmqServer->recvJson();

    if (data.isNull())
    {
        msgHolder.msgIsNull = true;
        return false;
    }
    if (data["msgType"].isNull())
        return false;
    msgHolder.msgType = data["msgType"].asInt();
    if (data["module"].isNull())
        return false;
    msgHolder.modules = data["module"].asInt();
    if (data["msgData"].isNull())
        return false;
    msgHolder.msgData = data["msgData"];
    if (data["needReply"].isNull())
        return false;
    msgHolder.needReply = data["needReply"].asBool();
    msgHolder.msgIsValid = false;
    msgHolder.msgIsNull = false;
    // std::cout << "stateMechine:debug:" << data.toStyledString() << std::endl;
    return true;
}