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

moduleBase *robotStateMechine::getModuleBasePtr()
{
    return baseInstance;
}

/*----------------------------------消息机钩子-----------------------------------*/
void robotStateMechine::updateHook()
{
    if (!stopFlag)
    {
        recvJson = zmqServer->recvJson();
        if (!recvJson.isNull())
        {
            replyJson = stateProcessing(recvJson);
            if (replyJson["needReply"].asBool())
            {
                if (zmqServer->sendJson(replyJson) < 0)
                    std::cout << "robot state mechine reply message failed!" << std::endl;
            }
        }
    }
}

Json::Value robotStateMechine::stateProcessing(const Json::Value jsonData)
{
    std::lock_guard<std::mutex> lock(mutex_msg_processing);
    clearHolder();
    if (parseMessage(jsonData))
    {
        switch (msgHolder.msgType)
        {
        case msgtype_t::MSG_TYPE_STATE:
            updateStateHolder(msgHolder.msgData);
            break;
        case msgtype_t::MSG_TYPE_CMD:
            preStateBase->filterHook(msgHolder);
            if (msgHolder.msgIsValid)
            {
                baseInstance = praseCmdToModule();
                if (baseInstance != nullptr)
                {
                    (*baseInstance)();
                    replyHolder.moduleReply = baseInstance->outPutReply();
                    updateStateHolder(replyHolder.moduleReply["stateMsg"]);
                }
            }
            break;
        case msgtype_t::MSG_TYPE_PUB:
            break;
        default:
            std::cout << "robotStateMechine: Unknow message type! " << std::endl;
        }
    }
    preStateBase->updateHook(this);
    nextStateBase = preStateBase->updateState(stateHolder);
    if (nextStateBase != nullptr)
    {
        delete preStateBase;
        preStateBase = nextStateBase;
    }
    return getReplyJson();
}

bool robotStateMechine::parseMessage(const Json::Value jsonData)
{
    if (jsonData.isNull())
        return false;
    //消息类型
    if (jsonData["msgType"].isNull())
        return false;
    msgHolder.msgType = jsonData["msgType"].asInt();
    switch (msgHolder.msgType)
    {
    case msgtype_t::MSG_TYPE_CMD:
        //执行模块
        if (jsonData["module"].isNull())
            return false;
        msgHolder.modules = jsonData["module"].asInt();
        //指令参数
        if (jsonData["msgData"].isNull())
            return false;
        msgHolder.msgData = jsonData["msgData"];
        break;
    case msgtype_t::MSG_TYPE_STATE:
        break;
    case msgtype_t::MSG_TYPE_PUB:
        break;
    default:
        return false;
    }
    //解析发布消息
    if (jsonData["needReply"].isNull())
        msgHolder.needReply = false;
    else
        msgHolder.needReply = jsonData["needReply"].asBool();
    //重置标志位
    msgHolder.msgIsValid = false;
    return true;
}

moduleBase *robotStateMechine::praseCmdToModule()
{
    //根据参数进行选取子类
    //指向子类
    switch (msgHolder.modules)
    {
    case msgModule_t::MSG_MODULE_HELLO:
        if (ptrhello == NULL)
        {
            baseInstance = NULL;
            std::cout << "robotStateMechine: The hello module unregistered!" << std::endl;
            break;
        }
        baseInstance = ptrhello;
        break;
    default:
        baseInstance = NULL;
        std::cout << "robotStateMechine: Unknow module!" << std::endl;
    }
    if (baseInstance != NULL)
        baseInstance->setMessage(msgHolder.msgData);
    return baseInstance;
}
Json::Value robotStateMechine::getReplyJson()
{
    Json::Value msgReply_JSON;
    msgReply_JSON["needReply"] = msgHolder.needReply;
    if (replyHolder.moduleReply.isNull())
        msgReply_JSON["replyMessage"] = replyHolder.stateMechineReply;
    else
        msgReply_JSON["replyMessage"] = replyHolder.moduleReply["resultMsg"];

    return msgReply_JSON;
}

void robotStateMechine::clearHolder()
{
    Json::Value clearJson;
    msgHolder.modules = msgModule_t::MSG_MODULE_UNDEFINE;
    msgHolder.msgData = clearJson;
    msgHolder.msgIsNull = true;
    msgHolder.msgIsValid = false;
    msgHolder.needReply = false;
    msgHolder.msgType = msgtype_t::MSG_TYPE_UNDEFINE;

    replyHolder.moduleReply = clearJson;
    replyHolder.stateMechineReply = clearJson;
}