#pragma once
#include <iostream>
#include "libjsoncpp/include/json.h"

enum msgModule_t
{
    MSG_MODULE_UNDEFINE = 0,
    MSG_MODULE_API,
    MSG_MODULE_MOTION,
    MSG_MODULE_IO,
    MSG_MODULE_HELLO,
};

enum msgCmd_t
{
    MSG_CMD_UNDEFINE = 0,
    MSG_CMD_HELLO_UNDEFINE = 100,
    MSG_CMD_MOTION_UNDEFINE = 200,
};

class robotStateMechine;
enum msgState_t
{
    MSG_STATE_NONE,
    MSG_STATE_EMERGENCY,
    MSG_STATE_HOMED,
};

enum msgtype_t
{
    MSG_TYPE_UNDEFINE,
    MSG_TYPE_CMD,
    MSG_TYPE_STATE,
    MSG_TYPE_PUB,
};
struct stateHolder_t
{
    bool emergencyStop;
    bool drag;
    bool jointHomed[6];
};

//命令消息执行动作
//状态消息更新状态
struct msgHolder_t
{
    bool msgIsValid;     //消息是否有效
    bool msgIsNull;      //是否获取到消息
    int msgType;         //消息类型
    int modules;         //消息执行模块
    Json::Value msgData; //消息主体
    bool needReply;      //是否需要返回
};
struct replyHolder_t
{
    Json::Value moduleReply;
    Json::Value stateMechineReply;
};

class stateBase
{
  public:
    stateBase(){};
    virtual ~stateBase(){};

    virtual void filterHook(msgHolder_t &msgHolder) = 0;            //消息过滤
    virtual void updateHook(robotStateMechine *rsm) = 0;            //执行
    virtual stateBase *updateState(const stateHolder_t &state) = 0; //状态更新
};