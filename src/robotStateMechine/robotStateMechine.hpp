#pragma once
#include "../hello/hello.hpp"
#include "../message/messageServer.hpp"
#include "robotStates.hpp"

class robotStateMechine
{
public:
  robotStateMechine()
  {
    preStateBase = new stateInit();
    stopFlag = false;
  };
  ~robotStateMechine()
  {
    delete preStateBase;
  };

  void updateHook();
  //返回基类指针
  static moduleBase *getModuleBasePtr();

  void setStop() { stopFlag = true; }
  void setStart() { stopFlag = false; }

protected:
  //得到指定模块子类
  moduleBase *praseCmdToModule();
  //刷新状态字
  void updateStateHolder(Json::Value) { return; };
  //new test
  bool recvMessage();
  bool replyMessage();

public:
  bool registerMessageServer(messageServer *msgServer_);
  bool registerHello(hello *hello_);

private:
  bool stopFlag;             //退出标志位
  msgHolder_t msgHolder;     //消息数据保存结构体
  replyHolder_t replyHolder; //应答数据保存结构体
  stateHolder_t stateHolder; //状态字保存结构体
  hello *ptrhello = nullptr;
  messageServer *zmqServer = nullptr; //zmq服务端
  stateBase *preStateBase = nullptr;  //当前状态
  stateBase *nextStateBase = nullptr; //下一状态
  static moduleBase *baseInstance;    //模块基类指针
};
