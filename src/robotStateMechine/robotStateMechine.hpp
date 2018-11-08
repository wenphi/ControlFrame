#pragma once
#include <mutex>
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

  void updateHook();                                       //钩子函数
  static moduleBase *getModuleBasePtr();                   //返回模块基类指针
  Json::Value stateProcessing(const Json::Value jsonData); //内部使用,调用则执行jsonData

  void setStop() { stopFlag = true; }
  void setStart() { stopFlag = false; }

  /*注册函数*/
public:
  bool registerMessageServer(messageServer *msgServer_);
  bool registerHello(hello *hello_);

private:
  Json::Value getReplyJson();                      //获取返回消息
  moduleBase *praseCmdToModule();                  //筛选消息执行模块
  bool parseMessage(const Json::Value jsonData);   //解析消息
  void updateStateHolder(Json::Value) { return; }; //更新状态字
  void clearHolder();                              //清空结构体

private:
  std::mutex mutex_msg_processing; //消息处理互斥
  bool stopFlag;                   //退出标志位
  //变量
  msgHolder_t msgHolder;     //消息数据保存结构体
  replyHolder_t replyHolder; //应答数据保存结构体
  stateHolder_t stateHolder; //状态字保存结构体
  Json::Value recvJson;
  Json::Value replyJson;
  //指针
  hello *ptrhello = nullptr;
  messageServer *zmqServer = nullptr; //zmq服务端
  stateBase *preStateBase = nullptr;  //当前状态
  stateBase *nextStateBase = nullptr; //下一状态
  static moduleBase *baseInstance;    //模块基类指针
};
