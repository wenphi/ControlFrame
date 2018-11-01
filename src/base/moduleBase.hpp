#pragma once
#include <iostream>
#include <list>
#include "actionBase.hpp"

class moduleBase
{
  // friend actionBase;

public:
  moduleBase()
  {
    needReply = false;
  }
  virtual ~moduleBase() {}
  virtual void operator()()
  {
    cmdBase = praseCmdtoAction(messageHolder);
    if (cmdBase != nullptr)
    {
      cmdBase->start();
      delete cmdBase;
    }
  };

  virtual actionBase *praseCmdtoAction(Json::Value JsonCmd) = 0;
  //输出
  Json::Value outPutReply()
  {
    return messageReply;
  }
  //输入
  bool setMessage(Json::Value &msg)
  {
    messageHolder = msg;
    return true;
  };
  bool setReply(Json::Value &data)
  {
    messageReply = data;
    return true;
  }

protected:
  // bool getReply(Json::Value &msg);
  actionBase *cmdBase;
  bool needReply;
  Json::Value messageHolder;
  Json::Value messageReply;
};