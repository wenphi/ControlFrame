#pragma once
#include "../base/moduleBase.hpp"
#include "helloCommands.hpp"

class hello : public moduleBase
{
public:
  hello(){};
  ~hello(){};

  //解析返回子类指针--重载
  actionBase *praseCmdtoAction(Json::Value jsonCmd);

  /*其他函数*/
  void writeData(int num);
  int readData();

private:
  int data = 0;
};