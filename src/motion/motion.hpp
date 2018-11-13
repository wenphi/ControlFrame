#pragma once
#include "../base/moduleBase.hpp"
#include "tp/tcq.hpp"
#include "motionCommands.hpp"
#include "motionType.hpp"
namespace MOTION
{

class motion : public moduleBase
{
public:
  motion()
  {
    motionInit();
  };
  ~motion(){};

  //解析返回子类指针--重载
  actionBase *praseCmdtoAction(Json::Value jsonCmd);

public:
  void motionHook();
  bool motionAddLine();
  void writeData(int num);
  int readData();

private:
  void motionInit();

private:
  int data = 0;
  TrajectoryCalcQueue tcqQueue;
  TrajectoryCalc tcQueue[100];
};
} // namespace MOTION
