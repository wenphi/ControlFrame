#pragma once
#include <mutex>
#include "libjsoncpp/include/json.h"
#include "../message/messageClient.hpp"
#include "../base/stateBase.hpp"
#include "../base/moduleBase.hpp"

class api : public moduleBase
{
public:
  api(std::string address_, std::string name_)
  {
    std::string name = name_ + ".api";
    msgClient = new messageClient(address_, name);
  };
  ~api()
  {
    delete msgClient;
  };

  actionBase *praseCmdtoAction(Json::Value jsonCmd) { return nullptr; }

  void setHello(int num);
  int readHello();

private:
  messageClient *msgClient;
  std::mutex api_block_mutex;
};