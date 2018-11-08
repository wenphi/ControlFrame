#include "robotStateMechine/robotStateMechine.hpp"
#include "libjsoncpp/include/json.h"
#include <csignal>
#include <iostream>
#include <thread>
#include <cstdlib>
#include <cstring>
#include <unistd.h>

bool stopFlag = false;
moduleBase *robotStateMechine ::baseInstance = nullptr;
std::string taddress = "/home/ywh";

void stop(int sig)
{
    if (sig)
        stopFlag = true;
}

void hook(robotStateMechine *rsm)
{
    while (!stopFlag)
    {
        Json::Value jsonData;
        jsonData["msgType"] = msgtype_t::MSG_TYPE_CMD;
        jsonData["module"] = msgModule_t::MSG_MODULE_HELLO;
        jsonData["needReply"] = true;
        jsonData["msgData"]["cmd"] = 1;
        jsonData = rsm->stateProcessing(jsonData);
        int num = jsonData["replyMessage"]["param1"].asInt();
        std::cout << "in hook num:" << num << std::endl;
        sleep(1);
    }
    std::cout << "hook exit done!" << std::endl;
}

int main()
{
    std::vector<std::thread> threads;
    signal(SIGINT, stop);

    robotStateMechine *rsm = new robotStateMechine();
    //创建
    messageServer *msgServer = new messageServer(taddress);
    hello *ptrHello = new hello();
    //注册
    rsm->registerMessageServer(msgServer);
    rsm->registerHello(ptrHello);
    rsm->setStart();
    //开启线程
    threads.emplace_back(hook, rsm);
    while (!stopFlag)
        rsm->updateHook();

    rsm->setStop();
    for (auto &thr : threads)
        thr.join();
    std::cout << "main() thread exit done!" << std::endl;
    delete rsm;
    delete msgServer;
    delete ptrHello;
    std::cout << "terminal exit done!" << std::endl;
    return 0;
}