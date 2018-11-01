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
std::string taddress = "ipc:///home/ywh/stateMechine.ipc";

void stop(int sig)
{
    if (sig)
        stopFlag = true;
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
    threads.emplace_back(&robotStateMechine::updateHook, rsm);
    rsm->setStart();
    while (!stopFlag)
        sleep(1);

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