#include "robotStateMechine/robotStateMechine.hpp"
#include "message/messageClient.hpp"
#include "message/messageServer.hpp"
#include <csignal>
#include <iostream>
#include <thread>
#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include <sys/time.h>

bool stopFlag = false;
moduleBase *robotStateMechine ::baseInstance = NULL;
std::string taddress = "/home/ywh";
//motion--addline
void testClient()
{
    messageClient *client0 = new messageClient(taddress, "Client0sendHook0");
    int count = 0;
    //发送
    Json::Value jsonData;
    char *recvmsg;
    std::string filter = "2";
    client0->setFilter(filter);
    client0->setFilter("6");
    while (!stopFlag)
    {

        jsonData = client0->subJson();
        if (!jsonData.isNull())
        {
            std::cout << "recv:\n"
                      << jsonData.toStyledString() << "\n"
                      << " message: " << jsonData["message"].asCString() << "\n"
                      << " data: " << jsonData["data"].asInt() << std::endl;
            //   sleep(1);
            count++;
        }
        if (count > 10)
            usleep(10000);
        else
            sleep(3);
    }
    delete client0;
    std::cout << "client send num:" << count << " exit done!" << std::endl;
}

void testServer()
{
    messageServer *msgServer = new messageServer(taddress);
    std::string repmsg;
    Json::Value jsonData;
    jsonData["message"] = "hello,i'm Server";
    int count = 0;
    int i = 0;
    char num[2];
    while (!stopFlag)
    {
        if (i >= 10)
            i = 0;
        snprintf(num, sizeof(num), "%d", i);
        repmsg = std::string(num);
        jsonData["data"] = count;
        i++;
        count++;
        msgServer->pubJson("2", jsonData);
        // sleep(1);
        usleep(10000);
    }
    delete msgServer;
    std::cout << "Server exit done!" << std::endl;
}

void stop(int sig)
{
    if (sig)
        stopFlag = true;
}

int main()
{
    std::vector<std::thread> threads;
    signal(SIGINT, stop);
    threads.emplace_back(testClient);
    threads.emplace_back(testServer);
    // threads.emplace_back(sendHook2);
    while (!stopFlag)
    {
        sleep(1);
    }
    for (auto &thr : threads)
        thr.join();

    std::cout << "all thread exit done!" << std::endl;
    std::cout << "term exit done!" << std::endl;
    return 0;
}