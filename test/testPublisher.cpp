#include "libzmq/include/zhelpers.h"
#include "api/api.hpp"
#include "libjsoncpp/include/json.h"
#include <csignal>
#include <iostream>
#include <thread>
#include <cstdlib>
#include <cstring>
#include <unistd.h>

#define CLIENT_NUM 10
bool stopFlag = false;
std::string taddress = "ipc://pub_sub.ipc";

enum FILTER_ENUM
{
    WEB = 0001,
    SOCK = 0002,
};

void stop(int sig)
{
    if (sig)
        stopFlag = true;
}

void testSub(int num)
{
    char cmd[0];
    sprintf(cmd, "%d", num);
    std::string filter = std::string(cmd);
    // std::string filter = "";
    void *pCtx = zmq_ctx_new();
    void *pSockSub = zmq_socket(pCtx, ZMQ_SUB);
    zmq_setsockopt(pSockSub, ZMQ_SUBSCRIBE, filter.c_str(), filter.length());
    // zmq_setsockopt(pSockSub, ZMQ_SUBSCRIBE, "", 0);
    int lingerTime = 10;
    zmq_setsockopt(pSockSub, ZMQ_LINGER, &lingerTime, sizeof(lingerTime));
    zmq_connect(pSockSub, taddress.c_str());
    std::size_t pose;
    int count = 0;
    Json::Value jsonData;
    Json::Reader reader;
    sleep(1);
    while (!stopFlag)
    {
        char *recvChar1 = s_recv(pSockSub, ZMQ_DONTWAIT);
        if (recvChar1 != NULL)
        {
            pose = std::string(recvChar1).find("_ID_End_");
            std::cout << "pose: " << pose << std::endl;
            if (pose == std::string::npos)
            {
                std::cout << "without \'_ID_End_\',recv is:\n"
                          << recvChar1 << std::endl;
                continue;
            }

            if (reader.parse((recvChar1 + pose + 8), jsonData))
            {
                std::cout << " recv_temp1:" << filter
                          << " recv_temp2:" << std::endl
                          << jsonData["message"].asCString() << "\n"
                          << jsonData["data"].asInt() << "\n"
                          << std::endl;
            }
            else
                std::cout << "parse to json failed:\n"
                          << recvChar1 + pose + 8 << std::endl;
            free(recvChar1);
            count++;
            // sleep(jsonData["data"].asInt());
        }
        usleep(1000);
    }
    std::cout << "testSub" << num << "recv num:" << count << " exit done!" << std::endl;
    zmq_close(pSockSub);
    zmq_ctx_destroy(pCtx);
}

int main()
{
    std::vector<std::thread> threads;
    signal(SIGINT, stop);
    void *pCtx = zmq_ctx_new();
    void *pSockPub = zmq_socket(pCtx, ZMQ_PUB);
    int sendHwm = 10;
    zmq_setsockopt(pSockPub, ZMQ_SNDHWM, &sendHwm, sizeof(sendHwm));
    int lingerTime = 10;
    zmq_setsockopt(pSockPub, ZMQ_LINGER, &lingerTime, sizeof(lingerTime));
    zmq_bind(pSockPub, taddress.c_str());
    for (int i = 0; i < CLIENT_NUM; i++)
    {
        threads.emplace_back(testSub, i);
    }
    char num[10];
    char identify[10];
    std::string str = "0123456789";
    snprintf(identify, sizeof(identify) + 1, "%s", str.c_str());
    std::cout << "test:" << sizeof(identify) << " | " << identify << " | " << std::string(identify) << std::endl;
    std::string clientName;
    std::string test;
    std::string testmore;
    int i = 0;
    Json::Value jsonData;
    jsonData["message"] = "in test clinet";
    int count = 0;
    while (!stopFlag)
    {
        if (i >= CLIENT_NUM)
            i = 0;
        jsonData["data"] = i;
        snprintf(num, sizeof(num), "%d", i);
        clientName = std::string(num);
        test = clientName + "_ID_End_" + jsonData.toStyledString();
        i++;
        if (s_send(pSockPub, const_cast<char *>(test.c_str())) < 0)
        {
            std::cout << "send data failed!" << std::endl;
            continue;
        }

        count++;
        usleep(1000000);
    }
    std::cout << "pub send num:" << count << std::endl;
    zmq_close(pSockPub);
    zmq_ctx_destroy(pCtx);
    for (auto &ths : threads)
        ths.join();

    return 0;
}