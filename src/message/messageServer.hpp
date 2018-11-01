#pragma once
#include "libjsoncpp/include/json.h"
#include "libzmq/include/zhelpers.h"
#include <iostream>

//一次只允许执行一条命令,否则导致发送地址出错
class messageServer
{
  public:
    messageServer(std::string address_)
    {
        address = address_;
        //发送--接收阻塞时间/ms
        rblockTime = 1000;
        sblockTime = 1000;
        //创建环境
        context = zmq_ctx_new();
        //创建socket
        pSockRouter = zmq_socket(context, ZMQ_ROUTER);
        //设置吞吐率
        int rate = 1024 * 1024;
        zmq_setsockopt(pSockRouter, ZMQ_RATE, &rate, sizeof(rate));
        //设置阻塞时间
        zmq_setsockopt(pSockRouter, ZMQ_SNDTIMEO, &sblockTime, sizeof(sblockTime));
        zmq_setsockopt(pSockRouter, ZMQ_RCVTIMEO, &rblockTime, sizeof(rblockTime));
        //设置socket退出时的阻塞时间--1000ms
        int linger_time = 10;
        zmq_setsockopt(pSockRouter, ZMQ_LINGER, &linger_time, sizeof(linger_time));
        //设置socket缓存的最大消息条数
        int recvHwm = 10;
        zmq_setsockopt(pSockRouter, ZMQ_RCVHWM, &recvHwm, sizeof(recvHwm));
        //绑定
        zmq_bind(pSockRouter, address.c_str());
        //完成
        msgReady = true;
    };
    ~messageServer()
    {
        zmq_close(pSockRouter);
        zmq_ctx_destroy(context);
        std::cout << "server:"
                  << "exit done!" << std::endl;
    };
    //初始化

    //发送
    int sendJson(Json::Value &jsonData)
    {
        int ret1, ret2, ret3;
        ret1 = s_sendmore(pSockRouter, const_cast<char *>(address.c_str()));
        if (ret1 < 0)
            return ret1;

        if (isBlock)
        {
            ret2 = s_sendmore(pSockRouter, (char *)"");
            if (ret2 < 0)
                return ret2;
        }
        ret3 = s_send(pSockRouter, const_cast<char *>(jsonData.toStyledString().c_str()));
        return ret3;
    };
    int sendChar(char *charData, int len)
    {
        int ret1, ret2, ret3;
        ret1 = s_sendmore(pSockRouter, const_cast<char *>(address.c_str()));
        if (ret1 < 0)
            return ret1;

        if (isBlock)
        {
            ret2 = s_sendmore(pSockRouter, (char *)"");
            if (ret2 < 0)
                return ret2;
        }
        ret3 = zmq_send(pSockRouter, charData, len, 0);
        return ret3;
    };
    //接收
    Json::Value recvJson()
    {
        Json::Reader reader;
        Json::Value jsonData;
        if (!msgReady)
            return jsonData;
        char *msgAddr = s_recv(pSockRouter);
        if (msgAddr == NULL)
            return jsonData;
        address = std::string(msgAddr); //保存地址
        free(msgAddr);
        char *charData = s_recv(pSockRouter);
        if (strcmp(charData, "") == 0)
        {
            isBlock = true;
            free(charData);
            charData = s_recv(pSockRouter);
        }
        else
            isBlock = false;
        reader.parse(charData, jsonData);
        free(charData);
        return jsonData;
    };
    //错误返回空指针,消息使用后记得释放!!
    char *recvChar()
    {
        char *charData = NULL;
        if (!msgReady)
            return charData;
        char *msgAddr = s_recv(pSockRouter);
        if (msgAddr == NULL)
            return charData;
        address = std::string(msgAddr); //保存地址
        free(msgAddr);
        charData = s_recv(pSockRouter);
        if (strcmp(charData, "") == 0)
        {
            isBlock = true;
            free(charData);
            charData = s_recv(pSockRouter);
        }
        else
            isBlock = false;
        return charData;
    };

  private:
    bool msgReady = false;
    bool isBlock;         //每次调用recv后刷新,决定了发送类型{地址+""+数据}/{地址+数据}
    std::string clientid; //每次调用recv后刷新,决定了下一次send的发送地址;
    std::string address;  //zmq链接的通讯地址
    void *context;        //zmq的环境/上下文
    void *pSockRouter;    //zmq创建Router的sock
    int rblockTime;       //接收阻塞时间
    int sblockTime;       //发送阻塞时间
};