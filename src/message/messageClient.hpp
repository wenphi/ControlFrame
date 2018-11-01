#pragma once
#include "libjsoncpp/include/json.h"
#include "libzmq/include/zhelpers.h"
#include <iostream>

//客户段可以选择一次发送接收多条
class messageClient
{
  public:
    messageClient(std::string address_, std::string id, int blockTime = 1000)
    {
        address = address_;
        clientid = id;
        idDealer = id + std::string(".dealer");
        idReq = id + std::string(".req");
        std::cout << "debug:client:identify: " << idDealer << "||" << idReq << std::endl;
        std::cout << "debug::client:address: " << address << std::endl;
        //创建环境
        context = zmq_ctx_new();
        //创建socket
        pSockDealer = zmq_socket(context, ZMQ_DEALER);
        pSockReq = zmq_socket(context, ZMQ_REQ);
        //设置阻塞时间
        zmq_setsockopt(pSockDealer, ZMQ_SNDTIMEO, &blockTime, sizeof(blockTime));
        zmq_setsockopt(pSockDealer, ZMQ_RCVTIMEO, &blockTime, sizeof(blockTime));
        //设置退出时的等待时间--1000ms
        int linger_time = 1000;
        zmq_setsockopt(pSockDealer, ZMQ_LINGER, &linger_time, sizeof(linger_time));
        zmq_setsockopt(pSockReq, ZMQ_LINGER, &linger_time, sizeof(linger_time));
        //设置socket缓存的最大消息条数
        int sendHwm = 10;
        int recvHwm=10;
        zmq_setsockopt(pSockDealer, ZMQ_SNDHWM, &sendHwm, sizeof(sendHwm));
        zmq_setsockopt(pSockDealer, ZMQ_RCVHWM, &recvHwm, sizeof(recvHwm));
        //设置id
        zmq_setsockopt(pSockDealer, ZMQ_IDENTITY, idDealer.c_str(), idDealer.size());
        zmq_setsockopt(pSockReq, ZMQ_IDENTITY, idReq.c_str(), idReq.size());
        //连接
        zmq_connect(pSockDealer, address.c_str());
        zmq_connect(pSockReq, address.c_str());
        //完成
        msgReady = true;
    };
    messageClient(std::string address_)
    {
        char identity[10];
        snprintf(identity, sizeof(identity), "%04X-%04X", within(0x10000), within(0x10000));
        clientid = std::string(identity);
        idDealer = std::string(identity) + std::string(".dealer");
        idReq = std::string(identity) + std::string(".req");
        std::cout << "debug:client:address:" << idDealer << "||" << idReq << std::endl;
        address = address_;
        //创建环境
        context = zmq_ctx_new();
        //创建socket
        pSockDealer = zmq_socket(context, ZMQ_DEALER);
        pSockReq = zmq_socket(context, ZMQ_REQ);
        //设置阻塞时间
        int blockTime = 1000;
        zmq_setsockopt(pSockDealer, ZMQ_SNDTIMEO, &blockTime, sizeof(blockTime));
        zmq_setsockopt(pSockDealer, ZMQ_RCVTIMEO, &blockTime, sizeof(blockTime));
        //配置id
        zmq_setsockopt(pSockDealer, ZMQ_IDENTITY, &idDealer, sizeof(idDealer));
        zmq_setsockopt(pSockReq, ZMQ_IDENTITY, &idReq, sizeof(idReq));
        //设置socket退出时的阻塞时间--1000ms
        int linger_time = 0;
        zmq_setsockopt(pSockDealer, ZMQ_LINGER, &linger_time, sizeof(linger_time));
        zmq_setsockopt(pSockReq, ZMQ_LINGER, &linger_time, sizeof(linger_time));
        //连接
        zmq_connect(pSockDealer, address.c_str());
        zmq_connect(pSockReq, address.c_str());
        //完成
        msgReady = true;
    };
    ~messageClient()
    {
        zmq_close(pSockDealer);
        zmq_close(pSockReq);
        zmq_ctx_destroy(context);
        // std::cout << "client:" << clientid << " exit done!" << std::endl;
    };
    //初始化

    //发送json格式的消息
    int sendJson(Json::Value &jsonData)
    {
        return s_send(pSockDealer, const_cast<char *>(jsonData.toStyledString().c_str()));
    };
    int sendJson_block(Json::Value &JsonData)
    {
        return s_send(pSockReq, const_cast<char *>(JsonData.toStyledString().c_str()));
    };
    //发送char格式的消息
    int sendChar(char *charData, int len)
    {
        return zmq_send(pSockDealer, charData, len, 0);
    };
    int sendChar_block(char *charData, int len)
    {
        return zmq_send(pSockReq, charData, len, 0);
    }
    //接收json格式的消息
    Json::Value recvJson()
    {
        Json::Reader reader;
        Json::Value jsonData;
        if (!msgReady)
            return jsonData;
        char *charData = s_recv(pSockDealer);
        if (charData == NULL)
            return jsonData;
        reader.parse(charData, jsonData);
        free(charData);
        return jsonData;
    };
    Json::Value recvJson_block()
    {
        Json::Reader reader;
        Json::Value jsonData;
        if (!msgReady)
            return jsonData;
        char *charData = s_recv(pSockReq);
        if (charData == NULL)
            return jsonData;
        reader.parse(charData, jsonData);
        free(charData);
        return jsonData;
    };
    //接收char*格式的消息,消息使用后记得释放!!
    char *recvChar()
    {
        return s_recv(pSockDealer);
    };
    char *recvChar_block()
    {
        return s_recv(pSockReq);
    }

  private:
    bool msgReady = false;

    std::string clientid;
    std::string idDealer; //zmq的sock标识
    std::string idReq;
    std::string address; //zmq链接的通讯地址

    void *context;     //zmq的环境/上下文
    void *pSockDealer; //zmq创建Dealer的sock
    void *pSockReq;    //zmq创建Req的sock
    int blockTime;     //阻塞时间
};
