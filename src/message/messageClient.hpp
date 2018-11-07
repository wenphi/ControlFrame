#pragma once
#include "libjsoncpp/include/json.h"
#include "libzmq/include/zhelpers.h"
#include <iostream>

//客户段可以选择一次发送接收多条
class messageClient
{
  public:
    messageClient(std::string directory_, std::string id, int blockTime = 1000)
    {
        address_router = "ipc://" + directory_ + "/QxRobotRouter.ipc";
        address_pub = "ipc://" + directory_ + "/QxRobotPubSub.ipc";
        clientid = id;
        idDealer = id + std::string(".dealer");
        idReq = id + std::string(".req");
        std::cout << "debug:client:identify: " << idDealer << "||" << idReq << std::endl;
        std::cout << "debug::client:address_router: " << address_router << "\n"
                  << "address_pub:" << address_pub << std::endl;
        //创建环境
        context = zmq_ctx_new();
        //创建socket
        pSockDealer = zmq_socket(context, ZMQ_DEALER);
        pSockReq = zmq_socket(context, ZMQ_REQ);
        pSockSub = zmq_socket(context, ZMQ_SUB);
        //设置阻塞时间
        zmq_setsockopt(pSockDealer, ZMQ_SNDTIMEO, &blockTime, sizeof(blockTime));
        zmq_setsockopt(pSockDealer, ZMQ_RCVTIMEO, &blockTime, sizeof(blockTime));
        //设置退出时的等待时间--100ms
        int linger_time = 100;
        zmq_setsockopt(pSockDealer, ZMQ_LINGER, &linger_time, sizeof(linger_time));
        zmq_setsockopt(pSockReq, ZMQ_LINGER, &linger_time, sizeof(linger_time));
        zmq_setsockopt(pSockSub, ZMQ_SUB, &linger_time, sizeof(linger_time));
        // zmq_setsockopt(psock)
        //设置socket缓存的最大消息条数
        int sendHwm = 10;
        int recvHwm = 10;
        zmq_setsockopt(pSockDealer, ZMQ_SNDHWM, &sendHwm, sizeof(sendHwm));
        zmq_setsockopt(pSockDealer, ZMQ_RCVHWM, &recvHwm, sizeof(recvHwm));
        //设置id
        zmq_setsockopt(pSockDealer, ZMQ_IDENTITY, idDealer.c_str(), idDealer.size());
        zmq_setsockopt(pSockReq, ZMQ_IDENTITY, idReq.c_str(), idReq.size());
        //连接
        zmq_connect(pSockDealer, address_router.c_str());
        zmq_connect(pSockReq, address_router.c_str());
        zmq_connect(pSockSub, address_pub.c_str());
        //完成
        msgReady = true;
    };
    ~messageClient()
    {
        zmq_close(pSockDealer);
        zmq_close(pSockReq);
        zmq_close(pSockSub);
        zmq_ctx_destroy(context);
        std::cout << "client:" << clientid << " exit done!" << std::endl;
    };
    //初始化
    int setFilter(std::string filter_)
    {
        return zmq_setsockopt(pSockSub, ZMQ_SUBSCRIBE, filter_.c_str(), filter_.length());
    }
    int removeFilter(std::string filter_)
    {
        return zmq_setsockopt(pSockSub, ZMQ_UNSUBSCRIBE, filter_.c_str(), filter_.length());
    }
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
    //接收订阅消息
    Json::Value subJson()
    {
        Json::Reader reader;
        Json::Value jsonData;
        int data_pose;
        if (!msgReady)
            return jsonData;
        char *charData = s_recv(pSockSub, ZMQ_DONTWAIT);
        if (charData == NULL)
            return jsonData;
        // std::cout << "debug:sub data:" << charData << std::endl;
        data_pose = std::string(charData).find("_ID_End_");
        if (data_pose == std::string::npos)
        {
            std::cout << "recv data with out \'_ID_End_\'" << std::endl;
            return jsonData;
        }
        reader.parse((charData + data_pose + 8), jsonData);
        free(charData);
        return jsonData;
    }

  private:
    bool msgReady = false;

    std::string clientid;
    std::string idDealer; //zmq的sock标识
    std::string idReq;
    std::string address_pub;    //zmq链接的通讯地址
    std::string address_router; //zmq应答模式的通讯地址

    void *context;     //zmq的环境/上下文
    void *pSockDealer; //zmq创建Dealer的sock
    void *pSockReq;    //zmq创建Req的sock
    void *pSockSub;    //zmq创建sub的sock
    int blockTime;     //阻塞时间
};
