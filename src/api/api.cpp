#include "api.hpp"

void api::setHello(int num)
{

    Json::Value jsonData;
    jsonData["msgType"] = msgtype_t::MSG_CMD;
    jsonData["module"] = msgModule_t::MSG_MODULE_HELLO;
    jsonData["needReply"] = true;
    jsonData["msgData"]["cmd"] = 0;
    jsonData["msgData"]["params"]["param1"] = num;
    msgClient->sendJson(jsonData);
    // std::cout << msgClient->recvJson().toStyledString() << std::endl;
}

int api::readHello()
{
    std::lock_guard<std::mutex> lock(api_block_mutex);
    Json::Value jsonData;
    jsonData["msgType"] = msgtype_t::MSG_CMD;
    jsonData["module"] = msgModule_t::MSG_MODULE_HELLO;
    jsonData["needReply"] = true;
    jsonData["msgData"]["cmd"] = 1;
    msgClient->sendJson_block(jsonData);
    jsonData = msgClient->recvJson_block();
    int num = jsonData["replyMessage"]["reply"]["param1"].asInt();
    return num;
}