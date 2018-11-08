#include "helloCommands.hpp"
#include "hello.hpp"
#include "../robotStateMechine/robotStateMechine.hpp"

bool helloCmdWriteData::run(Json::Value &jsonData)
{
    hello *helloptr;

    //指向hello模块
    helloptr = dynamic_cast<hello *>(robotStateMechine::getModuleBasePtr());
    //调用hello中的方法
    helloptr->writeData(jsonData["param1"].asInt());
    //设置返回值
    Json::Value jsonReply;
    jsonReply["resultMsg"]["param1"] = jsonData["param1"].asInt();
    helloptr->setReply(jsonReply);
    return true;
};
bool helloCmdReadData::run(Json::Value &jsonData)
{
    hello *helloptr;
    helloptr = dynamic_cast<hello *>(robotStateMechine::getModuleBasePtr());
    int data = helloptr->readData();
    //设置返回值
    Json::Value jsonReply;
    jsonReply["resultMsg"]["param1"] = data;
    jsonReply["state"] = 0;
    helloptr->setReply(jsonReply);
    return true;
}