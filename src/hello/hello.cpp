#include "hello.hpp"

actionBase *hello::praseCmdtoAction(Json::Value jsonCmd)
{
    // std::vector<autoValue> params;
    // MsgToParams(jsonCmd, params);
    Json::Value root;
    actionBase *cmdbase_;
    switch (jsonCmd["cmd"].asInt())
    {
    case 0:
        cmdbase_ = new helloCmdWriteData;
        break;
    case 1:
        cmdbase_ = new helloCmdReadData;
        break;
    default:
        cmdbase_ = nullptr;
    }
    if (cmdbase_)
    {
        root = jsonCmd["params"];
        cmdbase_->setParams(root);
    }
    return cmdbase_;
};

//函数
void hello::writeData(int num)
{
    data = num;
}

int hello::readData()
{
    return data;
}