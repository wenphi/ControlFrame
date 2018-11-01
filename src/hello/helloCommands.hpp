#pragma once
#include <iostream>
#include "../base/moduleBase.hpp"

class helloCmdWriteData : public actionBase
{
    bool run(Json::Value &jsonData);
};

class helloCmdReadData : public actionBase
{
    bool run(Json::Value &jsonData);
};
