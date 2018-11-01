#pragma once
#include <iostream>
#include "../base/moduleBase.hpp"

class apiCmdOnMessage : public actionBase
{
    bool run(Json::Value &jsonData);
};

