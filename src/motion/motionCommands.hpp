#pragma once
#include <iostream>
#include "../base/moduleBase.hpp"

namespace MOTION
{
class motionCmdAddLine : public actionBase
{
    bool run(Json::Value &jsonData);
};

} // namespace MOTION
