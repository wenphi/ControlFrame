#pragma once
#include <vector>
#include "libjsoncpp/include/json.h"

class actionBase
{
  public:
    actionBase() {}
    virtual ~actionBase() {}
    //子类调用
    void start()
    {
        run(jsonData);
    }

    bool setParams(Json::Value &jsonData_)
    {
        jsonData = jsonData_;
        return true;
    }

  private:
    virtual bool run(Json::Value &cmdData) = 0;
    Json::Value jsonData;
};