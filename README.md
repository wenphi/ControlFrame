基于消息的控制器框架
============================
> 目录
<!-- TOC -->

- [1. 依赖](#1-依赖)
- [2. 编译](#2-编译)
    - [2.1. 编译命令](#21-编译命令)
- [3. 数据包格式](#3-数据包格式)
    - [3.1. api发送命令](#31-api发送命令)
    - [3.2. api接收消息](#32-api接收消息)

<!-- /TOC -->

## 
# 1. 依赖
1. 当前框架需要zmq和jsoncpp,当前提供了zmq和jsoncpp的x86与arm的库文件,采用g++4.8编译如若需要其他版本库,下载如下代码自行编译替换即可
    - zmq消息框架,下载地址:https://github.com/zeromq/libzmq
    - jsoncpp,下载地址:https://github.com/open-source-parsers/jsoncpp

##
# 2. 编译
1. 控制器在不同的平台上，采用不同的编译配置

## 2.1. 编译命令
1. x86-linux
    - mkdir build ; cd build;
    - cmake .. 

2. x86-windows 
   
3. cross-arm-linux
    - mkdir build ; cd build ;
    - cmake .. -DCMAKE_TOOLCHAIN_FILE=../cross-arm-linux-gnueabihf.cmake 

##
# 3. 数据包格式
## 3.1. api发送命令
```json
{
    "msgType": 0,
    "module" : 0,
    "needReply":true,
    "msgData":
    {
        "cmd": 0,
        "params":
        {
            "param1": "string",
            "param2": 123,
            "...":
        }
    }
}
```
- msgType:0-1
    - 0表示该消息为命令消息
    - 1表示该消息为状态消息
-  module:0-n
    - 等于stateBase中定义的msgModule_t对应的枚举值
- needReply:true/false
    - true表示该消息需要返回
    - false表示该消息无需返回
- cmd
    - 对应模块的命令号
- params:参数
## 3.2 api接收消息
```json
{
    "replyMessage":
    {
        "result":,
        "reply":,
        "state":
    }
}
```


motion 
- kdl
    - 获取配置参数(轴的个数,dh参数,夹具信息等)
    - 机械臂正/逆解
- 算法库,可提供一些方法函数
    - 视觉标定等
- motion 
    - 获取配置参数
    - 添加点到点/直线/圆弧/样条
    - 配置夹具
    - 暂停/继续/停止
    - 同步轴的位置
    - 钩子函数(调用放回插补的关节值)
    - (待定:提前计算/)


