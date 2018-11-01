基于消息的控制器框架
============================
> 目录
<!-- TOC -->

- [1. 依赖](#1-依赖)
- [2. 编译](#2-编译)
    - [2.1. 编译命令](#21-编译命令)

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
    


