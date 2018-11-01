include_directories(${CMAKE_HOME_DIRECTORY}/depend)
include_directories(${CMAKE_HOME_DIRECTORY}/src)

#添加实例
link_directories(${CMAKE_HOME_DIRECTORY}/depend/libjsoncpp/linux_os_x86)
link_directories(${CMAKE_HOME_DIRECTORY}/depend/libzmq/linux_os_x86)

add_executable(test1 test1.cpp)
target_link_libraries(test1 test_src jsoncpp zmq)


