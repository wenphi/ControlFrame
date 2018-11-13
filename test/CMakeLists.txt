include_directories(${CMAKE_HOME_DIRECTORY}/depend)
include_directories(${CMAKE_HOME_DIRECTORY}/src)

#添加实例
link_directories(${CMAKE_HOME_DIRECTORY}/depend/libjsoncpp/linux_os_x86)
link_directories(${CMAKE_HOME_DIRECTORY}/depend/libzmq/linux_os_x86)
link_directories(${CMAKE_HOME_DIRECTORY}/depend/liborocosKdl/linux_os_x86)

add_executable(test1 test1.cpp)
target_link_libraries(test1 test_src jsoncpp zmq)

add_executable(testPubSub testPublisher.cpp)
target_link_libraries(testPubSub test_src jsoncpp zmq)

add_executable(testzmq testzmq.cpp)
target_link_libraries(testzmq zmq test_src jsoncpp)

add_executable(testMotion testMotion.cpp)
target_link_libraries(testMotion zmq test_src jsoncpp orocos-kdl)


