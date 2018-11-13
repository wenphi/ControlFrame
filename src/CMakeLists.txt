include_directories(${CMAKE_HOME_DIRECTORY}/depend)

aux_source_directory(${CMAKE_CURRENT_LIST_DIR}/base SRC_BASE)
aux_source_directory(${CMAKE_CURRENT_LIST_DIR}/api SRC_API)
aux_source_directory(${CMAKE_CURRENT_LIST_DIR}/message SRC_MSG)
aux_source_directory(${CMAKE_CURRENT_LIST_DIR}/robotStateMechine SRC_RSB)
aux_source_directory(${CMAKE_CURRENT_LIST_DIR}/hello SRC_HELLO)
#motion
aux_source_directory(${CMAKE_CURRENT_LIST_DIR}/motion/arithmetic SRC_MOTION2)
aux_source_directory(${CMAKE_CURRENT_LIST_DIR}/motion/kdl SRC_MOTION3)
aux_source_directory(${CMAKE_CURRENT_LIST_DIR}/motion/tp SRC_MOTION4)
aux_source_directory(${CMAKE_CURRENT_LIST_DIR}/motion SRC_MOTION1)
set(SRC_MOTION ${SRC_MOTION1} ${SRC_MOTION2} ${SRC_MOTION3} ${SRC_MOTION4})

set(SRC_ALL  ${SRC_BASE} ${SRC_API} ${SRC_RSB} ${SRC_MSG} ${SRC_HELLO} ${SRC_MOTION})
add_library(test_src STATIC ${SRC_ALL})