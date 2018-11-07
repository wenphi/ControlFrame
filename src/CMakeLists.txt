include_directories(${CMAKE_HOME_DIRECTORY}/depend)

aux_source_directory(${CMAKE_CURRENT_LIST_DIR}/base SRC_BASE)
aux_source_directory(${CMAKE_CURRENT_LIST_DIR}/api SRC_API)
aux_source_directory(${CMAKE_CURRENT_LIST_DIR}/message SRC_MSG)
aux_source_directory(${CMAKE_CURRENT_LIST_DIR}/robotStateMechine SRC_RSB)
aux_source_directory(${CMAKE_CURRENT_LIST_DIR}/hello SRC_HELLO)

set(SRC_ALL  ${SRC_BASE} ${SRC_API} ${SRC_RSB} ${SRC_MSG} ${SRC_HELLO})
add_library(test_src STATIC ${SRC_ALL})
