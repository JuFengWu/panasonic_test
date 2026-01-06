cmake_minimum_required(VERSION 3.10)
project(EtherCAT_Test)

set(CMAKE_CXX_STANDARD 17)
include_directories(${CMAKE_SOURCE_DIR}/soem)
include_directories(${CMAKE_SOURCE_DIR}/osal)
include_directories(${CMAKE_SOURCE_DIR}/osal/linux)
include_directories(${CMAKE_SOURCE_DIR}/oshw/linux)

# 加入你的程式碼
add_executable(ethercat_app ethercat_test_cst.cpp)

# 如果 libsoem.a 與 CMakeLists.txt 同資料夾：
target_link_libraries(ethercat_app ${CMAKE_SOURCE_DIR}/libsoem.a)

# 若編譯需要 pthread（SOEM 常用）
target_link_libraries(ethercat_app pthread)