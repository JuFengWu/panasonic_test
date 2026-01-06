cmake_minimum_required(VERSION 3.10)
project(EtherCAT_Test)

set(CMAKE_CXX_STANDARD 17)
include_directories(${CMAKE_SOURCE_DIR}/soem)
include_directories(${CMAKE_SOURCE_DIR}/osal)
include_directories(${CMAKE_SOURCE_DIR}/osal/linux)
include_directories(${CMAKE_SOURCE_DIR}/oshw/linux)

# 加入你的程式碼
add_executable(pp_mode ethercat_test_pp_mode_ok.cpp)

target_include_directories(pp_mode PRIVATE ${CMAKE_SOURCE_DIR}/include)

add_executable(test_pp_mode src/test_pp_mode.cpp src/panasonicA6B.cpp)
target_include_directories(test_pp_mode PRIVATE ${CMAKE_SOURCE_DIR}/include)

# 如果 libsoem.a 與 CMakeLists.txt 同資料夾：
target_link_libraries(pp_mode ${CMAKE_SOURCE_DIR}/libsoem.a)

# 若編譯需要 pthread（SOEM 常用）
target_link_libraries(pp_mode pthread)


# 加入你的程式碼
add_executable(csv_mode ethercat_test_csv.cpp)

target_include_directories(csv_mode PRIVATE ${CMAKE_SOURCE_DIR}/include)
# 如果 libsoem.a 與 CMakeLists.txt 同資料夾：
target_link_libraries(csv_mode ${CMAKE_SOURCE_DIR}/libsoem.a)

# 若編譯需要 pthread（SOEM 常用）
target_link_libraries(csv_mode pthread)


# 加入你的程式碼
add_executable(csp_mode ethercat_test_csp_ok.cpp)
target_include_directories(csp_mode PRIVATE ${CMAKE_SOURCE_DIR}/include)
# 如果 libsoem.a 與 CMakeLists.txt 同資料夾：
target_link_libraries(csp_mode ${CMAKE_SOURCE_DIR}/libsoem.a)

# 若編譯需要 pthread（SOEM 常用）
target_link_libraries(csp_mode pthread)


# 加入你的程式碼
add_executable(cst_mode ethercat_test_cst.cpp)
target_include_directories(cst_mode PRIVATE ${CMAKE_SOURCE_DIR}/include)
# 如果 libsoem.a 與 CMakeLists.txt 同資料夾：
target_link_libraries(cst_mode ${CMAKE_SOURCE_DIR}/libsoem.a)

# 若編譯需要 pthread（SOEM 常用）
target_link_libraries(cst_mode pthread)
