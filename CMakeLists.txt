cmake_minimum_required(VERSION 3.10)
if(POLICY CMP0148)
  cmake_policy(SET CMP0148 NEW)
endif()
project(EtherCAT_Test)

set(CMAKE_CXX_STANDARD 17)
option(BUILD_PYTHON "Build Python module bindings" ON)
include_directories(${CMAKE_SOURCE_DIR}/soem)
include_directories(${CMAKE_SOURCE_DIR}/osal)
include_directories(${CMAKE_SOURCE_DIR}/osal/linux)
include_directories(${CMAKE_SOURCE_DIR}/oshw/linux)

add_library(soem_pic STATIC
  soem/ethercatbase.c
  soem/ethercatcoe.c
  soem/ethercatconfig.c
  soem/ethercatdc.c
  soem/ethercateoe.c
  soem/ethercatfoe.c
  soem/ethercatmain.c
  soem/ethercatprint.c
  soem/ethercatsoe.c
  osal/linux/osal.c
  oshw/linux/nicdrv.c
  oshw/linux/oshw.c
)
set_target_properties(soem_pic PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(test_run
    demo/test_run.cpp
    src/fakeInitializer.cpp
    src/panasonicA6.cpp
    src/fakeMotor.cpp
    src/motionSystem.cpp
    src/motionInitializer.cpp
    src/panasonicEthercatInitializer.cpp
    src/cyclicSession.cpp
)
target_include_directories(test_run PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(test_run ${CMAKE_SOURCE_DIR}/libsoem.a)

add_executable(beginner_demo
    demo/beginner_demo.cpp
    src/fakeInitializer.cpp
    src/panasonicA6.cpp
    src/fakeMotor.cpp
    src/motionSystem.cpp
    src/motionInitializer.cpp
    src/panasonicEthercatInitializer.cpp
    src/cyclicSession.cpp
)
target_include_directories(beginner_demo PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(beginner_demo ${CMAKE_SOURCE_DIR}/libsoem.a)

add_executable(test_run_sync
    demo/test_run_sync.cpp
    src/fakeInitializer.cpp
    src/panasonicA6.cpp
    src/fakeMotor.cpp
    src/motionSystem.cpp
    src/motionInitializer.cpp
    src/panasonicEthercatInitializer.cpp
    src/cyclicSession.cpp
)
target_include_directories(test_run_sync PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(test_run_sync ${CMAKE_SOURCE_DIR}/libsoem.a)

add_executable(get_slave_count
    demo/get_slave_count.cpp
    src/fakeInitializer.cpp
    src/panasonicA6.cpp
    src/fakeMotor.cpp
    src/motionSystem.cpp
    src/motionInitializer.cpp
    src/panasonicEthercatInitializer.cpp
    src/cyclicSession.cpp
)
target_include_directories(get_slave_count PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(get_slave_count ${CMAKE_SOURCE_DIR}/libsoem.a)


add_executable(pp_mode ethercat_test_pp_mode_ok.cpp)
target_include_directories(pp_mode PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(pp_mode ${CMAKE_SOURCE_DIR}/libsoem.a)
target_link_libraries(pp_mode pthread)


add_executable(csv_mode ethercat_test_csv.cpp)
target_include_directories(csv_mode PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(csv_mode ${CMAKE_SOURCE_DIR}/libsoem.a)
target_link_libraries(csv_mode pthread)


add_executable(csp_mode ethercat_test_csp_ok.cpp)
target_include_directories(csp_mode PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(csp_mode ${CMAKE_SOURCE_DIR}/libsoem.a)
target_link_libraries(csp_mode pthread)


add_executable(cst_mode ethercat_test_cst.cpp)
target_include_directories(cst_mode PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(cst_mode ${CMAKE_SOURCE_DIR}/libsoem.a)
target_link_libraries(cst_mode pthread)


add_executable(test_rt test/rt_test.cpp)

if (BUILD_PYTHON)
  include(FetchContent)
  FetchContent_Declare(
    pybind11
    GIT_REPOSITORY https://github.com/pybind/pybind11.git
    GIT_TAG v2.11.1
  )
  FetchContent_MakeAvailable(pybind11)

  pybind11_add_module(motorcode
    bindings/pybindings.cpp
    src/fakeInitializer.cpp
    src/panasonicA6.cpp
    src/fakeMotor.cpp
    src/motionSystem.cpp
    src/motionInitializer.cpp
    src/panasonicEthercatInitializer.cpp
    src/cyclicSession.cpp
  )
  target_include_directories(motorcode PRIVATE ${CMAKE_SOURCE_DIR}/include)
  target_link_libraries(motorcode PRIVATE soem_pic pthread)
endif ()

