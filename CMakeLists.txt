cmake_minimum_required(VERSION 3.10)
project(EtherCAT_Test)

set(CMAKE_CXX_STANDARD 17)
include_directories(${CMAKE_SOURCE_DIR}/soem)
include_directories(${CMAKE_SOURCE_DIR}/osal)
include_directories(${CMAKE_SOURCE_DIR}/osal/linux)
include_directories(${CMAKE_SOURCE_DIR}/oshw/linux)

add_executable(test_run
    demo/test_run.cpp
    src/fakeInitializer.cpp
    src/panasonicA6.cpp
    src/fakeMotor.cpp
    src/motionSystem.cpp
    src/motionInitializer.cpp
    src/panasonicEthercatInitializer.cpp
    src/cyclicSession.cpp
    src/SoemEthercat.cpp
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
    src/SoemEthercat.cpp
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
    src/SoemEthercat.cpp
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
    src/SoemEthercat.cpp
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

# ---- Cppcheck static analysis ----
option(ENABLE_CPPCHECK "Enable cppcheck analysis target" ON)
if(ENABLE_CPPCHECK)
    find_program(CPPCHECK_EXE NAMES cppcheck)
    if(CPPCHECK_EXE)
        file(GLOB_RECURSE CPPCHECK_SOURCES CONFIGURE_DEPENDS
            ${CMAKE_SOURCE_DIR}/src/*.c
            ${CMAKE_SOURCE_DIR}/src/*.cpp
            ${CMAKE_SOURCE_DIR}/src/*.h
            ${CMAKE_SOURCE_DIR}/src/*.hpp
            ${CMAKE_SOURCE_DIR}/include/*.c
            ${CMAKE_SOURCE_DIR}/include/*.cpp
            ${CMAKE_SOURCE_DIR}/include/*.h
            ${CMAKE_SOURCE_DIR}/include/*.hpp
            ${CMAKE_SOURCE_DIR}/demo/*.c
            ${CMAKE_SOURCE_DIR}/demo/*.cpp
            ${CMAKE_SOURCE_DIR}/demo/*.h
            ${CMAKE_SOURCE_DIR}/demo/*.hpp
            ${CMAKE_SOURCE_DIR}/test/*.c
            ${CMAKE_SOURCE_DIR}/test/*.cpp
            ${CMAKE_SOURCE_DIR}/test/*.h
            ${CMAKE_SOURCE_DIR}/test/*.hpp
        )
        add_custom_target(cppcheck
            COMMAND ${CPPCHECK_EXE}
                --enable=warning,style,performance,portability
                --inline-suppr
                --language=c++
                --std=c++17
                -I ${CMAKE_SOURCE_DIR}/include
                -I ${CMAKE_SOURCE_DIR}/soem
                -I ${CMAKE_SOURCE_DIR}/osal
                -I ${CMAKE_SOURCE_DIR}/osal/linux
                -I ${CMAKE_SOURCE_DIR}/oshw/linux
                ${CPPCHECK_SOURCES}
            WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
            COMMENT "Running cppcheck on project sources"
        )
    else()
        message(WARNING "cppcheck not found: cppcheck target will be unavailable.")
    endif()
endif()
