 cmake_minimum_required(VERSION 3.5.1)
project(masi_main)
#set(CMAKE_BUILD_TYPE Release)

include_directories(include)

file(GLOB SOURCES "*.cpp")

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

if(APPLE)
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu11")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")
else()
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu11 -fms-extensions -Wl,--no-as-needed")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11 -fms-extensions -Wl,--no-as-needed")
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -lm")
endif()


#SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

add_executable(
masi_main 
src/masi_main.cpp
src/masi_uart.cpp
src/masi_proto2.cpp)


find_package(Threads REQUIRED)
if(THREADS_HAVE_PTHREAD_ARG)
SET_PROPERTY(TARGET masi_main PROPERTY COMPILE_OPTIONS "-pthread")
SET_PROPERTY(TARGET masi_main PROPERTY IMTERFACE_COMPLIE_OPTIONS "-pthread")

endif()
if(CMAKE_THREAD_LIBS_INIT)
target_link_libraries(masi_main "${CMAKE_THREAD_LIBS_INIT}")

endif()


