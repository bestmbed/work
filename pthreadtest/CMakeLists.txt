CMAKE_MINIMUM_REQUIRED(VERSION 3.5.1)

project(pthreadT1)

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
    pthreadT1
    pthreadT1.cpp
)
	
add_executable(
    pthread2
    pthread2.cpp
)
find_package(Threads REQUIRED)
if(THREADS_HAVE_PTHREAD_ARG)
SET_PROPERTY(TARGET pthreadT1 PROPERTY COMPILE_OPTIONS "-pthread")
SET_PROPERTY(TARGET pthreadT1 PROPERTY IMTERFACE_COMPLIE_OPTIONS "-pthread")
SET_PROPERTY(TARGET pthread2 PROPERTY COMPILE_OPTIONS "-pthread")
SET_PROPERTY(TARGET pthread2 PROPERTY IMTERFACE_COMPLIE_OPTIONS "-pthread")
endif()
if(CMAKE_THREAD_LIBS_INIT)
target_link_libraries(pthreadT1 "${CMAKE_THREAD_LIBS_INIT}")
target_link_libraries(pthread2 "${CMAKE_THREAD_LIBS_INIT}")
endif()
