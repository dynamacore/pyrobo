cmake_minimum_required(VERSION 3.16)
project(PYROBO)

set(SOURCE_DIR "csrc")

include_directories(${SOURCE_DIR})

add_executable(quat ${SOURCE_DIR}/quaternions.cpp)