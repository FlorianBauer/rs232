cmake_minimum_required(VERSION 3.10)
project("rs232")

# Set C++11 as standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# General compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

add_executable("demo_tx" "DemoTx.cpp" "../Rs232.cpp")
add_executable("demo_rx" "DemoRx.cpp" "../Rs232.cpp")

