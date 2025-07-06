cmake_minimum_required(VERSION 3.28)
project(PIDController)

set(CMAKE_CXX_STANDARD 17)

add_executable(PIDController main.cpp
        PIDController.cpp
        PIDController.h
        MathUtils.cpp
        MathUtils.h)
