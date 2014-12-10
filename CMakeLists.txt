#   MessyBsp. BSP collision and loading example code.
#   Copyright (C) 2014 Richard Maxwell <jodi.the.tigger@gmail.com>
#   This file is part of MessyBsp
#   MessyBsp is free software: you can redistribute it and/or modify
#   it under the terms of the GNU Affero General Public License as published by
#   the Free Software Foundation, either version 3 of the License, or
#   (at your option) any later version.
#   This program is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
#   GNU Affero General Public License for more details.
#   You should have received a copy of the GNU Affero General Public License
#   along with this program. If not, see <http://www.gnu.org/licenses/>

###############
# Project info
###############
# TODO: When cmake 3.1 gets released, use the commented out code instead.
project(MessyBsp)

#cmake_minimum_required(VERSION 3.1.0 FATAL_ERROR)
cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

###############
# Compiler Info
###############
include(CheckCXXCompilerFlag)

# Check compiler options
check_cxx_compiler_flag("-std=c++14" HAS_CPP14)
check_cxx_compiler_flag("-march=native" HAS_NATIVE)
check_cxx_compiler_flag("-fno-rtti" CAN_DISABLE_RTTI)
check_cxx_compiler_flag("-fno-exceptions" CAN_DISABLE_EXCEPTIONS)
check_cxx_compiler_flag(-Wall HAS_WALL)
check_cxx_compiler_flag(-W4 HAS_W4)
check_cxx_compiler_flag(-Wextra HAS_WEXTRA)
check_cxx_compiler_flag(-Werror HAS_WERROR)
check_cxx_compiler_flag(-WX HAS_WERROR_MSVC)
check_cxx_compiler_flag("/MP" HAS_MULTIPROCESSOR_COMPILE)

# Only platforms with clangs libc++ will build with clang with these build flags.
# Currently, that's only OSX.
set(TEMP_STACK ${CMAKE_REQUIRED_FLAGS})
set(CMAKE_REQUIRED_FLAGS "-std=c++14 -stdlib=libc++")
check_cxx_source_compiles("
#include <cstdint>
int main(int, int)
{
return 0;
}
" HAS_CPP14_AND_LIB)
set(CMAKE_REQUIRED_FLAGS ${TEMP_STACK})
unset(TEMP_STACK)

###############
# Prerequisites
###############
if (HAS_CPP14_AND_LIB)
    message("Using HAS_CPP14_AND_LIB.")
    set(CMAKE_CXX_FLAGS "-std=c++14 -stdlib=libc++")
elseif (HAS_CPP14)
    set(CMAKE_CXX_FLAGS "-std=c++14")
elseif(NOT MSVC14)
    message(FATAL_ERROR "Compiler requires C++14 functionality (MSVC14).")
endif()

if (HAS_NATIVE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
endif()

if (CAN_DISABLE_RTTI)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-rtti")
endif()

if (CAN_DISABLE_EXCEPTIONS)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-exceptions")
endif()

# prefer W4 over WALL for MSVC at least.
if (HAS_W4)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -W4")
elseif(HAS_WALL)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
endif()

if (HAS_WEXTRA)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wextra")
endif()

if (HAS_WERROR)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror")
endif()

if (HAS_WERROR_MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -WX")
endif()

if (HAS_MULTIPROCESSOR_COMPILE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
endif()

# Set a default build type for single-configuration
# CMake generators if no build type is set.
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

## C++11/14 features I use. (CMake 3.1)
#set(needed_features

#    cxx_generic_lambdas
#    cxx_auto_type
#    cxx_uniform_initialization
#    cxx_trailing_return_types
#    cxx_right_angle_brackets
#    cxx_return_type_deduction
#    cxx_nullptr
#    cxx_alignas)

#target_compile_features(MessyBsp PRIVATE ${needed_features})

message("CXX_FLAGS: ${CMAKE_CXX_FLAGS}")

###############
# Source
###############
set(
    SOURCE_LIST
    main.cpp
    Bsp.hpp
    Bsp.cpp
    BspBrushToMesh.cpp
    BspBrushToMesh.hpp
    Plane.hpp
    Trace.cpp
    Trace.hpp
    TraceTest.cpp
    TraceTest.hpp
    rAssert.hpp
    rAssert.cpp
    Geometry.hpp
    Matrix3x3Maths.hpp
    PlaneMaths.hpp
    QuaternionMaths.hpp
    VectorMaths3.hpp
    VectorMaths4.hpp
    third-party/ConvexHull/hull.cpp
    third-party/ConvexHull/hull.h)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3 -march=native -Wall -g")

###############
# Link
###############
add_executable(MessyBsp ${SOURCE_LIST})


