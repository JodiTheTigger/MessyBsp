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
project(MessyBsp)

cmake_minimum_required(VERSION 3.1.0 FATAL_ERROR)
if (MSVC)
    cmake_minimum_required(VERSION 3.2.0 FATAL_ERROR)
endif()

###############
# Compiler Info
###############
include(CheckCXXCompilerFlag)

# Check compiler options
check_cxx_compiler_flag("-march=native" HAS_NATIVE)
check_cxx_compiler_flag("-fno-rtti" CAN_DISABLE_RTTI)
check_cxx_compiler_flag("-fno-exceptions" CAN_DISABLE_EXCEPTIONS)
check_cxx_compiler_flag(-Wall HAS_WALL)
check_cxx_compiler_flag(-W4 HAS_W4)
check_cxx_compiler_flag(-Wextra HAS_WEXTRA)
check_cxx_compiler_flag(-Werror HAS_WERROR)
check_cxx_compiler_flag(-WX HAS_WERROR_MSVC)
check_cxx_compiler_flag("/MP" HAS_MULTIPROCESSOR_COMPILE)

# Clang complains about perfectly valid brace initilisation in c++14
check_cxx_compiler_flag(-Wno-missing-braces HAS_IGNORE_MISSING_BRACES)

# 3rd party hull.cpp tries to disable a GCC warning, only to generate a
# clang one due to unrecognised pragmas.
check_cxx_compiler_flag(-Wno-unknown-pragmas HAS_IGNORE_UNKNOWN_PRAGMAS)


###############
# Prerequisites
###############

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
    set(WARNINGS_AS_ERRORS "${WARNINGS_AS_ERRORS} -Werror")
endif()

if (HAS_WERROR_MSVC)
    set(WARNINGS_AS_ERRORS "${WARNINGS_AS_ERRORS} -WX")
endif()

if (HAS_MULTIPROCESSOR_COMPILE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
endif()

if (HAS_IGNORE_MISSING_BRACES)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-missing-braces")
endif()

if (HAS_IGNORE_UNKNOWN_PRAGMAS)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unknown-pragmas")
endif()

if (MSVC)
    add_definitions(-D_CRT_SECURE_NO_WARNINGS)

	# Hack to allow MSVC2015 Preview to work
    add_definitions(-DMSVC2015_PREVIEW)
endif()

# Set a default build type for single-configuration
# CMake generators if no build type is set.
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

###############

# C++11/14 features I use.
set(needed_features

    cxx_generic_lambdas
    cxx_auto_type
    cxx_uniform_initialization
    cxx_trailing_return_types
    cxx_right_angle_brackets
    cxx_return_type_deduction
    cxx_nullptr
    cxx_alignas)

###############

Find_Package(GLEW REQUIRED)
if(NOT GLEW_FOUND)
    message(FATAL_ERROR "GLEW not found!")
endif()
include_directories(${GLEW_INCLUDE_DIR})

Find_Package(OpenGL REQUIRED)
if(NOT OPENGL_FOUND)
    message(FATAL_ERROR "OPENGL not found!")
endif()
include_directories(${OPENGL_INCLUDE_DIR})

# Cmake doesn't have a FindSDL2, had to get
# a third party one from the net.
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}")
Find_Package(SDL2 REQUIRED)
if(NOT SDL2_FOUND)
    message(FATAL_ERROR "SDL not found!")
endif()
include_directories(${SDL2_INCLUDE_DIR})


###############
# Source
###############
set(SOURCE_LIST_THIRDPARTY
    third-party/getopt/getopt.h
    third-party/getopt/getopt.c
    third-party/ConvexHull/hull.cpp
    third-party/ConvexHull/hull.h)

set(
    SOURCE_LIST
    main.cpp
    GLDebug.hpp
    Bsp.hpp
    Bsp.cpp
    BspBrushToMesh.cpp
    BspBrushToMesh.hpp
    Trace.cpp
    Trace.hpp
    TraceTest.cpp
    TraceTest.hpp
    rAssert.hpp
    rAssert.cpp
    Geometry.hpp
    Matrix4x4Maths.hpp
    PlaneMaths.hpp
    QuaternionMaths.hpp
    VectorMaths3.hpp
    VectorMaths4.hpp)

set(
    TEST_LIST
    test/TestBspBrushToMesh.cpp)

set_source_files_properties(
    ${SOURCE_LIST}
    PROPERTIES
    COMPILE_FLAGS ${WARNINGS_AS_ERRORS})


#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3 -march=native -Wall -g")

###############
# Link
###############
add_executable(MessyBsp ${SOURCE_LIST} ${SOURCE_LIST_THIRDPARTY})
target_compile_features(MessyBsp PRIVATE ${needed_features})
target_link_libraries(${PROJECT_NAME} ${SDL2_LIBRARY})
target_link_libraries(${PROJECT_NAME} ${GLEW_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${OPENGL_LIBRARIES})

message("CXX_FLAGS: ${CMAKE_CXX_FLAGS}")
