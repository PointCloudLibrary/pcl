#
#  Copyright 2011-12 ARM Limited
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.
#
cmake_minimum_required(VERSION 2.6)

project(NE10 C ASM)

option(NE10_BUILD_SHARED "Build NE10 shared libraries" OFF)
option(NE10_BUILD_STATIC "Build NE10 static libraries" ON)
option(NE10_BUILD_EXAMPLES "Build NE10 examples" ON)

set(NE10_VERSION 10)

# set complile flags for ARM.
set( CMAKE_C_FLAGS "-O3 -mthumb-interwork -march=armv7-a -mcpu=cortex-a9 -mfpu=vfp3" )

set( CMAKE_ASM_FLAGS "-mthumb-interwork -march=armv7-a -mcpu=cortex-a9 -mfpu=neon" )

# The NE10 library.
add_subdirectory(source)

if(NE10_BUILD_EXAMPLES)
    add_subdirectory(test)
endif()


