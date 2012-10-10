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

# header
include_directories (
    ../inc
    ../
)

if(NE10_BUILD_SHARED)
    add_executable(NE10_test_dynamic NE10_test.c)
    target_link_libraries (
        NE10_test_dynamic
        NE10_test
        m
    )
endif()

if(NE10_BUILD_STATIC)
    add_executable(NE10_test_static NE10_test.c)
    target_link_libraries (
        NE10_test_static
        NE10
        m
    )
endif()


