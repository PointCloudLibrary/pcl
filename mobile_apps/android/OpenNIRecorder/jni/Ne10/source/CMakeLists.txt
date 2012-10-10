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


# Define C files.
set(NE10_C_SRCS
    NE10_abs.c
    NE10_addc.c
    NE10_add.c
    NE10_divc.c
    NE10_div.c
    NE10_len.c
    NE10_mlac.c
    NE10_mla.c
    NE10_mulc.c
    NE10_mul.c
    NE10_normalize.c
    NE10_rsbc.c
    NE10_setc.c
    NE10_subc.c
    NE10_sub.c
    NE10_dot.c
    NE10_cross.c
    NE10_addmat.c
    NE10_submat.c
    NE10_mulmat.c
    NE10_mulcmatvec.c
    NE10_detmat.c
    NE10_invmat.c
    NE10_transmat.c
    NE10_identitymat.c
)

# Define intrinsic NEON files.
set(NE10_INTRINSIC_SRCS
    NE10_addc.neon.c
    NE10_divc.neon.c
    NE10_mlac.neon.c
    NE10_mulc.neon.c
    NE10_rsbc.neon.c
    NE10_setc.neon.c
    NE10_subc.neon.c
    NE10_addmat.neon.c
    NE10_submat.neon.c
)

# Tell CMake these files need to be compiled with "-mfpu=neon"
foreach(intrinsic_file ${NE10_INTRINSIC_SRCS})
    set_source_files_properties(${intrinsic_file} PROPERTIES COMPILE_FLAGS "-mfpu=neon" )
endforeach(intrinsic_file)

# Define NEON files.
set(NE10_NEON_SRCS
    NE10_abs.neon.s
    NE10_add.neon.s
    NE10_div.neon.s
    NE10_len.neon.s
    NE10_mla.neon.s
    NE10_mul.neon.s
    NE10_normalize.neon.s
    NE10_sub.neon.s
    NE10_dot.neon.s
    NE10_cross.neon.s
    NE10_mulmat.neon.s
    NE10_mulcmatvec.neon.s
    NE10_detmat.neon.s
    NE10_invmat.neon.s
    NE10_transmat.neon.s
    NE10_identitymat.neon.s
    NE10_detmat.neon.inc.s
)

# Tell CMake these files need to go to the C compiler
set(FLAGS "-mfpu=neon -Wa,-I../../inc -Wa,-I../../headers -Wa,-I../../ -Wa,-I../../source" )
foreach(neon_file ${NE10_NEON_SRCS})
    set_property (SOURCE ${neon_file} PROPERTY LANGUAGE C)
    set_source_files_properties(
        ${neon_file} PROPERTIES COMPILE_FLAGS
        ${FLAGS}
    )
endforeach(neon_file)

# Define init files.
set(NE10_INIT_SRCS
    ../NE10_init.c
)
# Define test files.
set(NE10_TEST_SRCS
    NE10_abs_test.c
    NE10_addc_test.c
    NE10_add_test.c
    NE10_divc_test.c
    NE10_div_test.c
    NE10_len_test.c
    NE10_mlac_test.c
    NE10_mla_test.c
    NE10_mulc_test.c
    NE10_mul_test.c
    NE10_normalize_test.c
    NE10_rsbc_test.c
    NE10_setc_test.c
    NE10_subc_test.c
    NE10_sub_test.c intrinsic_file
    NE10_dot_test.c
    NE10_cross_test.c
    NE10_addmat_test.c
    NE10_submat_test.c
    NE10_mulmat_test.c
    NE10_mulcmatvec_test.c
    NE10_detmat_test.c
    NE10_invmat_test.c
    NE10_transmat_test.c
    NE10_identitymat_test.c
)

include_directories (
    ../inc
    ../headers
    ../
    ../source
)

if(NE10_BUILD_STATIC)
    add_library( NE10 STATIC
        ${NE10_C_SRCS}
        ${NE10_INTRINSIC_SRCS}
        ${NE10_NEON_SRCS}
        ${NE10_INIT_SRCS}
    )
    set_target_properties(NE10 PROPERTIES
        CLEAN_DIRECT_OUTPUT 1
        VERSION ${NE10_VERSION}
    )
endif()

if(NE10_BUILD_SHARED)

    add_library( NE10_shared SHARED
        ${NE10_C_SRCS}
        ${NE10_INTRINSIC_SRCS}
        ${NE10_NEON_SRCS}
        ${NE10_INIT_SRCS}
    )

    set_target_properties(NE10_shared PROPERTIES
        OUTPUT_NAME "NE10"
        CLEAN_DIRECT_OUTPUT 1
        VERSION ${NE10_VERSION}
    )

    add_library( NE10_test SHARED
        ${NE10_C_SRCS}
        ${NE10_INTRINSIC_SRCS}
        ${NE10_NEON_SRCS}
        ${NE10_INIT_SRCS}
    )

    set_target_properties(NE10_test PROPERTIES
        OUTPUT_NAME "NE10_test"
        CLEAN_DIRECT_OUTPUT 1
        VERSION ${NE10_VERSION}
    )

endif()

# install libraries
#if(NE10_BUILD_SHARED)
#    install(TARGETS NE10_shared DESTINATION lib)
#endif()
#if(NE10_BUILD_STATIC)
#    install(TARGETS NE10 DESTINATION lib)
#endif()
