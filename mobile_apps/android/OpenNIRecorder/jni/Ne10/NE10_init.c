/*
 *  Copyright 2011-12 ARM Limited
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */

#include "NE10.h"

#include <stdio.h>

#define CPUINFO_BUFFER_SIZE  (1024*4)

// This local variable indicates whether or not the running platform supports ARM NEON
arm_result_t is_NEON_available = NE10_ERR;

arm_result_t NE10_HasNEON()
{
    return is_NEON_available;
}

arm_result_t NE10_init()
{
    FILE*   infofile = NULL;               // To open the file /proc/cpuinfo
    char    cpuinfo[CPUINFO_BUFFER_SIZE];  // The buffer to read in the string
    size_t  bytes = 0;                     // Numbers of bytes read from the file
    int     i = 0;                         // Temporary loop counter

    memset( cpuinfo, 0, CPUINFO_BUFFER_SIZE );
    infofile = fopen( "/proc/cpuinfo", "r" );
    bytes    = fread( cpuinfo, 1, sizeof(cpuinfo), infofile );
    fclose( infofile );

    if( 0 == bytes || CPUINFO_BUFFER_SIZE == bytes )
    {
        fprintf( stderr, "ERROR: Couldn't read the file \"/proc/cpuinfo\". NE10_init() failed.\n");
        return NE10_ERR;
    }

    while( '\0' != cpuinfo[i] ) cpuinfo[i++] = (char)tolower(cpuinfo[i]);

    if ( 0 != strstr(cpuinfo, "neon") )
    {
       is_NEON_available = NE10_OK;
    }

    if ( NE10_OK == NE10_HasNEON() )
    {
      addc_float = addc_float_neon;
      addc_vec2f = addc_vec2f_neon;
      addc_vec3f = addc_vec3f_neon;
      addc_vec4f = addc_vec4f_neon;
      subc_float = subc_float_neon;
      subc_vec2f = subc_vec2f_neon;
      subc_vec3f = subc_vec3f_neon;
      subc_vec4f = subc_vec4f_neon;
      rsbc_float = rsbc_float_neon;
      rsbc_vec2f = rsbc_vec2f_neon;
      rsbc_vec3f = rsbc_vec3f_neon;
      rsbc_vec4f = rsbc_vec4f_neon;
      mulc_float = mulc_float_neon;
      mulc_vec2f = mulc_vec2f_neon;
      mulc_vec3f = mulc_vec3f_neon;
      mulc_vec4f = mulc_vec4f_neon;
      divc_float = divc_float_neon;
      divc_vec2f = divc_vec2f_neon;
      divc_vec3f = divc_vec3f_neon;
      divc_vec4f = divc_vec4f_neon;
      setc_float = setc_float_neon;
      setc_vec2f = setc_vec2f_neon;
      setc_vec3f = setc_vec3f_neon;
      setc_vec4f = setc_vec4f_neon;
      mlac_float = mlac_float_neon;
      mlac_vec2f = mlac_vec2f_neon;
      mlac_vec3f = mlac_vec3f_neon;
      mlac_vec4f = mlac_vec4f_neon;
      add_float = add_float_neon;
      sub_float = sub_float_neon;
      mul_float = mul_float_neon;
      div_float = div_float_neon;
      mla_float = mla_float_neon;
      abs_float = abs_float_neon;
      len_vec2f = len_vec2f_neon;
      len_vec3f = len_vec3f_neon;
      len_vec4f = len_vec4f_neon;
      normalize_vec2f = normalize_vec2f_neon;
      normalize_vec3f = normalize_vec3f_neon;
      normalize_vec4f = normalize_vec4f_neon;

      abs_vec2f = abs_vec2f_neon;
      abs_vec3f = abs_vec3f_neon;
      abs_vec4f = abs_vec4f_neon;
      vmul_vec2f = vmul_vec2f_neon;
      vmul_vec3f = vmul_vec3f_neon;
      vmul_vec4f = vmul_vec4f_neon;
      vdiv_vec2f = vdiv_vec2f_neon;
      vdiv_vec3f = vdiv_vec3f_neon;
      vdiv_vec4f = vdiv_vec4f_neon;
      vmla_vec2f = vmla_vec2f_neon;
      vmla_vec3f = vmla_vec3f_neon;
      vmla_vec4f = vmla_vec4f_neon;
      add_vec2f = add_vec2f_neon;
      add_vec3f = add_vec3f_neon;
      add_vec4f = add_vec4f_neon;
      sub_vec2f = sub_vec2f_neon;
      sub_vec3f = sub_vec3f_neon;
      sub_vec4f = sub_vec4f_neon;
      dot_vec2f = dot_vec2f_neon;
      dot_vec3f = dot_vec3f_neon;
      dot_vec4f = dot_vec4f_neon;
      cross_vec3f = cross_vec3f_neon;

      addmat_2x2f = addmat_2x2f_neon;
      addmat_3x3f = addmat_3x3f_neon;
      addmat_4x4f = addmat_4x4f_neon;
      submat_2x2f = submat_2x2f_neon;
      submat_3x3f = submat_3x3f_neon;
      submat_4x4f = submat_4x4f_neon;
      mulmat_2x2f = mulmat_2x2f_neon;
      mulmat_3x3f = mulmat_3x3f_neon;
      mulmat_4x4f = mulmat_4x4f_neon;
      mulcmatvec_cm2x2f_v2f = mulcmatvec_cm2x2f_v2f_neon;
      mulcmatvec_cm3x3f_v3f = mulcmatvec_cm3x3f_v3f_neon;
      mulcmatvec_cm4x4f_v4f = mulcmatvec_cm4x4f_v4f_neon;
      detmat_2x2f = detmat_2x2f_neon;
      detmat_3x3f = detmat_3x3f_neon;
      detmat_4x4f = detmat_4x4f_neon;
      invmat_2x2f = invmat_2x2f_neon;
      invmat_3x3f = invmat_3x3f_neon;
      invmat_4x4f = invmat_4x4f_neon;
      transmat_4x4f = transmat_4x4f_neon;
      identitymat_4x4f = identitymat_4x4f_neon;
      transmat_3x3f = transmat_3x3f_neon;
      identitymat_3x3f = identitymat_3x3f_neon;
      transmat_2x2f = transmat_2x2f_neon;
      identitymat_2x2f = identitymat_2x2f_neon;
    }
    else
    {
      addc_float = addc_float_c;
      addc_vec2f = addc_vec2f_c;
      addc_vec3f = addc_vec3f_c;
      addc_vec4f = addc_vec4f_c;
      subc_float = subc_float_c;
      subc_vec2f = subc_vec2f_c;
      subc_vec3f = subc_vec3f_c;
      subc_vec4f = subc_vec4f_c;
      rsbc_float = rsbc_float_c;
      rsbc_vec2f = rsbc_vec2f_c;
      rsbc_vec3f = rsbc_vec3f_c;
      rsbc_vec4f = rsbc_vec4f_c;
      mulc_float = mulc_float_c;
      mulc_vec2f = mulc_vec2f_c;
      mulc_vec3f = mulc_vec3f_c;
      mulc_vec4f = mulc_vec4f_c;
      divc_float = divc_float_c;
      divc_vec2f = divc_vec2f_c;
      divc_vec3f = divc_vec3f_c;
      divc_vec4f = divc_vec4f_c;
      setc_float = setc_float_c;
      setc_vec2f = setc_vec2f_c;
      setc_vec3f = setc_vec3f_c;
      setc_vec4f = setc_vec4f_c;
      mlac_float = mlac_float_c;
      mlac_vec2f = mlac_vec2f_c;
      mlac_vec3f = mlac_vec3f_c;
      mlac_vec4f = mlac_vec4f_c;
      add_float = add_float_c;
      sub_float = sub_float_c;
      mul_float = mul_float_c;
      div_float = div_float_c;
      mla_float = mla_float_c;
      abs_float = abs_float_c;
      len_vec2f = len_vec2f_c;
      len_vec3f = len_vec3f_c;
      len_vec4f = len_vec4f_c;
      normalize_vec2f = normalize_vec2f_c;
      normalize_vec3f = normalize_vec3f_c;
      normalize_vec4f = normalize_vec4f_c;

      abs_vec2f = abs_vec2f_c;
      abs_vec3f = abs_vec3f_c;
      abs_vec4f = abs_vec4f_c;
      vmul_vec2f = vmul_vec2f_c;
      vmul_vec3f = vmul_vec3f_c;
      vmul_vec4f = vmul_vec4f_c;
      vdiv_vec2f = vdiv_vec2f_c;
      vdiv_vec3f = vdiv_vec3f_c;
      vdiv_vec4f = vdiv_vec4f_c;
      vmla_vec2f = vmla_vec2f_c;
      vmla_vec3f = vmla_vec3f_c;
      vmla_vec4f = vmla_vec4f_c;
      add_vec2f = add_vec2f_c;
      add_vec3f = add_vec3f_c;
      add_vec4f = add_vec4f_c;
      sub_vec2f = sub_vec2f_c;
      sub_vec3f = sub_vec3f_c;
      sub_vec4f = sub_vec4f_c;
      dot_vec2f = dot_vec2f_c;
      dot_vec3f = dot_vec3f_c;
      dot_vec4f = dot_vec4f_c;
      cross_vec3f = cross_vec3f_c;

      addmat_2x2f = addmat_2x2f_c;
      addmat_3x3f = addmat_3x3f_c;
      addmat_4x4f = addmat_4x4f_c;
      submat_2x2f = submat_2x2f_c;
      submat_3x3f = submat_3x3f_c;
      submat_4x4f = submat_4x4f_c;
      mulmat_2x2f = mulmat_2x2f_c;
      mulmat_3x3f = mulmat_3x3f_c;
      mulmat_4x4f = mulmat_4x4f_c;
      mulcmatvec_cm2x2f_v2f = mulcmatvec_cm2x2f_v2f_c;
      mulcmatvec_cm3x3f_v3f = mulcmatvec_cm3x3f_v3f_c;
      mulcmatvec_cm4x4f_v4f = mulcmatvec_cm4x4f_v4f_c;
      detmat_2x2f = detmat_2x2f_c;
      detmat_3x3f = detmat_3x3f_c;
      detmat_4x4f = detmat_4x4f_c;
      invmat_2x2f = invmat_2x2f_c;
      invmat_3x3f = invmat_3x3f_c;
      invmat_4x4f = invmat_4x4f_c;
      transmat_4x4f = transmat_4x4f_c;
      identitymat_4x4f = identitymat_4x4f_c;
      transmat_3x3f = transmat_3x3f_c;
      identitymat_3x3f = identitymat_3x3f_c;
      transmat_2x2f = transmat_2x2f_c;
      identitymat_2x2f = identitymat_2x2f_c;
    }
}

// These are actual definitions of our function pointers that are declared in inc/NE10.h
arm_result_t (*addc_float)(arm_float_t * dst, arm_float_t * src, const arm_float_t cst, unsigned int count);
arm_result_t (*addc_vec2f)(arm_vec2f_t * dst, arm_vec2f_t * src, const arm_vec2f_t * cst, unsigned int count);
arm_result_t (*addc_vec3f)(arm_vec3f_t * dst, arm_vec3f_t * src, const arm_vec3f_t * cst, unsigned int count);
arm_result_t (*addc_vec4f)(arm_vec4f_t * dst, arm_vec4f_t * src, const arm_vec4f_t * cst, unsigned int count);
arm_result_t (*subc_float)(arm_float_t * dst, arm_float_t * src, const arm_float_t cst, unsigned int count);
arm_result_t (*subc_vec2f)(arm_vec2f_t * dst, arm_vec2f_t * src, const arm_vec2f_t * cst, unsigned int count);
arm_result_t (*subc_vec3f)(arm_vec3f_t * dst, arm_vec3f_t * src, const arm_vec3f_t * cst, unsigned int count);
arm_result_t (*subc_vec4f)(arm_vec4f_t * dst, arm_vec4f_t * src, const arm_vec4f_t * cst, unsigned int count);
arm_result_t (*rsbc_float)(arm_float_t * dst, arm_float_t *src, const arm_float_t cst, unsigned int count);
arm_result_t (*rsbc_vec2f)(arm_vec2f_t * dst, arm_vec2f_t * src, const arm_vec2f_t * cst, unsigned int count);
arm_result_t (*rsbc_vec3f)(arm_vec3f_t * dst, arm_vec3f_t * src, const arm_vec3f_t * cst, unsigned int count);
arm_result_t (*rsbc_vec4f)(arm_vec4f_t * dst, arm_vec4f_t * src, const arm_vec4f_t * cst, unsigned int count);
arm_result_t (*mulc_float)(arm_float_t * dst, arm_float_t * src, const arm_float_t cst, unsigned int count);
arm_result_t (*mulc_vec2f)(arm_vec2f_t * dst, arm_vec2f_t * src, const arm_vec2f_t * cst, unsigned int count);
arm_result_t (*mulc_vec3f)(arm_vec3f_t * dst, arm_vec3f_t * src, const arm_vec3f_t * cst, unsigned int count);
arm_result_t (*mulc_vec4f)(arm_vec4f_t * dst, arm_vec4f_t * src, const arm_vec4f_t * cst, unsigned int count);
arm_result_t (*divc_float)(arm_float_t * dst, arm_float_t * src, const arm_float_t cst, unsigned int count);
arm_result_t (*divc_vec2f)(arm_vec2f_t * dst, arm_vec2f_t * src, const arm_vec2f_t * cst, unsigned int count);
arm_result_t (*divc_vec3f)(arm_vec3f_t * dst, arm_vec3f_t * src, const arm_vec3f_t * cst, unsigned int count);
arm_result_t (*divc_vec4f)(arm_vec4f_t * dst, arm_vec4f_t * src, const arm_vec4f_t * cst, unsigned int count);
arm_result_t (*setc_float)(arm_float_t * dst, const arm_float_t cst, unsigned int count);
arm_result_t (*setc_vec2f)(arm_vec2f_t * dst, const arm_vec2f_t * cst, unsigned int count);
arm_result_t (*setc_vec3f)(arm_vec3f_t * dst, const arm_vec3f_t * cst, unsigned int count);
arm_result_t (*setc_vec4f)(arm_vec4f_t * dst, const arm_vec4f_t * cst, unsigned int count);
arm_result_t (*mlac_float)(arm_float_t * dst, arm_float_t * acc, arm_float_t * src, const arm_float_t cst, unsigned int count);
arm_result_t (*mlac_vec2f)(arm_vec2f_t * dst, arm_vec2f_t * acc, arm_vec2f_t * src, const arm_vec2f_t * cst, unsigned int count);
arm_result_t (*mlac_vec3f)(arm_vec3f_t * dst, arm_vec3f_t * acc, arm_vec3f_t * src, const arm_vec3f_t * cst, unsigned int count);
arm_result_t (*mlac_vec4f)(arm_vec4f_t * dst, arm_vec4f_t * acc, arm_vec4f_t * src, const arm_vec4f_t * cst, unsigned int count);
arm_result_t (*add_float)(arm_float_t * dst, arm_float_t * src1, arm_float_t * src2, unsigned int count);
arm_result_t (*sub_float)(arm_float_t * dst, arm_float_t * src1, arm_float_t * src2, unsigned int count);
arm_result_t (*mul_float)(arm_float_t * dst, arm_float_t * src1, arm_float_t * src2, unsigned int count);
arm_result_t (*div_float)(arm_float_t * dst, arm_float_t * src1, arm_float_t * src2, unsigned int count);
arm_result_t (*mla_float)(arm_float_t * dst, arm_float_t * acc, arm_float_t * src1, arm_float_t * src2, unsigned int count);
arm_result_t (*abs_float)(arm_float_t * dst, arm_float_t * src, unsigned int count);
arm_result_t (*len_vec2f)(arm_float_t * dst, arm_vec2f_t * src, unsigned int count);
arm_result_t (*len_vec3f)(arm_float_t * dst, arm_vec3f_t * src, unsigned int count);
arm_result_t (*len_vec4f)(arm_float_t * dst, arm_vec4f_t * src, unsigned int count);
arm_result_t (*normalize_vec2f)(arm_vec2f_t * dst, arm_vec2f_t * src, unsigned int count);
arm_result_t (*normalize_vec3f)(arm_vec3f_t * dst, arm_vec3f_t * src, unsigned int count);
arm_result_t (*normalize_vec4f)(arm_vec4f_t * dst, arm_vec4f_t * src, unsigned int count);

arm_result_t (*abs_vec2f)(arm_vec2f_t * dst, arm_vec2f_t * src, unsigned int count);
arm_result_t (*abs_vec3f)(arm_vec3f_t * dst, arm_vec3f_t * src, unsigned int count);
arm_result_t (*abs_vec4f)(arm_vec4f_t * dst, arm_vec4f_t * src, unsigned int count);
arm_result_t (*vmul_vec2f)(arm_vec2f_t * dst, arm_vec2f_t * src1, arm_vec2f_t * src2, unsigned int count);
arm_result_t (*vmul_vec3f)(arm_vec3f_t * dst, arm_vec3f_t * src1, arm_vec3f_t * src2, unsigned int count);
arm_result_t (*vmul_vec4f)(arm_vec4f_t * dst, arm_vec4f_t * src1, arm_vec4f_t * src2, unsigned int count);
arm_result_t (*vdiv_vec2f)(arm_vec2f_t * dst, arm_vec2f_t * src1, arm_vec2f_t * src2, unsigned int count);
arm_result_t (*vdiv_vec3f)(arm_vec3f_t * dst, arm_vec3f_t * src1, arm_vec3f_t * src2, unsigned int count);
arm_result_t (*vdiv_vec4f)(arm_vec4f_t * dst, arm_vec4f_t * src1, arm_vec4f_t * src2, unsigned int count);
arm_result_t (*vmla_vec2f)(arm_vec2f_t * acc, arm_vec2f_t * src1, arm_vec2f_t * src2, unsigned int count);
arm_result_t (*vmla_vec3f)(arm_vec3f_t * acc, arm_vec3f_t * src1, arm_vec3f_t * src2, unsigned int count);
arm_result_t (*vmla_vec4f)(arm_vec4f_t * acc, arm_vec4f_t * src1, arm_vec4f_t * src2, unsigned int count);
arm_result_t (*add_vec2f)(arm_vec2f_t * dst, arm_vec2f_t * src1, arm_vec2f_t * src2, unsigned int count);
arm_result_t (*add_vec3f)(arm_vec3f_t * dst, arm_vec3f_t * src1, arm_vec3f_t * src2, unsigned int count);
arm_result_t (*add_vec4f)(arm_vec4f_t * dst, arm_vec4f_t * src1, arm_vec4f_t * src2, unsigned int count);
arm_result_t (*sub_vec2f)(arm_vec2f_t * dst, arm_vec2f_t * src1, arm_vec2f_t * src2, unsigned int count);
arm_result_t (*sub_vec3f)(arm_vec3f_t * dst, arm_vec3f_t * src1, arm_vec3f_t * src2, unsigned int count);
arm_result_t (*sub_vec4f)(arm_vec4f_t * dst, arm_vec4f_t * src1, arm_vec4f_t * src2, unsigned int count);
arm_result_t (*dot_vec2f)(arm_float_t * dst, arm_vec2f_t * src1, arm_vec2f_t * src2, unsigned int count);
arm_result_t (*dot_vec3f)(arm_float_t * dst, arm_vec3f_t * src1, arm_vec3f_t * src2, unsigned int count);
arm_result_t (*dot_vec4f)(arm_float_t * dst, arm_vec4f_t * src1, arm_vec4f_t * src2, unsigned int count);
arm_result_t (*cross_vec3f)(arm_vec3f_t * dst, arm_vec3f_t * src1, arm_vec3f_t * src2, unsigned int count);

arm_result_t (*addmat_2x2f)(arm_mat2x2f_t * dst, arm_mat2x2f_t * src1, arm_mat2x2f_t * src2, unsigned int count);
arm_result_t (*addmat_3x3f)(arm_mat3x3f_t * dst, arm_mat3x3f_t * src1, arm_mat3x3f_t * src2, unsigned int count);
arm_result_t (*addmat_4x4f)(arm_mat4x4f_t * dst, arm_mat4x4f_t * src1, arm_mat4x4f_t * src2, unsigned int count);
arm_result_t (*submat_2x2f)(arm_mat2x2f_t * dst, arm_mat2x2f_t * src1, arm_mat2x2f_t * src2, unsigned int count);
arm_result_t (*submat_3x3f)(arm_mat3x3f_t * dst, arm_mat3x3f_t * src1, arm_mat3x3f_t * src2, unsigned int count);
arm_result_t (*submat_4x4f)(arm_mat4x4f_t * dst, arm_mat4x4f_t * src1, arm_mat4x4f_t * src2, unsigned int count);
arm_result_t (*mulmat_2x2f)(arm_mat2x2f_t * dst, arm_mat2x2f_t * src1, arm_mat2x2f_t * src2, unsigned int count);
arm_result_t (*mulmat_3x3f)(arm_mat3x3f_t * dst, arm_mat3x3f_t * src1, arm_mat3x3f_t * src2, unsigned int count);
arm_result_t (*mulmat_4x4f)(arm_mat4x4f_t * dst, arm_mat4x4f_t * src1, arm_mat4x4f_t * src2, unsigned int count);
arm_result_t (*mulcmatvec_cm4x4f_v4f)(arm_vec4f_t * dst, const arm_mat4x4f_t * cst, arm_vec4f_t * src, unsigned int count);
arm_result_t (*mulcmatvec_cm3x3f_v3f)(arm_vec3f_t * dst, const arm_mat3x3f_t * cst, arm_vec3f_t * src, unsigned int count);
arm_result_t (*mulcmatvec_cm2x2f_v2f)(arm_vec2f_t * dst, const arm_mat2x2f_t * cst, arm_vec2f_t * src, unsigned int count);
arm_result_t (*detmat_4x4f)(arm_float_t * dst, arm_mat4x4f_t * src, unsigned int count);
arm_result_t (*detmat_3x3f)(arm_float_t * dst, arm_mat3x3f_t * src, unsigned int count);
arm_result_t (*detmat_2x2f)(arm_float_t * dst, arm_mat2x2f_t * src, unsigned int count);
arm_result_t (*invmat_4x4f)(arm_mat4x4f_t * dst, arm_mat4x4f_t * src, unsigned int count);
arm_result_t (*invmat_3x3f)(arm_mat3x3f_t * dst, arm_mat3x3f_t * src, unsigned int count);
arm_result_t (*invmat_2x2f)(arm_mat2x2f_t * dst, arm_mat2x2f_t * src, unsigned int count);
arm_result_t (*transmat_4x4f)(arm_mat4x4f_t * dst, arm_mat4x4f_t * src, unsigned int count);
arm_result_t (*identitymat_4x4f)(arm_mat4x4f_t * dst, unsigned int count);
arm_result_t (*transmat_3x3f)(arm_mat3x3f_t * dst, arm_mat3x3f_t * src, unsigned int count);
arm_result_t (*identitymat_3x3f)(arm_mat3x3f_t * dst, unsigned int count);
arm_result_t (*transmat_2x2f)(arm_mat2x2f_t * dst, arm_mat2x2f_t * src, unsigned int count);
arm_result_t (*identitymat_2x2f)(arm_mat2x2f_t * dst, unsigned int count);
