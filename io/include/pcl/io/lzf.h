/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2000-2008 Marc Alexander Lehmann <schmorp@schmorp.de>
 * Copyright (c) 2010-2011, Willow Garage, Inc.
 * 
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 * 
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

#pragma once

#include <pcl/pcl_macros.h>

namespace pcl
{
  /** \brief Compress in_len bytes stored at the memory block starting at
    * \a in_data and write the result to \a out_data, up to a maximum length
    * of \a out_len bytes using Marc Lehmann's LZF algorithm.
    *
    * If the output buffer is not large enough or any error occurs return 0,
    * otherwise return the number of bytes used, which might be considerably
    * more than in_len (but less than 104% of the original size), so it
    * makes sense to always use out_len == in_len - 1), to ensure _some_
    * compression, and store the data uncompressed otherwise (with a flag, of
    * course.
    *
    * \note The buffers must not be overlapping.
    *
    * \param[in] in_data the input uncompressed buffer
    * \param[in] in_len the length of the input buffer
    * \param[out] out_data the output buffer where the compressed result will be stored
    * \param[out] out_len the length of the output buffer
    *
    */
  PCL_EXPORTS unsigned int 
  lzfCompress (const void *const in_data,  unsigned int in_len,
               void             *out_data, unsigned int out_len);

  /** \brief Decompress data compressed with the \a lzfCompress function and
    * stored at location \a in_data and length \a in_len. The result will be
    * stored at \a out_data up to a maximum of \a out_len characters.
    *
    * If the output buffer is not large enough to hold the decompressed
    * data, a 0 is returned and errno is set to E2BIG. Otherwise the number
    * of decompressed bytes (i.e. the original length of the data) is
    * returned.
    *
    * If an error in the compressed data is detected, a zero is returned and
    * errno is set to EINVAL.
    *
    * This function is very fast, about as fast as a copying loop.
    * \param[in] in_data the input compressed buffer 
    * \param[in] in_len the length of the input buffer
    * \param[out] out_data the output buffer (must be resized to \a out_len)
    * \param[out] out_len the length of the output buffer
    */
  PCL_EXPORTS unsigned int 
  lzfDecompress (const void *const in_data,  unsigned int in_len,
                 void             *out_data, unsigned int out_len);
}
