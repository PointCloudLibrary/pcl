/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (C) 2009-2010, NVIDIA Corporation, all rights reserved.
 *  Third party copyrights are property of their respective owners.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
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
 * $Id:  $
 * Ported to PCL by Koen Buys : Attention Work in progress!
 */

#ifndef _ncv_color_conversion_hpp_
#define _ncv_color_conversion_hpp_

#include "NCVPixelOperations.hpp"

enum NCVColorSpace
{
    NCVColorSpaceGray,
    NCVColorSpaceRGBA,
};

template<NCVColorSpace CSin, NCVColorSpace CSout, typename Tin, typename Tout> struct __pixColorConv {
static void _pixColorConv(const Tin &pixIn, Tout &pixOut);
};

template<typename Tin, typename Tout> struct __pixColorConv<NCVColorSpaceRGBA, NCVColorSpaceGray, Tin, Tout> {
static void _pixColorConv(const Tin &pixIn, Tout &pixOut)
{
    Ncv32f luma = 0.299f * pixIn.x + 0.587f * pixIn.y + 0.114f * pixIn.z;
    _TDemoteClampNN(luma, pixOut.x);
}};

template<typename Tin, typename Tout> struct __pixColorConv<NCVColorSpaceGray, NCVColorSpaceRGBA, Tin, Tout> {
static void _pixColorConv(const Tin &pixIn, Tout &pixOut)
{
    _TDemoteClampNN(pixIn.x, pixOut.x);
    _TDemoteClampNN(pixIn.x, pixOut.y);
    _TDemoteClampNN(pixIn.x, pixOut.z);
    pixOut.w = 0;
}};

template<NCVColorSpace CSin, NCVColorSpace CSout, typename Tin, typename Tout>
static
NCVStatus _ncvColorConv_host(const NCVMatrix<Tin> &h_imgIn,
                             const NCVMatrix<Tout> &h_imgOut)
{
    ncvAssertReturn(h_imgIn.size() == h_imgOut.size(), NCV_DIMENSIONS_INVALID);
    ncvAssertReturn(h_imgIn.memType() == h_imgOut.memType() &&
                    (h_imgIn.memType() == NCVMemoryTypeHostPinned || h_imgIn.memType() == NCVMemoryTypeNone), NCV_MEM_RESIDENCE_ERROR);
    NCV_SET_SKIP_COND(h_imgIn.memType() == NCVMemoryTypeNone);
    NCV_SKIP_COND_BEGIN

    for (Ncv32u i=0; i<h_imgIn.height(); i++)
    {
        for (Ncv32u j=0; j<h_imgIn.width(); j++)
        {
            __pixColorConv<CSin, CSout, Tin, Tout>::_pixColorConv(h_imgIn.at(j,i), h_imgOut.at(j,i));
        }
    }

    NCV_SKIP_COND_END
    return NCV_SUCCESS;
}

#endif //_ncv_color_conversion_hpp_
