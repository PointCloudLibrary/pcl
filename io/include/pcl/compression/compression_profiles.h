/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 * Author: Julius Kammerl (julius@kammerl.de)
 */

#pragma once

namespace pcl
{
  namespace io
  {

    enum compression_Profiles_e
    {
      LOW_RES_ONLINE_COMPRESSION_WITHOUT_COLOR,
      LOW_RES_ONLINE_COMPRESSION_WITH_COLOR,

      MED_RES_ONLINE_COMPRESSION_WITHOUT_COLOR,
      MED_RES_ONLINE_COMPRESSION_WITH_COLOR,

      HIGH_RES_ONLINE_COMPRESSION_WITHOUT_COLOR,
      HIGH_RES_ONLINE_COMPRESSION_WITH_COLOR,

      LOW_RES_OFFLINE_COMPRESSION_WITHOUT_COLOR,
      LOW_RES_OFFLINE_COMPRESSION_WITH_COLOR,

      MED_RES_OFFLINE_COMPRESSION_WITHOUT_COLOR,
      MED_RES_OFFLINE_COMPRESSION_WITH_COLOR,

      HIGH_RES_OFFLINE_COMPRESSION_WITHOUT_COLOR,
      HIGH_RES_OFFLINE_COMPRESSION_WITH_COLOR,

      COMPRESSION_PROFILE_COUNT,
      MANUAL_CONFIGURATION
    };

    // compression configuration profile
    struct configurationProfile_t
    {
      double pointResolution;
      const double octreeResolution;
      bool doVoxelGridDownSampling;
      unsigned int iFrameRate;
      const unsigned char colorBitResolution;
      bool doColorEncoding;
    };

    // predefined configuration parameters
    const struct configurationProfile_t compressionProfiles_[COMPRESSION_PROFILE_COUNT] = {
    {
    // PROFILE: LOW_RES_ONLINE_COMPRESSION_WITHOUT_COLOR
       0.01, /* pointResolution = */
       0.01, /* octreeResolution = */
       true, /* doVoxelGridDownDownSampling = */
       50, /* iFrameRate = */
       4, /* colorBitResolution = */
       false /* doColorEncoding = */
    }, {
    // PROFILE: LOW_RES_ONLINE_COMPRESSION_WITH_COLOR
        0.01, /* pointResolution = */
        0.01, /* octreeResolution = */
        true, /* doVoxelGridDownDownSampling = */
        50, /* iFrameRate = */
        4, /* colorBitResolution = */
        true /* doColorEncoding = */
    }, {
    // PROFILE: MED_RES_ONLINE_COMPRESSION_WITHOUT_COLOR
        0.005, /* pointResolution = */
        0.01, /* octreeResolution = */
        false, /* doVoxelGridDownDownSampling = */
        40, /* iFrameRate = */
        5, /* colorBitResolution = */
        false /* doColorEncoding = */
    }, {
    // PROFILE: MED_RES_ONLINE_COMPRESSION_WITH_COLOR
        0.005, /* pointResolution = */
        0.01, /* octreeResolution = */
        false, /* doVoxelGridDownDownSampling = */
        40, /* iFrameRate = */
        5, /* colorBitResolution = */
        true /* doColorEncoding = */
    }, {
    // PROFILE: HIGH_RES_ONLINE_COMPRESSION_WITHOUT_COLOR
        0.0001, /* pointResolution = */
        0.01, /* octreeResolution = */
        false, /* doVoxelGridDownDownSampling = */
        30, /* iFrameRate = */
        7, /* colorBitResolution = */
        false /* doColorEncoding = */
    }, {
    // PROFILE: HIGH_RES_ONLINE_COMPRESSION_WITH_COLOR
        0.0001, /* pointResolution = */
        0.01, /* octreeResolution = */
        false, /* doVoxelGridDownDownSampling = */
        30, /* iFrameRate = */
        7, /* colorBitResolution = */
        true /* doColorEncoding = */
    }, {
    // PROFILE: LOW_RES_OFFLINE_COMPRESSION_WITHOUT_COLOR
        0.01, /* pointResolution = */
        0.01, /* octreeResolution = */
        true, /* doVoxelGridDownDownSampling = */
        100, /* iFrameRate = */
        4, /* colorBitResolution = */
        false /* doColorEncoding = */
    }, {
    // PROFILE: LOW_RES_OFFLINE_COMPRESSION_WITH_COLOR
        0.01, /* pointResolution = */
        0.01, /* octreeResolution = */
        true, /* doVoxelGridDownDownSampling = */
        100, /* iFrameRate = */
        4, /* colorBitResolution = */
        true /* doColorEncoding = */
    }, {
    // PROFILE: MED_RES_OFFLINE_COMPRESSION_WITHOUT_COLOR
        0.005, /* pointResolution = */
        0.005, /* octreeResolution = */
        true, /* doVoxelGridDownDownSampling = */
        100, /* iFrameRate = */
        5, /* colorBitResolution = */
        false /* doColorEncoding = */
    }, {
    // PROFILE: MED_RES_OFFLINE_COMPRESSION_WITH_COLOR
        0.005, /* pointResolution = */
        0.01, /* octreeResolution = */
        false, /* doVoxelGridDownDownSampling = */
        100, /* iFrameRate = */
        5, /* colorBitResolution = */
        true /* doColorEncoding = */
    }, {
    // PROFILE: HIGH_RES_OFFLINE_COMPRESSION_WITHOUT_COLOR
        0.0001, /* pointResolution = */
        0.0001, /* octreeResolution = */
        true, /* doVoxelGridDownDownSampling = */
        100, /* iFrameRate = */
        8, /* colorBitResolution = */
        false /* doColorEncoding = */
    }, {
    // PROFILE: HIGH_RES_OFFLINE_COMPRESSION_WITH_COLOR
        0.0001, /* pointResolution = */
        0.01, /* octreeResolution = */
        false, /* doVoxelGridDownDownSampling = */
        100, /* iFrameRate = */
        8, /* colorBitResolution = */
        true /* doColorEncoding = */
    }};

  }
}
