/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 *   * Neither the name of the copyright holder(s) nor the names of its
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
 *
 */

#include <iostream> 
#include <pcl/test/gtest.h>
#include <pcl/features/narf.h>
#include <pcl/common/eigen.h>
#include <pcl/common/angles.h> // for deg2rad

using namespace pcl;

TEST (PCL, Narf_save_load)
{
  Narf narf;
  getTransformation(1.0f, 2.0f, 3.0f, deg2rad(10.0f), deg2rad(20.0f), deg2rad(30.0f), narf.getTransformation());
  narf.getPosition() = narf.getTransformation().inverse().translation ();
  narf.getSurfacePatchPixelSize() = 10;
  narf.setSurfacePatch(new float[narf.getSurfacePatchPixelSize()*narf.getSurfacePatchPixelSize()]);
  for (int i=0; i<narf.getSurfacePatchPixelSize()*narf.getSurfacePatchPixelSize(); ++i)
    narf.getSurfacePatch()[i] = static_cast<float> (i);
  narf.getSurfacePatchWorldSize() = 0.5f;
  narf.getSurfacePatchRotation() = deg2rad(10.0f);
  narf.extractDescriptor(36);
  
  std::stringstream test_stream;
  narf.saveBinary(test_stream);
  
  Narf narf2;
  narf2.loadBinary(test_stream);
  
  //  EXPECT_EQ (narf.getTransformation().matrix(), narf2.getTransformation().matrix());
  // The above generates http://msdn.microsoft.com/en-us/library/sxe76d9e.aspx in VS2010
  // Therefore we use this:
  for (Eigen::Index i = 0; i < narf.getTransformation().matrix().size(); ++i)
    EXPECT_EQ (narf.getTransformation().data()[i], narf2.getTransformation().data()[i]);
  EXPECT_EQ (narf.getPosition(), narf2.getPosition());
  EXPECT_EQ (narf.getSurfacePatchPixelSize(), narf2.getSurfacePatchPixelSize());
  for (int i=0; i<narf.getSurfacePatchPixelSize()*narf.getSurfacePatchPixelSize(); ++i)
    EXPECT_EQ (narf.getSurfacePatch()[i], narf2.getSurfacePatch()[i]);
  EXPECT_EQ (narf.getSurfacePatchWorldSize(), narf2.getSurfacePatchWorldSize());
  EXPECT_EQ (narf.getSurfacePatchRotation(), narf2.getSurfacePatchRotation());
  EXPECT_EQ (narf.getDescriptorSize(), narf2.getDescriptorSize());
  for (int i=0; i<narf.getDescriptorSize(); ++i)
    EXPECT_EQ (narf.getDescriptor()[i], narf2.getDescriptor()[i]);
}

/* ---[ */
int
  main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
