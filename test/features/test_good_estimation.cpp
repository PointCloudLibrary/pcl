/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2017-, Open Perception, Inc.
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
 */

#include <gtest/gtest.h>
#include <pcl/features/good.h>
#include <pcl/io/pcd_io.h>


pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (PCL, GOODEstimation)
{
  const int NUMBER_OF_BINS = 15;     
  typedef pcl::GOODEstimation<pcl::PointXYZRGBA, NUMBER_OF_BINS>::Descriptor Descriptor;
  pcl::PointCloud<Descriptor> object_description;   
  pcl::GOODEstimation<pcl::PointXYZRGBA, NUMBER_OF_BINS> test_GOOD_descriptor;   
  test_GOOD_descriptor.setThreshold (0.0015);  
  test_GOOD_descriptor.setInputCloud (cloud); // pass original point cloud
  test_GOOD_descriptor.compute (object_description); // Actually compute the GOOD discriptor for the given object
  
  const float expected_values [675] =
  { 0, 0, 0, 0, 0, 0, 0, 0.000875657, 0, 0.000218914, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.00627554, 0.00861062, 0.0105079, 0.0110187, 0.00634851, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.000364857, 
    0.0140835, 0.0164915, 0.015324, 0.0158348, 0.0134267, 0.00218914, 0, 0, 0, 0, 0, 0, 0, 0.00131349, 0.00890251, 0.0104349, 0.0172942, 0.0170753, 0.0165645, 0.0156888, 0.010289, 
    0.000218914, 0, 0, 0, 0, 0, 0, 0.00496206, 0.0106538, 0.0137186, 0.0138646, 0.0140105, 0.0137186, 0.0116754, 0.0124051, 0.00211617, 0, 0, 0, 0, 0, 0, 0.00488908, 0.00992411, 
    0.0106538, 0.0116025, 0.0132078, 0.0121133, 0.0107998, 0.0111646, 0.000729714, 0, 0, 0, 0, 0, 0, 0.00372154, 0.00882954, 0.0106538, 0.0119673, 0.0119673, 0.0119673, 0.0108727, 
    0.0110187, 0.00167834, 0, 0, 0, 0, 0, 0, 0.00182428, 0.00904845, 0.0100701, 0.0108727, 0.0114565, 0.0110917, 0.0100701, 0.00941331, 0.00248103, 0, 0, 0, 0, 0, 0, 0.00313777, 
    0.00846468, 0.00934034, 0.0103619, 0.0106538, 0.0113106, 0.0104349, 0.010143, 0.00211617, 0, 0, 0, 0, 0, 0, 0.00445125, 0.0081728, 0.00890251, 0.010289, 0.0109457, 0.010143, 
    0.00941331, 0.00846468, 0.00240806, 0, 0, 0, 0, 0, 0, 0.00328371, 0.00839171, 0.00875657, 0.00941331, 0.0105809, 0.00934034, 0.00977817, 0.00904845, 0.00109457, 0, 0, 0, 0, 0, 0,
    0.00291886, 0.00685931, 0.00846468, 0.00926737, 0.00955925, 0.00955925, 0.00926737, 0.00831874, 0.00226211, 0, 0, 0, 0, 0, 0, 0.00350263, 0.00744308, 0.00795388, 0.00882954, 
    0.0091944, 0.00912142, 0.00890251, 0.00795388, 0.00160537, 0, 0, 0, 0, 0, 0, 0.00131349, 0.00685931, 0.00751605, 0.00861062, 0.00912142, 0.00861062, 0.00751605, 0.00853765, 
    0.0020432, 0, 0, 0, 0, 0, 0, 0.00109457, 0.00226211, 0.00328371, 0.0056188, 0.00539988, 0.00437828, 0.00350263, 0.00321074, 0.000729714, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.000218914, 0.00649445, 0.0267075, 0.00299183, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0.00175131, 0.0290426, 0.0513719, 0.0140105, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.0139375, 0.0768389, 0.0391127, 0.000218914, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0.0110917, 0.0218914, 0.071293, 0.0488179, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.00277291, 0.0197023, 0.0111646, 0.086617, 0.0396235, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.017951, 
    0.00875657, 0.0480881, 0.0799766, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.000437828, 0.015324, 0.0262697, 0.0934034, 0.00248103, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.00328371, 0.0820198, 
    0.0268535, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.00218914, 0.0168564, 0.000437828, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.00109457, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.000145943, 0.0186807, 0.015397, 0.00853765, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.000291886, 0.0172942, 0.0289696, 0.0289696, 0.00218914, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.00350263, 0.0259048, 0.033275, 0.0311588, 0.00394046, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0.00248103, 0.0155429, 0.0278751, 0.0296264, 0.0215995, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.00467017, 0.0115295, 0.0233508, 0.0261967, 0.0193374, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0.0061296, 0.00948628, 0.0244454, 0.0248103, 0.017805, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.00299183, 0.0154699, 0.0213076, 0.0231319, 0.0134267, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0.00554583, 0.0163456, 0.0206509, 0.0247373, 0.0086836, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7.29714e-05, 0.00788091, 0.0161997, 0.0188266, 0.0264886, 0.00372154, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0.000437828, 0.00656743, 0.0173672, 0.0184618, 0.0267075, 0.000145943, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.000875657, 0.00576474, 0.017878, 0.0189726, 0.022986, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0.00335668, 0.00554583, 0.0168564, 0.020286, 0.0184618, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.00211617, 0.00569177, 0.0188266, 0.0183888, 0.0151051, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0.000218914, 0.00138646, 0.00342966, 0.00722417, 0.00934034, 0.00788091, 0, 0, 0, 0, 0, 0
  };

  EXPECT_EQ (object_description.points[0].descriptorSize (), 675);
  for (size_t i = 0; i < size_t (object_description.points[0].descriptorSize ()); ++i)
  {
    EXPECT_NEAR (object_description.points[0].histogram[i], expected_values[i], 1e-5);
  }
  
  Eigen::Matrix4f transformation = test_GOOD_descriptor.getTransformationMatrix ();
  Eigen::Matrix4f expected_transformation_values;
  expected_transformation_values << 
  -0.0108418, 0.887046, 0.461555, -0.236646,
  0.998495, -0.0152106, 0.0526872, 0.0132545,
  0.0537565, 0.461432, -0.885546, 0.75173,
  0, 0, 0, 1;

  for (int i = 0; i < transformation.rows(); ++i)
  {
    for (int j = 0; j < transformation.cols (); ++j)
    {
      EXPECT_NEAR (transformation (i, j), expected_transformation_values (i, j), 1e-5);
    }
  }
    
  pcl::PointXYZ center_of_bounding_box = test_GOOD_descriptor.getCenterOfObjectBoundingBox (); 
  EXPECT_EQ (-0.0624728, center_of_bounding_box.x);
  EXPECT_EQ (-0.140744, center_of_bounding_box.y);
  EXPECT_EQ (0.790559, center_of_bounding_box.z);

  Eigen::Vector4f bounding_box_dimensions = test_GOOD_descriptor.getObjectBoundingBoxDimensions ();
  EXPECT_EQ (0.257679, bounding_box_dimensions(0));
  EXPECT_EQ (0.151612, bounding_box_dimensions(1));
  EXPECT_EQ (0.111211, bounding_box_dimensions(2));
  EXPECT_EQ (0, bounding_box_dimensions(3));
  
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* ---[ */
int
main (int argc, char** argv)
{
  if (argc < 2)
  {
    std::cerr << "No test file given. Please download `milk.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }     
       
  if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> (argv[1], *cloud) == -1)
  {
    std::cerr << "Failed to read test file. Please download `milk.pcd` and pass its path to the test." << std::endl;
    return (-1);
  }
  
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
