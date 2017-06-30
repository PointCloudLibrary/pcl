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

/** \brief GOOD: a Global Orthographic Object Descriptor for 3D object recognition and manipulation.
  * GOOD descriptor has been designed to be robust, descriptive and efficient to compute and use. 
  * It has two outstanding characteristics: 
  * 
  * (1) Providing a good trade-off among :
  *	- descriptiveness,
  *	- robustness,
  *	- computation time,
  *	- memory usage.
  * 
  * (2) Allowing concurrent object recognition and pose estimation for manipulation.
  * 
  * \note This is an implementation of the GOOD descriptor which has been presented in the following papers:
  * 
  *	[1] Kasaei, S. Hamidreza,  Ana Maria Tomé, Luís Seabra Lopes, Miguel Oliveira 
  *	"GOOD: A global orthographic object descriptor for 3D object recognition and manipulation." 
  *	Pattern Recognition Letters 83 (2016): 312-320.http://dx.doi.org/10.1016/j.patrec.2016.07.006
  *
  *	[2] Kasaei, S. Hamidreza, Luís Seabra Lopes, Ana Maria Tomé, Miguel Oliveira 
  * 	"An orthographic descriptor for 3D object learning and recognition." 
  *	2016 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Daejeon, 2016, 
  *	pp. 4158-4163. doi: 10.1109/IROS.2016.7759612
  * 
  * Please adequately refer to this work any time this code is being used by citing above papers.
  * If you do publish a paper where GOOD descriptor helped your research, we encourage you to cite the above papers in your publications.
  * 
  * \author Hamidreza Kasaei (Seyed.Hamidreza[at]ua[dot]pt)
  */

#include <pcl/features/good.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <boost/filesystem.hpp>
#include <pcl/io/auto_io.h>

typedef pcl::PointXYZRGBA PointT;  
typedef pcl::PointCloud<PointT>::Ptr PointCloudIn;


int main(int argc, char* argv[])
{  

  if (argc != 2) 
  {
    std::cout << "\n Syntax is: example_good_descriptor <path/file_name.pcd>" << std::endl;
    return 0;
  }
  
  std::string object_path =  argv[1];  
  pcl::PointCloud<PointT>::Ptr object (new pcl::PointCloud<PointT>);
  if (pcl::io::load(object_path, *object)==-1)  
  {
    std::cout << "\n file extension is not correct. Syntax is: example_good_descriptor <path/file_name.pcd>  or example_good_descriptor <path/file_name.ply>" << std::endl;
    return -1;
  }
  
  std::vector<PointCloudIn> vector_of_projected_views;
  PointCloudIn transformed_object (new pcl::PointCloud<PointT>);

  Eigen::Matrix4f transformation;
  pcl::PointXYZ center_of_bounding_box;
  pcl::PointXYZ bounding_box_dimensions;
  std::string order_of_projected_planes;
    
  // Setup the GOOD descriptor
  const int number_of_bins = 5;   
  const unsigned int lengh_of_descriptor = 3 * number_of_bins * number_of_bins; 
  pcl::PointCloud<pcl::Histogram<lengh_of_descriptor> > object_description;

  
  pcl::GOODEstimation<PointT, number_of_bins> test_GOOD_descriptor ; 
  test_GOOD_descriptor.setThreshold(0.0015);  
  // NOTE : GOOD descriptor can be setup in a line:  pcl::GOODEstimation<PointT, number_of_bins> test_GOOD_descriptor (0.0015); 
  test_GOOD_descriptor.setInputCloud(object); // pass original point cloud
  test_GOOD_descriptor.compute(object_description);   // Actually compute the GOOD discriptor for the given object
  
  // Printing GOOD_descriptor for the given point cloud
  pcl::Histogram<lengh_of_descriptor> GOOD_descriptor = object_description.points[0];
  std::cout <<"\n GOOD =" << GOOD_descriptor <<std::endl; 
  
  /*_________________________________________
  |                                          |
  | Functionalities for Object Manipulation  |
  |__________________________________________| */   
  // The following functinalities of GOOD descriptor are usefull for manipulation tasks:

  // Get objec point cloud in local reference frame
  test_GOOD_descriptor.getTransformedObject (transformed_object);

  // Get three orthographic projects and transformation matrix 
  test_GOOD_descriptor.getOrthographicProjections (vector_of_projected_views);  
  test_GOOD_descriptor.getTransformationMatrix (transformation);
  std::cout << "\n transofrmation matrix =\n"<<transformation << std::endl;
  
  // Get object bounding box information 
  test_GOOD_descriptor.getCenterOfObjectBoundingBox (center_of_bounding_box); 
  test_GOOD_descriptor.getObjectBoundingBoxDimensions(bounding_box_dimensions);
  std::cout<<"\n center_of_bounding_box = " << center_of_bounding_box<<std::endl;
  std::cout<<"\n bounding_box_dimensions = " << bounding_box_dimensions <<std::endl;
  
  // Get the order of the three projected planes 
  test_GOOD_descriptor.getOrderOfProjectedPlanes(order_of_projected_planes);
  std::cout << "\n order of projected planes = "<<order_of_projected_planes << std::endl;

  // NOTE : Pass the following point cloud to a pcl::visualization::PCLVisualizer forVisualizing the 
  //transformed object, local reference fram and three orthographic projections
  
  //pcl::PointCloud<PointT>::Ptr transformed_object_and_projected_views (new pcl::PointCloud<PointT>);
  //*transformed_object_and_projected_views += *vector_of_projected_views.at(0);
  //*transformed_object_and_projected_views += *vector_of_projected_views.at(1);
  //*transformed_object_and_projected_views += *vector_of_projected_views.at(2);
  //*transformed_object_and_projected_views += *transformed_object;    

  return 0;
}
