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
 */

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Estimate a rigid rotation transformation between a source and a target point cloud using SVD.
  * \param cloud_src the source point cloud dataset
  * \param cloud_tgt the target point cloud dataset
  * \param transformation_matrix the resultant transformation matrix
  */
template <typename PointSource, typename PointTarget> void
  pcl::estimateRigidTransformationGICP (const pcl::PointCloud<PointSource> &cloud_src, 
                                       const pcl::PointCloud<PointTarget> &cloud_tgt, 
                                       Eigen::Matrix4f &transformation_matrix)
{
  ROS_ASSERT (cloud_src.points.size () == cloud_tgt.points.size ());

  // <cloud_src,cloud_src> is the source dataset
  transformation_matrix.setIdentity ();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Estimate a rigid rotation transformation between a source and a target point cloud using SVD.
  * \param cloud_src the source point cloud dataset
  * \param indices_src the vector of indices describing the points of interest in \a cloud_src
  * \param cloud_tgt the target point cloud dataset
  * \param indices_tgt the vector of indices describing the correspondences of the interst points from \a indices_src
  * \param transformation_matrix the resultant transformation matrix
  */
template <typename PointSource, typename PointTarget> void
  pcl::estimateRigidTransformationGICP (const pcl::PointCloud<PointSource> &cloud_src, 
                                       const std::vector<int> &indices_src, 
                                       const pcl::PointCloud<PointTarget> &cloud_tgt, 
                                       const std::vector<int> &indices_tgt, 
                                       Eigen::Matrix4f &transformation_matrix)
{
  assert (indices_src.size () == indices_tgt.size ());

  // <cloud_src,cloud_src> is the source dataset
  transformation_matrix.setIdentity ();

}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Estimate a rigid rotation transformation between a source and a target point cloud using SVD.
  * \param cloud_src the source point cloud dataset
  * \param indices_src the vector of indices describing the points of interest in \a cloud_src
  * \param cloud_tgt the target point cloud dataset
  * \param transformation_matrix the resultant transformation matrix
  */
template <typename PointSource, typename PointTarget> void
  pcl::estimateRigidTransformationGICP (const pcl::PointCloud<PointSource> &cloud_src, 
                                       const std::vector<int> &indices_src, 
                                       const pcl::PointCloud<PointTarget> &cloud_tgt, 
                                       Eigen::Matrix4f &transformation_matrix)
{
  ROS_ASSERT (indices_src.size () == cloud_tgt.points.size ());

  // <cloud_src,cloud_src> is the source dataset
  transformation_matrix.setIdentity ();

}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Rigid transformation computation method.
  * \param output the transformed input point cloud dataset using the rigid transformation found
  */
template <typename PointSource, typename PointTarget> void
  pcl::GeneralizedIterativeClosestPoint<PointSource, PointTarget>::computeTransformation (PointCloudSource &output)
{
  dgc::gicp::GICPPointSet p1, p2;
  dgc_transform_t t_base, t0, t1;
  double gicp_epsilon = 1e-3;

  p1.SetDebug(false);
  p2.SetDebug(false);

  // set up the transformations
  dgc_transform_identity(t_base);
  dgc_transform_identity(t0);
  dgc_transform_identity(t1);

  // load points
  dgc::gicp::GICPPoint pt;
  pt.range = -1;
  for(int k = 0; k < 3; k++) {
    for(int l = 0; l < 3; l++) {
      pt.C[k][l] = (k == l)?1:0;
    }
  }    
  p1.point_.resize(input_->points.size());
  p2.point_.resize(target_->points.size());
  for(unsigned int i = 0; i < input_->points.size(); ++i) {
    pt.x = input_->points[i].x;
    pt.y = input_->points[i].y;
    pt.z = input_->points[i].z;
    p1.point_[i] = pt;
  }
  for(unsigned int i = 0; i < target_->points.size(); ++i) {
    pt.x = target_->points[i].x;
    pt.y = target_->points[i].y;
    pt.z = target_->points[i].z;
    p2.point_[i] = pt;
  }

  // setting up matrices and trees
  p1.SetGICPEpsilon(gicp_epsilon);
  p2.SetGICPEpsilon(gicp_epsilon);  
  p1.BuildKDTree();
  p1.ComputeMatrices();
  p2.BuildKDTree();
  p2.ComputeMatrices();

  // register
  //int iterations = p2.AlignScan(&p1, t_base, t1, max_distance);
  p2.AlignScan(&p1, t_base, t1, max_distance_);

  // transform point cloud
  for (unsigned int i = 0; i < p1.point_.size(); ++i) 
    dgc_transform_point(&p1[i].x, &p1[i].y, &p1[i].z, t1);

  // write output cloud
  output.points.resize(p1.point_.size());
  for(unsigned int i = 0; i < p1.point_.size(); ++i) {
    output.points[i].x = p1.point_[i].x;
    output.points[i].y = p1.point_[i].y;
    output.points[i].z = p1.point_[i].z;
  }

  Eigen::Affine3f transformation_gicp;
  double tx, ty, tz, rx, ry, rz;
  dgc_transform_get_translation(t1, &tx, &ty, &tz);
  dgc_transform_get_rotation(t1, &rx, &ry, &rz);
  

  // compute final transform (hehe)

  float A=cosf(rz),  B=sinf(rz),  C=cosf(ry), D=sinf(ry),
    E=cosf(rx), F=sinf(rx), DE=D*E,        DF=D*F;
  final_transformation_(0,0) = A*C;  final_transformation_(0,1) = A*DF - B*E;  final_transformation_(0,2) = B*F + A*DE;  final_transformation_(0,3) = tx;
  final_transformation_(1,0) = B*C;  final_transformation_(1,1) = A*E + B*DF;  final_transformation_(1,2) = B*DE - A*F;  final_transformation_(1,3) = ty;
  final_transformation_(2,0) = -D;   final_transformation_(2,1) = C*F;         final_transformation_(2,2) = C*E;         final_transformation_(2,3) = tz;
  final_transformation_(3,0) = 0;    final_transformation_(3,1) = 0;           final_transformation_(3,2) = 0;           final_transformation_(3,3) = 1;

  //  transformation_gicp = pcl::getTransformation( (float)(tx), (float)(ty), (float)(tz), (float)(rx), (float)(ry), (float)(rz));
  //  final_transformation_ = transformation_gicp; //transformation_ * final_transformation_;

}

