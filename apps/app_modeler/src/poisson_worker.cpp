/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012, Willow Garage, Inc.
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

#include <pcl/apps/modeler/poisson_worker.h>
#include <pcl/apps/modeler/parameter_dialog.h>
#include <pcl/apps/modeler/parameter.h>
#include <pcl/apps/modeler/cloud_mesh.h>
#include <pcl/apps/modeler/cloud_mesh_item.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/impl/poisson.hpp>

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::PoissonReconstructionWorker::PoissonReconstructionWorker(const QList<CloudMeshItem*>& cloud_mesh_items, QWidget* parent) :
  AbstractWorker(cloud_mesh_items, parent),
  depth_(NULL), solver_divide_(NULL), iso_divide_(NULL), degree_(NULL), scale_(NULL), samples_per_node_(NULL)
{

}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::PoissonReconstructionWorker::~PoissonReconstructionWorker(void)
{
  delete depth_;
  delete solver_divide_;
  delete iso_divide_;
  delete degree_;
  delete scale_;
  delete samples_per_node_;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::PoissonReconstructionWorker::setupParameters()
{
  pcl::Poisson<pcl::PointSurfel> poisson;
  depth_ = new IntParameter("Maximum Tree Depth",
    "Maximum depth of the tree that will be used for surface reconstruction. \
    Running at depth d corresponds to solving on a voxel grid whose resolution \
    is no larger than 2^d x 2^d x 2^d. Note that since the reconstructor adapts \
    the octree to the sampling density, the specified reconstruction depth \
    is only an upper bound.",
    poisson.getDepth(), 2, 16);

  solver_divide_ = new IntParameter("Solver Divide",
    "The depth at which a block Gauss-Seidel solver is used to solve the Laplacian \
    equation. Using this parameter helps reduce the memory overhead at the cost of \
    a small increase in reconstruction time. (In practice, we have found that for \
    reconstructions of depth 9 or higher a subdivide depth of 7 or 8 can greatly \
    reduce the memory usage.)",
    poisson.getSolverDivide(), 2, 16);

  iso_divide_ = new IntParameter("Iso Divide",
    "Depth at which a block iso-surface extractor should be used to extract the \
    iso-surface. Using this parameter helps reduce the memory overhead at the cost \
    of a small increase in extraction time. (In practice, we have found that for \
    reconstructions of depth 9 or higher a subdivide depth of 7 or 8 can greatly \
    reduce the memory usage.)",
    poisson.getIsoDivide(), 2, 16);

  degree_ = new IntParameter("Degree", "Degree", poisson.getDegree(), 1, 5);

  scale_ = new DoubleParameter("Scale",
    "The ratio between the diameter of the cube used for reconstruction and the \
    diameter of the samples' bounding cube.",
    poisson.getScale(), 0.1, 10.0, 0.01);

  samples_per_node_ = new DoubleParameter("Samples Per Node",
    "The minimum number of sample points that should fall within an octree node as \
    the octree construction is adapted to sampling density. For noise-free samples, small \
    values in the range [1.0 - 5.0] can be used. For more noisy samples, larger values in \
    the range [15.0 - 20.0] may be needed to provide a smoother, noise-reduced, reconstruction.",
    poisson.getScale(), 0.1, 10.0, 0.01);

  parameter_dialog_->addParameter(depth_);
  parameter_dialog_->addParameter(solver_divide_);
  parameter_dialog_->addParameter(iso_divide_);
  parameter_dialog_->addParameter(degree_);
  parameter_dialog_->addParameter(scale_);
  parameter_dialog_->addParameter(samples_per_node_);

  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::PoissonReconstructionWorker::processImpl(CloudMeshItem* cloud_mesh_item)
{
  pcl::Poisson<pcl::PointSurfel> poisson;
  poisson.setDegree(*depth_);
  poisson.setSolverDivide(*solver_divide_);
  poisson.setIsoDivide(*iso_divide_);
  poisson.setDegree(*degree_);
  poisson.setScale (float (*scale_));
  poisson.setScale (float (*samples_per_node_));

  poisson.setConfidence(true);
  poisson.setManifold(true);

  poisson.setInputCloud(cloud_mesh_item->getCloudMesh()->getCloud());

  CloudMesh::PointCloudPtr cloud(new CloudMesh::PointCloud());
  poisson.reconstruct(*cloud, cloud_mesh_item->getCloudMesh()->getPolygons());
  cloud_mesh_item->getCloudMesh()->getCloud() = cloud;

  return;
}
