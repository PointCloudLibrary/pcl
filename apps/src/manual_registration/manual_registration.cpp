/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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
 * $Id: $
 *
 * \author: Koen Buys - KU Leuven
 */

#include <pcl/apps/manual_registration.h>
#include <pcl/apps/pcl_viewer_dialog.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h> // for loadPCDFile
#include <pcl/registration/gicp.h>

#include <QApplication>
#include <QEvent>
#include <QMutexLocker>
#include <QObject>

#include <vtkVersion.h>
#if VTK_MAJOR_VERSION >= 9 || (VTK_MAJOR_VERSION == 8 && VTK_MINOR_VERSION >= 2)
#define HAS_QVTKOPENGLWINDOW_H
#include <QVTKOpenGLWindow.h>
#endif
#include <ui_manual_registration.h>

#include <vtkCamera.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkRendererCollection.h>
#include <vtkRenderWindow.h>

using namespace pcl;
using namespace pcl::visualization;
using std::string;
using std::to_string;

//////////////////////////////////////////////////////////////////////////////////////////////////////////
ManualRegistration::ManualRegistration(float voxel_size) : voxel_size_(voxel_size)
{
  ui_ = new Ui::MainWindow;
  ui_->setupUi(this);

  this->setWindowTitle("PCL Manual Registration");

  // Set up the source window
#if VTK_MAJOR_VERSION > 8
  auto renderer_src = vtkSmartPointer<vtkRenderer>::New();
  auto renderWindow_src = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
  renderWindow_src->AddRenderer(renderer_src);
  vis_src_.reset(
      new pcl::visualization::PCLVisualizer(renderer_src, renderWindow_src, "", false));
#else
  vis_src_.reset(new pcl::visualization::PCLVisualizer("", false));
#endif // VTK_MAJOR_VERSION > 8
  setRenderWindowCompat(*(ui_->qvtk_widget_src), *(vis_src_->getRenderWindow()));
  vis_src_->setupInteractor(getInteractorCompat(*(ui_->qvtk_widget_src)),
                            getRenderWindowCompat(*(ui_->qvtk_widget_src)));

  vis_src_->getInteractorStyle()->setKeyboardModifier(
      pcl::visualization::INTERACTOR_KB_MOD_SHIFT);

  vis_src_->registerPointPickingCallback(&ManualRegistration::SrcPointPickCallback,
                                         *this);

  // Set up the destination window
#if VTK_MAJOR_VERSION > 8
  auto renderer_dst = vtkSmartPointer<vtkRenderer>::New();
  auto renderWindow_dst = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
  renderWindow_dst->AddRenderer(renderer_dst);
  vis_dst_.reset(
      new pcl::visualization::PCLVisualizer(renderer_dst, renderWindow_dst, "", false));
#else
  vis_dst_.reset(new pcl::visualization::PCLVisualizer("", false));
#endif // VTK_MAJOR_VERSION > 8
  setRenderWindowCompat(*(ui_->qvtk_widget_dst), *(vis_dst_->getRenderWindow()));
  vis_dst_->setupInteractor(getInteractorCompat(*(ui_->qvtk_widget_dst)),
                            getRenderWindowCompat(*(ui_->qvtk_widget_dst)));

  vis_dst_->getInteractorStyle()->setKeyboardModifier(
      pcl::visualization::INTERACTOR_KB_MOD_SHIFT);

  vis_dst_->registerPointPickingCallback(&ManualRegistration::DstPointPickCallback,
                                         *this);
  // Render view
  refreshView();

  // Connect all buttons
  connect(ui_->confirmSrcPointButton,
          SIGNAL(clicked()),
          this,
          SLOT(confirmSrcPointPressed()));
  connect(ui_->confirmDstPointButton,
          SIGNAL(clicked()),
          this,
          SLOT(confirmDstPointPressed()));
  connect(ui_->calculateButton, SIGNAL(clicked()), this, SLOT(calculatePressed()));
  connect(ui_->clearButton, SIGNAL(clicked()), this, SLOT(clearPressed()));
  connect(ui_->orthoButton, SIGNAL(stateChanged(int)), this, SLOT(orthoChanged(int)));
  connect(ui_->applyTransformButton,
          SIGNAL(clicked()),
          this,
          SLOT(applyTransformPressed()));
  connect(ui_->refineButton, SIGNAL(clicked()), this, SLOT(refinePressed()));
}

void
ManualRegistration::SrcPointPickCallback(
    const pcl::visualization::PointPickingEvent& event, void*)
{
  // Check to see if we got a valid point. Early exit.
  int idx = event.getPointIndex();
  if (idx == -1)
    return;

  // Get the point that was picked
  event.getPoint(src_point_.x, src_point_.y, src_point_.z);
  PCL_INFO("Src Window: Clicked point %d with X:%f Y:%f Z:%f\n",
           idx,
           src_point_.x,
           src_point_.y,
           src_point_.z);
  src_point_selected_ = true;
}

void
ManualRegistration::DstPointPickCallback(
    const pcl::visualization::PointPickingEvent& event, void*)
{
  // Check to see if we got a valid point. Early exit.
  int idx = event.getPointIndex();
  if (idx == -1)
    return;

  // Get the point that was picked
  event.getPoint(dst_point_.x, dst_point_.y, dst_point_.z);
  PCL_INFO("Dst Window: Clicked point %d with X:%f Y:%f Z:%f\n",
           idx,
           dst_point_.x,
           dst_point_.y,
           dst_point_.z);
  dst_point_selected_ = true;
}

void
ManualRegistration::confirmSrcPointPressed()
{
  if (src_point_selected_) {
    src_pc_.push_back(src_point_);
    PCL_INFO("Selected %zu source points\n", static_cast<std::size_t>(src_pc_.size()));
    src_point_selected_ = false;
    src_pc_.width = src_pc_.size();
    const string annotation = "marker-" + to_string(annotations_src_.size());
    vis_src_->addSphere(src_point_, 0.02, annotation);
    vis_src_->setShapeRenderingProperties(PCL_VISUALIZER_OPACITY, 0.2, annotation);
    vis_src_->setShapeRenderingProperties(
        PCL_VISUALIZER_COLOR, 0.5, 0.25, 0.25, annotation);
    vis_src_->getShapeActorMap()->at(annotation)->SetPickable(false);
    annotations_src_.emplace(annotation);

    refreshView();
  }
  else {
    PCL_INFO("Please select a point in the source window first\n");
  }
}

void
ManualRegistration::confirmDstPointPressed()
{
  if (dst_point_selected_) {
    dst_pc_.push_back(dst_point_);
    PCL_INFO("Selected %zu destination points\n",
             static_cast<std::size_t>(dst_pc_.size()));
    dst_point_selected_ = false;
    dst_pc_.width = dst_pc_.size();

    const string annotation = "marker-" + std::to_string(annotations_dst_.size());
    vis_dst_->addSphere(dst_point_, 0.02, annotation);
    vis_dst_->setShapeRenderingProperties(PCL_VISUALIZER_OPACITY, 0.2, annotation);
    vis_dst_->setShapeRenderingProperties(
        PCL_VISUALIZER_COLOR, 0.5, 0.25, 0.25, annotation);
    vis_dst_->getShapeActorMap()->at(annotation)->SetPickable(false);
    annotations_dst_.emplace(annotation);

    refreshView();
  }
  else {
    PCL_INFO("Please select a point in the destination window first\n");
  }
}

void
ManualRegistration::calculatePressed()
{
  if (dst_pc_.size() != src_pc_.size()) {
    PCL_INFO("You haven't selected an equal amount of points, please do so\n");
    return;
  }
  pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> tfe;
  tfe.estimateRigidTransformation(src_pc_, dst_pc_, transform_);
  PCL_INFO_STREAM("Calculated transform:\n" << transform_ << std::endl);
}

void
ManualRegistration::clearPressed()
{
  PCL_INFO("Clearing points.");
  dst_point_selected_ = false;
  src_point_selected_ = false;
  src_pc_.clear();
  dst_pc_.clear();
  src_pc_.height = 1;
  src_pc_.width = 0;
  dst_pc_.height = 1;
  dst_pc_.width = 0;

  for (const string& annotation : annotations_src_) {
    vis_src_->removeShape(annotation);
  }
  annotations_src_.clear();

  for (const string& annotation : annotations_dst_) {
    vis_dst_->removeShape(annotation);
  }
  annotations_dst_.clear();

  refreshView();
}

void
ManualRegistration::orthoChanged(int state)
{
  PCL_INFO("Ortho state %d\n", state);
  if (state == 0) // Not selected
  {
    vis_src_->getRenderWindow()
        ->GetRenderers()
        ->GetFirstRenderer()
        ->GetActiveCamera()
        ->SetParallelProjection(0);
    vis_dst_->getRenderWindow()
        ->GetRenderers()
        ->GetFirstRenderer()
        ->GetActiveCamera()
        ->SetParallelProjection(0);
  }
  if (state == 2) // Selected
  {
    vis_src_->getRenderWindow()
        ->GetRenderers()
        ->GetFirstRenderer()
        ->GetActiveCamera()
        ->SetParallelProjection(1);
    vis_dst_->getRenderWindow()
        ->GetRenderers()
        ->GetFirstRenderer()
        ->GetActiveCamera()
        ->SetParallelProjection(1);
  }

  refreshView();
}

// TODO
void
ManualRegistration::applyTransformPressed()
{
  PCLViewerDialog* diag = new PCLViewerDialog(this);
  diag->setModal(true);
  diag->setGeometry(this->x(), this->y(), this->width(), this->height());
  diag->setPointClouds(cloud_src_, cloud_dst_, Eigen::Affine3f(transform_));
  diag->show();
}

void
ManualRegistration::refinePressed()
{
  PCL_INFO("Refining transform ...\n");
  VoxelGrid<PointT> grid_filter;
  grid_filter.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
  PointCloud<PointT>::Ptr src_copy{new PointCloud<PointT>(*cloud_src_)};
  PointCloud<PointT>::Ptr dst_copy{new PointCloud<PointT>(*cloud_dst_)};
  grid_filter.setInputCloud(src_copy);
  grid_filter.filter(*src_copy);
  grid_filter.setInputCloud(dst_copy);
  grid_filter.filter(*dst_copy);

  using ICP = GeneralizedIterativeClosestPoint<PointT, PointT>;
  ICP::Ptr icp = pcl::make_shared<ICP>();
  icp->setInputSource(src_copy);
  icp->setInputTarget(dst_copy);

  icp->setMaximumIterations(100);
  icp->setMaxCorrespondenceDistance(0.3);
  icp->setEuclideanFitnessEpsilon(0.01);
  icp->setTransformationEpsilon(0.01);
  icp->setTransformationRotationEpsilon(0.01);
  PointCloud<PointT>::Ptr aligned{new PointCloud<PointT>};
  icp->align(*aligned, transform_);
  transform_ = icp->getFinalTransformation();

  PCL_INFO_STREAM("Calculated transform:\n" << transform_ << std::endl);
}

void
ManualRegistration::refreshView()
{
#if VTK_MAJOR_VERSION > 8
  ui_->qvtk_widget_dst->renderWindow()->Render();
  ui_->qvtk_widget_src->renderWindow()->Render();
#else
  ui_->qvtk_widget_dst->update();
  ui_->qvtk_widget_src->update();
#endif // VTK_MAJOR_VERSION > 8
}

void
print_usage()
{
  PCL_INFO("manual_registration cloud1.pcd cloud2.pcd\n");
  PCL_INFO("\t cloud1 \t source cloud\n");
  PCL_INFO("\t cloud2 \t destination cloud\n");
  PCL_INFO("\t voxel_size \t voxel size for automatic refinement\n");
}

int
main(int argc, char** argv)
{
#ifdef HAS_QVTKOPENGLWINDOW_H
  QSurfaceFormat::setDefaultFormat(QVTKOpenGLWindow::defaultFormat());
#endif
  QApplication app(argc, argv);

  pcl::PointCloud<PointT>::Ptr cloud_src(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_dst(new pcl::PointCloud<PointT>);

  if (argc < 4) {
    PCL_ERROR("Incorrect usage\n");
    print_usage();
    return -1;
  }

  // TODO do this with PCL console
  if (pcl::io::loadPCDFile<PointT>(argv[1], *cloud_src) == -1) //* load the file
  {
    PCL_ERROR("Couldn't read file %s \n", argv[1]);
    return -1;
  }
  if (pcl::io::loadPCDFile<PointT>(argv[2], *cloud_dst) == -1) //* load the file
  {
    PCL_ERROR("Couldn't read file %s \n", argv[2]);
    return -1;
  }

  const float voxel_size = std::atof(argv[3]);

  ManualRegistration man_reg(voxel_size);

  man_reg.setSrcCloud(cloud_src);
  man_reg.setDstCloud(cloud_dst);

  man_reg.show();

  return QApplication::exec();
}
