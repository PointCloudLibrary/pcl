#include <pcl/apps/pcl_viewer_dialog.h>

#include <ui_pcl_viewer_dialog.h>

#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkRenderWindow.h>

using namespace pcl::visualization;

PCLViewerDialog::PCLViewerDialog(QWidget* parent) : QDialog(parent)
{
  ui_ = new Ui::PCLViewerDialogUi;
  ui_->setupUi(this);

  // Set up the source window
#if VTK_MAJOR_VERSION > 8
  auto renderer = vtkSmartPointer<vtkRenderer>::New();
  auto renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
  renderWindow->AddRenderer(renderer);
  vis_.reset(new pcl::visualization::PCLVisualizer(renderer, renderWindow, "", false));
#else
  vis_.reset(new pcl::visualization::PCLVisualizer("", false));
#endif // VTK_MAJOR_VERSION > 8
  setRenderWindowCompat(*(ui_->qvtk_widget), *(vis_->getRenderWindow()));
  vis_->setupInteractor(getInteractorCompat(*(ui_->qvtk_widget)),
                        getRenderWindowCompat(*(ui_->qvtk_widget)));
}
void
PCLViewerDialog::setPointClouds(CloudT::ConstPtr src_cloud,
                                CloudT::ConstPtr tgt_cloud,
                                const Eigen::Affine3f& t)
{
  vis_->addPointCloud(tgt_cloud, "cloud_dst_");
  vis_->addPointCloud(src_cloud, "cloud_src_");
  vis_->updatePointCloudPose("cloud_src_", t);
  vis_->setPointCloudRenderingProperties(PCL_VISUALIZER_COLOR, 1, 0, 0, "cloud_src_");

  refreshView();
}

void
PCLViewerDialog::refreshView()
{
#if VTK_MAJOR_VERSION > 8
  ui_->qvtk_widget->renderWindow()->Render();
#else
  ui_->qvtk_widget->update();
#endif // VTK_MAJOR_VERSION > 8
}
