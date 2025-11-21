#pragma once
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Geometry>

#include <QDialog>
#include <QTimer>

using CloudT = pcl::PointCloud<pcl::PointXYZ>;

namespace Ui {
class PCLViewerDialogUi;
}

class PCLViewerDialog : public QDialog {
  Q_OBJECT
  Ui::PCLViewerDialogUi* ui_;
  pcl::visualization::PCLVisualizer::Ptr vis_;

public:
  PCLViewerDialog(QWidget* parent = 0);

  void
  setPointClouds(CloudT::ConstPtr src_cloud,
                 CloudT::ConstPtr tgt_cloud,
                 const Eigen::Affine3f& t);

public Q_SLOTS:

  void
  refreshView();
};
