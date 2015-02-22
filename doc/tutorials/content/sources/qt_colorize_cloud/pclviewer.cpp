#include "pclviewer.h"
#include "../build/ui_pclviewer.h"

PCLViewer::PCLViewer (QWidget *parent) :
    QMainWindow (parent),
    ui (new Ui::PCLViewer),
    filtering_axis_ (1),  // = y
    color_mode_ (4)  // = Rainbow
{
  ui->setupUi (this);
  this->setWindowTitle ("PCL viewer");

  // Setup the cloud pointer
  cloud_.reset (new PointCloudT);
  // The number of points in the cloud
  cloud_->resize (500);

  // Fill the cloud with random points
  for (size_t i = 0; i < cloud_->points.size (); ++i)
  {
    cloud_->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud_->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud_->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }

  // Set up the QVTK window
  viewer_.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
  viewer_->setBackgroundColor (0.1, 0.1, 0.1);
  ui->qvtkWidget->SetRenderWindow (viewer_->getRenderWindow ());
  viewer_->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
  ui->qvtkWidget->update ();

  // Connect "Load" and "Save" buttons and their functions
  connect (ui->pushButton_load, SIGNAL(clicked ()), this, SLOT(loadFileButtonPressed ()));
  connect (ui->pushButton_save, SIGNAL(clicked ()), this, SLOT(saveFileButtonPressed ()));

  // Connect X,Y,Z radio buttons and their functions
  connect (ui->radioButton_x, SIGNAL(clicked ()), this, SLOT(axisChosen ()));
  connect (ui->radioButton_y, SIGNAL(clicked ()), this, SLOT(axisChosen ()));
  connect (ui->radioButton_z, SIGNAL(clicked ()), this, SLOT(axisChosen ()));

  connect (ui->radioButton_BlueRed, SIGNAL(clicked ()), this, SLOT(lookUpTableChosen()));
  connect (ui->radioButton_GreenMagenta, SIGNAL(clicked ()), this, SLOT(lookUpTableChosen()));
  connect (ui->radioButton_WhiteRed, SIGNAL(clicked ()), this, SLOT(lookUpTableChosen()));
  connect (ui->radioButton_GreyRed, SIGNAL(clicked ()), this, SLOT(lookUpTableChosen()));
  connect (ui->radioButton_Rainbow, SIGNAL(clicked ()), this, SLOT(lookUpTableChosen()));

  // Color the randomly generated cloud
  colorCloudDistances ();
  viewer_->addPointCloud (cloud_, "cloud");
  viewer_->resetCamera ();
  ui->qvtkWidget->update ();
}

PCLViewer::~PCLViewer ()
{
  delete ui;
}

void
PCLViewer::loadFileButtonPressed ()
{
  // You might want to change "/home/" if you're not on an *nix platform
  QString filename = QFileDialog::getOpenFileName (this, tr ("Open point cloud"), "/home/", tr ("Point cloud data (*.pcd *.ply)"));

  PCL_INFO("File chosen: %s\n", filename.toStdString ().c_str ());
  PointCloudT::Ptr cloud_tmp (new PointCloudT);

  if (filename.isEmpty ())
    return;

  int return_status;
  if (filename.endsWith (".pcd", Qt::CaseInsensitive))
    return_status = pcl::io::loadPCDFile (filename.toStdString (), *cloud_tmp);
  else
    return_status = pcl::io::loadPLYFile (filename.toStdString (), *cloud_tmp);

  if (return_status != 0)
  {
    PCL_ERROR("Error reading point cloud %s\n", filename.toStdString ().c_str ());
    return;
  }

  // If point cloud contains NaN values, remove them before updating the visualizer point cloud
  if (cloud_tmp->is_dense)
    pcl::copyPointCloud (*cloud_tmp, *cloud_);
  else
  {
    PCL_WARN("Cloud is not dense! Non finite points will be removed\n");
    std::vector<int> vec;
    pcl::removeNaNFromPointCloud (*cloud_tmp, *cloud_, vec);
  }

  colorCloudDistances ();
  viewer_->updatePointCloud (cloud_, "cloud");
  viewer_->resetCamera ();
  ui->qvtkWidget->update ();
}

void
PCLViewer::saveFileButtonPressed ()
{
  // You might want to change "/home/" if you're not on an *nix platform
  QString filename = QFileDialog::getSaveFileName(this, tr ("Open point cloud"), "/home/", tr ("Point cloud data (*.pcd *.ply)"));

  PCL_INFO("File chosen: %s\n", filename.toStdString ().c_str ());

  if (filename.isEmpty ())
    return;

  int return_status;
  if (filename.endsWith (".pcd", Qt::CaseInsensitive))
    return_status = pcl::io::savePCDFileBinary (filename.toStdString (), *cloud_);
  else if (filename.endsWith (".ply", Qt::CaseInsensitive))
    return_status = pcl::io::savePLYFileBinary (filename.toStdString (), *cloud_);
  else
  {
    filename.append(".ply");
    return_status = pcl::io::savePLYFileBinary (filename.toStdString (), *cloud_);
  }

  if (return_status != 0)
  {
    PCL_ERROR("Error writing point cloud %s\n", filename.toStdString ().c_str ());
    return;
  }
}

void
PCLViewer::axisChosen ()
{
  // Only 1 of the button can be checked at the time (mutual exclusivity) in a group of radio buttons
  if (ui->radioButton_x->isChecked ())
  {
    PCL_INFO("x filtering chosen\n");
    filtering_axis_ = 0;
  }
  else if (ui->radioButton_y->isChecked ())
  {
    PCL_INFO("y filtering chosen\n");
    filtering_axis_ = 1;
  }
  else
  {
    PCL_INFO("z filtering chosen\n");
    filtering_axis_ = 2;
  }

  colorCloudDistances ();
  viewer_->updatePointCloud (cloud_, "cloud");
  ui->qvtkWidget->update ();
}

void
PCLViewer::lookUpTableChosen ()
{
  // Only 1 of the button can be checked at the time (mutual exclusivity) in a group of radio buttons
  if (ui->radioButton_BlueRed->isChecked ())
  {
    PCL_INFO("Blue -> Red LUT chosen\n");
    color_mode_ = 0;
  }
  else if (ui->radioButton_GreenMagenta->isChecked ())
  {
    PCL_INFO("Green -> Magenta LUT chosen\n");
    color_mode_ = 1;
  }
  else if (ui->radioButton_WhiteRed->isChecked ())
  {
    PCL_INFO("White -> Red LUT chosen\n");
    color_mode_ = 2;
  }
  else if (ui->radioButton_GreyRed->isChecked ())
  {
    PCL_INFO("Grey / Red LUT chosen\n");
    color_mode_ = 3;
  }
  else
  {
    PCL_INFO("Rainbow LUT chosen\n");
    color_mode_ = 4;
  }

  colorCloudDistances ();
  viewer_->updatePointCloud (cloud_, "cloud");
  ui->qvtkWidget->update ();
}

void
PCLViewer::colorCloudDistances ()
{
  // Find the minimum and maximum values along the selected axis
  double min, max;
  // Set an initial value
  switch (filtering_axis_)
  {
    case 0:  // x
      min = cloud_->points[0].x;
      max = cloud_->points[0].x;
      break;
    case 1:  // y
      min = cloud_->points[0].y;
      max = cloud_->points[0].y;
      break;
    default:  // z
      min = cloud_->points[0].z;
      max = cloud_->points[0].z;
      break;
  }

  // Search for the minimum/maximum
  for (PointCloudT::iterator cloud_it = cloud_->begin (); cloud_it != cloud_->end (); ++cloud_it)
  {
    switch (filtering_axis_)
    {
      case 0:  // x
        if (min > cloud_it->x)
          min = cloud_it->x;

        if (max < cloud_it->x)
          max = cloud_it->x;
        break;
      case 1:  // y
        if (min > cloud_it->y)
          min = cloud_it->y;

        if (max < cloud_it->y)
          max = cloud_it->y;
        break;
      default:  // z
        if (min > cloud_it->z)
          min = cloud_it->z;

        if (max < cloud_it->z)
          max = cloud_it->z;
        break;
    }
  }

  // Compute LUT scaling to fit the full histogram spectrum
  double lut_scale = 255.0 / (max - min);  // max is 255, min is 0

  if (min == max)  // In case the cloud is flat on the chosen direction (x,y or z)
    lut_scale = 1.0;  // Avoid rounding error in boost

  for (PointCloudT::iterator cloud_it = cloud_->begin (); cloud_it != cloud_->end (); ++cloud_it)
  {
    int value;
    switch (filtering_axis_)
    {
      case 0:  // x
        value = boost::math::iround ( (cloud_it->x - min) * lut_scale);  // Round the number to the closest integer
        break;
      case 1:  // y
        value = boost::math::iround ( (cloud_it->y - min) * lut_scale);
        break;
      default:  // z
        value = boost::math::iround ( (cloud_it->z - min) * lut_scale);
        break;
    }

    // Apply color to the cloud
    switch (color_mode_)
    {
      case 0:
        // Blue (= min) -> Red (= max)
        cloud_it->r = value;
        cloud_it->g = 0;
        cloud_it->b = 255 - value;
        break;
      case 1:
        // Green (= min) -> Magenta (= max)
        cloud_it->r = value;
        cloud_it->g = 255 - value;
        cloud_it->b = value;
        break;
      case 2:
        // White (= min) -> Red (= max)
        cloud_it->r = 255;
        cloud_it->g = 255 - value;
        cloud_it->b = 255 - value;
        break;
      case 3:
        // Grey (< 128) / Red (> 128)
        if (value > 128)
        {
          cloud_it->r = 255;
          cloud_it->g = 0;
          cloud_it->b = 0;
        }
        else
        {
          cloud_it->r = 128;
          cloud_it->g = 128;
          cloud_it->b = 128;
        }
        break;
      default:
        // Blue -> Green -> Red (~ rainbow)
        cloud_it->r = value > 128 ? (value - 128) * 2 : 0;  // r[128] = 0, r[255] = 255
        cloud_it->g = value < 128 ? 2 * value : 255 - ( (value - 128) * 2);  // g[0] = 0, g[128] = 255, g[255] = 0
        cloud_it->b = value < 128 ? 255 - (2 * value) : 0;  // b[0] = 255, b[128] = 0
    }
  }
}
