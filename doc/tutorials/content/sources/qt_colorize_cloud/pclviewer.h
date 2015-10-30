#ifndef PCLVIEWER_H
#define PCLVIEWER_H

// Qt
#include <QMainWindow>
#include <QFileDialog>

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/visualization/pcl_visualizer.h>

// Boost
#include <boost/math/special_functions/round.hpp>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

namespace Ui
{
  class PCLViewer;
}

class PCLViewer : public QMainWindow
{
  Q_OBJECT

  public:
    /** @brief Constructor */
    explicit
    PCLViewer (QWidget *parent = 0);

    /** @brief Destructor */
    ~PCLViewer ();

  public slots:
    /** @brief Triggered whenever the "Save file" button is clicked */
    void
    saveFileButtonPressed ();

    /** @brief Triggered whenever the "Load file" button is clicked */
    void
    loadFileButtonPressed ();

    /** @brief Triggered whenever a button in the "Color on axis" group is clicked */
    void
    axisChosen ();

    /** @brief Triggered whenever a button in the "Color mode" group is clicked */
    void
    lookUpTableChosen ();

  protected:
    /** @brief The PCL visualizer object */
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;

    /** @brief The point cloud displayed */
    PointCloudT::Ptr cloud_;

    /** @brief 0 = x | 1 = y | 2 = z */
    int filtering_axis_;

    /** @brief Holds the color mode for @ref colorCloudDistances */
    int color_mode_;

    /** @brief Color point cloud on X,Y or Z axis using a Look-Up Table (LUT)
     * Computes a LUT and color the cloud accordingly, available color palettes are :
     *
     *  Values are on a scale from 0 to 255:
     *  0. Blue (= 0) -> Red (= 255), this is the default value
     *  1. Green (= 0) -> Magenta (= 255)
     *  2. White (= 0) -> Red (= 255)
     *  3. Grey (< 128) / Red (> 128)
     *  4. Blue -> Green -> Red (~ rainbow)
     *
     * @warning If there's an outlier in the data the color may seem uniform because of this outlier!
     * @note A boost rounding exception error will be thrown if used with a non dense point cloud
     */
    void
    colorCloudDistances ();

  private:
    /** @brief ui pointer */
    Ui::PCLViewer *ui;
};

#endif // PCLVIEWER_H
