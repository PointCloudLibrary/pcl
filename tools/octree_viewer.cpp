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
 *  \author Raphael Favier
 * */

#include <pcl/io/pcd_io.h>
#include <pcl/common/time.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/visualization/common/common.h>

#include <pcl/octree/octree_pointcloud_voxelcentroid.h>

#include <pcl/filters/filter.h>
#include "boost.h"

#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkCubeSource.h>
//=============================
// Displaying cubes is very long!
// so we limit their numbers.
 const int MAX_DISPLAYED_CUBES(15000);
//=============================

class OctreeViewer
{
public:
  OctreeViewer (std::string &filename, double resolution) :
    viz ("Octree visualizator"), cloud (new pcl::PointCloud<pcl::PointXYZ>()),
        displayCloud (new pcl::PointCloud<pcl::PointXYZ>()), octree (resolution), displayCubes(false),
        showPointsWithCubes (false), wireframe (true)
  {

    //try to load the cloud
    if (!loadCloud(filename))
      return;

    //register keyboard callbacks
    viz.registerKeyboardCallback(&OctreeViewer::keyboardEventOccurred, *this, 0);

    //key legends
    viz.addText("Keys:", 0, 170, 0.0, 1.0, 0.0, "keys_t");
    viz.addText("a -> Increment displayed depth", 10, 155, 0.0, 1.0, 0.0, "key_a_t");
    viz.addText("z -> Decrement displayed depth", 10, 140, 0.0, 1.0, 0.0, "key_z_t");
    viz.addText("d -> Toggle Point/Cube representation", 10, 125, 0.0, 1.0, 0.0, "key_d_t");
    viz.addText("x -> Show/Hide original cloud", 10, 110, 0.0, 1.0, 0.0, "key_x_t");
    viz.addText("s/w -> Surface/Wireframe representation", 10, 95, 0.0, 1.0, 0.0, "key_sw_t");

    //set current level to half the maximum one
    displayedDepth = static_cast<int> (floor (octree.getTreeDepth() / 2.0));
    if (displayedDepth == 0)
      displayedDepth = 1;

    //show octree at default depth
    extractPointsAtLevel(displayedDepth);

    //reset camera
    viz.resetCameraViewpoint("cloud");

    //run main loop
    run();

  }

private:
  //========================================================
  // PRIVATE ATTRIBUTES
  //========================================================
  //visualizer
  pcl::PointCloud<pcl::PointXYZ>::Ptr xyz;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyz_rgb;

  pcl::visualization::PCLVisualizer viz;
  //original cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  //displayed_cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr displayCloud;
  //octree
  pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ> octree;
  //level
  int displayedDepth;
  //bool to decide if we display points or cubes
  bool displayCubes, showPointsWithCubes, wireframe;
  //========================================================

  /* \brief Callback to interact with the keyboard
   *
   */
  void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void *)
  {

    if (event.getKeySym() == "a" && event.keyDown())
    {
      IncrementLevel();
    }
    else if (event.getKeySym() == "z" && event.keyDown())
    {
      DecrementLevel();
    }
    else if (event.getKeySym() == "d" && event.keyDown())
    {
      displayCubes = !displayCubes;
      update();
    }
    else if (event.getKeySym() == "x" && event.keyDown())
    {
      showPointsWithCubes = !showPointsWithCubes;
      update();
    }
    else if (event.getKeySym() == "w" && event.keyDown())
    {
      if(!wireframe)
        wireframe=true;
      update();
    }
    else if (event.getKeySym() == "s" && event.keyDown())
    {
      if(wireframe)
        wireframe=false;
      update();
    }
  }

  /* \brief Graphic loop for the viewer
   *
   */
  void run()
  {
    while (!viz.wasStopped())
    {
      //main loop of the visualizer
      viz.spinOnce(100);
      boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
  }

  /* \brief Helper function that read a pointcloud file (returns false if pbl)
   *  Also initialize the octree
   *
   */
  bool loadCloud(std::string &filename)
  {
    std::cout << "Loading file " << filename.c_str() << std::endl;
    //read cloud
    if (pcl::io::loadPCDFile(filename, *cloud))
    {
      std::cerr << "ERROR: Cannot open file " << filename << "! Aborting..." << std::endl;
      return false;
    }

    //remove NaN Points
    std::vector<int> nanIndexes;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, nanIndexes);
    std::cout << "Loaded " << cloud->points.size() << " points" << std::endl;

    //create octree structure
    octree.setInputCloud(cloud);
    //update bounding box automatically
    octree.defineBoundingBox();
    //add points in the tree
    octree.addPointsFromInputCloud();
    return true;
  }

  /* \brief Helper function that draw info for the user on the viewer
   *
   */
  void showLegend(bool showCubes)
  {
    char dataDisplay[256];
    sprintf(dataDisplay, "Displaying data as %s", (showCubes) ? ("CUBES") : ("POINTS"));
    viz.removeShape("disp_t");
    viz.addText(dataDisplay, 0, 60, 1.0, 0.0, 0.0, "disp_t");

    char level[256];
    sprintf(level, "Displayed depth is %d on %d", displayedDepth, octree.getTreeDepth());
    viz.removeShape("level_t1");
    viz.addText(level, 0, 45, 1.0, 0.0, 0.0, "level_t1");

    viz.removeShape("level_t2");
    sprintf(level, "Voxel size: %.4fm [%lu voxels]", sqrt(octree.getVoxelSquaredSideLen(displayedDepth)),
            displayCloud->points.size());
    viz.addText(level, 0, 30, 1.0, 0.0, 0.0, "level_t2");

    viz.removeShape("org_t");
    if (showPointsWithCubes)
      viz.addText("Displaying original cloud", 0, 15, 1.0, 0.0, 0.0, "org_t");
  }

  /* \brief Visual update. Create visualizations and add them to the viewer
   *
   */
  void update()
  {
    //remove existing shapes from visualizer
    clearView();

    //prevent the display of too many cubes
    bool displayCubeLegend = displayCubes && static_cast<int> (displayCloud->points.size ()) <= MAX_DISPLAYED_CUBES;

    showLegend(displayCubeLegend);

    if (displayCubeLegend)
    {
      //show octree as cubes
      showCubes(sqrt(octree.getVoxelSquaredSideLen(displayedDepth)));
      if (showPointsWithCubes)
      {
        //add original cloud in visualizer
        pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> color_handler(cloud, "z");
        viz.addPointCloud(cloud, color_handler, "cloud");
      }
    }
    else
    {
      //add current cloud in visualizer
      pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> color_handler(displayCloud,"z");
      viz.addPointCloud(displayCloud, color_handler, "cloud");
    }
  }

  /* \brief remove dynamic objects from the viewer
   *
   */
  void clearView()
  {
    //remove cubes if any
    vtkRenderer *renderer = viz.getRenderWindow()->GetRenderers()->GetFirstRenderer();
    while (renderer->GetActors()->GetNumberOfItems() > 0)
      renderer->RemoveActor(renderer->GetActors()->GetLastActor());
    //remove point clouds if any
    viz.removePointCloud("cloud");
  }

  /* \brief Create a vtkSmartPointer object containing a cube
   *
   */
  vtkSmartPointer<vtkPolyData> GetCuboid(double minX, double maxX, double minY, double maxY, double minZ, double maxZ)
  {
    vtkSmartPointer<vtkCubeSource> cube = vtkSmartPointer<vtkCubeSource>::New();
    cube->SetBounds(minX, maxX, minY, maxY, minZ, maxZ);
    return cube->GetOutput();
  }

  /* \brief display octree cubes via vtk-functions
   *
   */
  void showCubes(double voxelSideLen)
  {
    //get the renderer of the visualizer object
    vtkRenderer *renderer = viz.getRenderWindow()->GetRenderers()->GetFirstRenderer();

    vtkSmartPointer<vtkAppendPolyData> treeWireframe = vtkSmartPointer<vtkAppendPolyData>::New();
    size_t i;
    double s = voxelSideLen / 2.0;
    for (i = 0; i < displayCloud->points.size(); i++)
    {

      double x = displayCloud->points[i].x;
      double y = displayCloud->points[i].y;
      double z = displayCloud->points[i].z;

#if VTK_MAJOR_VERSION < 6
      treeWireframe->AddInput(GetCuboid(x - s, x + s, y - s, y + s, z - s, z + s));
#else
      treeWireframe->AddInputData (GetCuboid (x - s, x + s, y - s, y + s, z - s, z + s));
#endif
    }

    vtkSmartPointer<vtkActor> treeActor = vtkSmartPointer<vtkActor>::New();

    vtkSmartPointer<vtkDataSetMapper> mapper = vtkSmartPointer<vtkDataSetMapper>::New();
#if VTK_MAJOR_VERSION < 6
    mapper->SetInput(treeWireframe->GetOutput());
#else
    mapper->SetInputData (treeWireframe->GetOutput ());
#endif
    treeActor->SetMapper(mapper);

    treeActor->GetProperty()->SetColor(1.0, 1.0, 1.0);
    treeActor->GetProperty()->SetLineWidth(2);
    if(wireframe)
    {
      treeActor->GetProperty()->SetRepresentationToWireframe();
      treeActor->GetProperty()->SetOpacity(0.35);
    }
    else
      treeActor->GetProperty()->SetRepresentationToSurface();

    renderer->AddActor(treeActor);
  }

  /* \brief Extracts all the points of depth = level from the octree
   *
   */
  void extractPointsAtLevel(int depth)
  {
    displayCloud->points.clear();

    pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ>::Iterator tree_it;
    pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ>::Iterator tree_it_end = octree.end();

    pcl::PointXYZ pt;
    std::cout << "===== Extracting data at depth " << depth << "... " << std::flush;
    double start = pcl::getTime ();

    for (tree_it = octree.begin(depth); tree_it!=tree_it_end; ++tree_it)
    {
      Eigen::Vector3f voxel_min, voxel_max;
      octree.getVoxelBounds(tree_it, voxel_min, voxel_max);

      pt.x = (voxel_min.x() + voxel_max.x()) / 2.0f;
      pt.y = (voxel_min.y() + voxel_max.y()) / 2.0f;
      pt.z = (voxel_min.z() + voxel_max.z()) / 2.0f;
      displayCloud->points.push_back(pt);
    }

    double end = pcl::getTime ();
    printf("%lu pts, %.4gs. %.4gs./pt. =====\n", displayCloud->points.size (), end - start,
           (end - start) / static_cast<double> (displayCloud->points.size ()));

    update();
  }

  /* \brief Helper function to increase the octree display level by one
   *
   */
  bool IncrementLevel()
  {
    if (displayedDepth < static_cast<int> (octree.getTreeDepth ()))
    {
      displayedDepth++;
      extractPointsAtLevel(displayedDepth);
      return true;
    }
    else
      return false;
  }

  /* \brief Helper function to decrease the octree display level by one
   *
   */
  bool DecrementLevel()
  {
    if (displayedDepth > 0)
    {
      displayedDepth--;
      extractPointsAtLevel(displayedDepth);
      return true;
    }
    return false;
  }

};

int main(int argc, char ** argv)
{
  if (argc != 3)
  {
    std::cerr << "ERROR: Syntax is octreeVisu <pcd file> <resolution>" << std::endl;
    std::cerr << "EXAMPLE: ./octreeVisu bun0.pcd 0.001" << std::endl;
    return -1;
  }

  std::string cloud_path(argv[1]);
  OctreeViewer v(cloud_path, atof(argv[2]));
}
