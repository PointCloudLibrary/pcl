/**
 * Demo program for simulation library
 * A virtual camera generates simulated point clouds
 * No visual output, point clouds saved to file
 * 
 * three different demo modes:
 * 0 - static camera, 100 poses
 * 1 - circular camera flying around the scene, 16 poses
 * 2 - camera translates between 2 poses using slerp, 20 poses
 * pcl_sim_terminal_demo 2 ../../../../kmcl/models/table_models/meta_model.ply  
 */

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <boost/shared_ptr.hpp>
#ifdef _WIN32
# define WIN32_LEAN_AND_MEAN
# include <windows.h>
#endif
#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "pcl/common/common.h"
#include "pcl/common/transforms.h"

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

// define the following in order to eliminate the deprecated headers warning
#define VTK_EXCLUDE_STRSTREAM_HEADERS
#include <pcl/io/vtk_lib_io.h>

#include "pcl/simulation/camera.h"
#include "pcl/simulation/model.h"
#include "pcl/simulation/scene.h"
#include "pcl/simulation/range_likelihood.h"

#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

// RangeImage:
#include <pcl/range_image/range_image_planar.h>

// Pop-up viewer
#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>

// Writing PNG files:
#include <opencv2/opencv.hpp>

using namespace Eigen;
using namespace pcl;
using namespace pcl::console;
using namespace pcl::io;
using namespace pcl::simulation;

using namespace std;

uint16_t t_gamma[2048];

Scene::Ptr scene_;
Camera::Ptr camera_;
RangeLikelihood::Ptr range_likelihood_;

int window_width_;
int window_height_;
bool paused_;
bool write_file_;

void printHelp (int argc, char **argv)
{
  print_error ("Syntax is: %s <mode 1,2 or 3> <filename>\n", argv[0]);
  print_info ("acceptable filenames include vtk, obj and ply. ply can support colour\n");
}

void write_score_image(const float* score_buffer)
{
  int npixels = range_likelihood_->getWidth() * range_likelihood_->getHeight();
  uint8_t* score_img = new uint8_t[npixels * 3];

  float min_score = score_buffer[0];
  float max_score = score_buffer[0];
  for (int i=1; i<npixels; i++)
  {
    if (score_buffer[i] < min_score) min_score = score_buffer[i];
    if (score_buffer[i] > max_score) max_score = score_buffer[i];
  }

  for (int y = 0; y <  480; ++y)
  {
    for (int x = 0; x < 640; ++x)
    {
      int i = y*640 + x ;
      int i_in= (480-1 -y) *640 + x ; // flip up

      float d = (score_buffer[i_in]-min_score)/(max_score-min_score);
      score_img[3*i+0] = 0;
      score_img[3*i+1] = d*255;
      score_img[3*i+2] = 0;
    }
  }

  // Write to file:
  IplImage *cv_ipl = cvCreateImage( cvSize(640 ,480), 8, 3);
  cv::Mat cv_mat(cv_ipl);
  cv_mat.data = score_img;
  cv::imwrite("score_image.png", cv_mat);

  delete [] score_img;
}

void write_depth_image(const float* depth_buffer)
{
  int npixels = range_likelihood_->getWidth() * range_likelihood_->getHeight();
  uint8_t* depth_img = new uint8_t[npixels * 3];

  float min_depth = depth_buffer[0];
  float max_depth = depth_buffer[0];
  for (int i=1; i<npixels; i++)
  {
    if (depth_buffer[i] < min_depth) min_depth = depth_buffer[i];
    if (depth_buffer[i] > max_depth) max_depth = depth_buffer[i];
  }

  for (int y = 0; y <  480; ++y)
  {
    for (int x = 0; x < 640; ++x)
    {
      int i= y*640 + x ;
      int i_in= (480-1 -y) *640 + x ; // flip up down

      float zn = 0.7;
      float zf = 20.0;
      float d = depth_buffer[i_in];
      float z = -zf*zn/((zf-zn)*(d - zf/(zf-zn)));
      float b = 0.075;
      float f = 580.0;
      uint16_t kd = static_cast<uint16_t>(1090 - b*f/z*8);
      if (kd < 0) kd = 0;
      else if (kd>2047) kd = 2047;

      int pval = t_gamma[kd];
      int lb = pval & 0xff;
      switch (pval>>8) 
      {
        case 0:
            depth_img[3*i+2] = 255;
            depth_img[3*i+1] = 255-lb;
            depth_img[3*i+0] = 255-lb;
            break;
        case 1:
            depth_img[3*i+2] = 255;
            depth_img[3*i+1] = lb;
            depth_img[3*i+0] = 0;
            break;
        case 2:
            depth_img[3*i+2] = 255-lb;
            depth_img[3*i+1] = 255;
            depth_img[3*i+0] = 0;
            break;
        case 3:
            depth_img[3*i+2] = 0;
            depth_img[3*i+1] = 255;
            depth_img[3*i+0] = lb;
            break;
        case 4:
            depth_img[3*i+2] = 0;
            depth_img[3*i+1] = 255-lb;
            depth_img[3*i+0] = 255;
            break;
        case 5:
            depth_img[3*i+2] = 0;
            depth_img[3*i+1] = 0;
            depth_img[3*i+0] = 255-lb;
            break;
        default:
            depth_img[3*i+2] = 0;
            depth_img[3*i+1] = 0;
            depth_img[3*i+0] = 0;
            break;
      }
    }
  }

  // Write to file:
  IplImage *cv_ipl = cvCreateImage( cvSize(640 ,480), 8, 3);
  cv::Mat cv_mat(cv_ipl);
  cv_mat.data = depth_img;
  cv::imwrite("depth_image.png", cv_mat);

  delete [] depth_img;
}

void write_rgb_image(const uint8_t* rgb_buffer)
{
  int npixels = range_likelihood_->getWidth() * range_likelihood_->getHeight();
  uint8_t* rgb_img = new uint8_t[npixels * 3];

  for (int y = 0; y <  480; ++y)
  {
    for (int x = 0; x < 640; ++x)
    {
      int px= y*640 + x ;
      int px_in= (480-1 -y) *640 + x ; // flip up down
      rgb_img [3* (px) +0] = rgb_buffer[3*px_in+0];
      rgb_img [3* (px) +1] = rgb_buffer[3*px_in+1];
      rgb_img [3* (px) +2] = rgb_buffer[3*px_in+2];
    }
  }

  // Write to file:
  IplImage *cv_ipl = cvCreateImage( cvSize(640 ,480), 8, 3);
  cv::Mat cv_mat(cv_ipl);
  cv_mat.data = rgb_img ;
  cv::imwrite("rgb_image.png", cv_mat);

  delete [] rgb_img;
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  // Snippet taken from PCLVisualizer tutorial:
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();

  //viewer->addModelFromPLYFile("/home/mfallon/projects/kmcl/kmcl/models/table_scene/meta_model.ply");

  return (viewer);
}

void capture (Eigen::Isometry3d pose_in, string point_cloud_fname)
{
  // No reference image - but this is kept for compatability with range_test_v2:
  float* reference = new float[range_likelihood_->getRowHeight() * range_likelihood_->getColWidth()];
  const float* depth_buffer = range_likelihood_->getDepthBuffer();
  // Copy one image from our last as a reference.
  for (int i=0, n=0; i<range_likelihood_->getRowHeight(); ++i)
  {
    for (int j=0; j<range_likelihood_->getColWidth(); ++j)
    {
      reference[n++] = depth_buffer[i*range_likelihood_->getWidth() + j];
    }
  }

  std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d> > poses;
  std::vector<float> scores;
  int n = 1;
  poses.push_back (pose_in);
  range_likelihood_->computeLikelihoods (reference, poses, scores);
  std::cout << "camera: " << camera_->getX ()
       << " " << camera_->getY ()
       << " " << camera_->getZ ()
       << " " << camera_->getRoll ()
       << " " << camera_->getPitch ()
       << " " << camera_->getYaw ()
       << std::endl;

  delete [] reference;


  // Benchmark Values for sim_terminal_demo ( march 27 2012): >>> used PolygonMeshModel<< 
  // 27840 triangle faces
  // 13670 vertices

  // 45.00Hz: simuation only
  //  1.28Hz: simuation, addNoise?    , getPointCloud, writeASCII
  // 33.33Hz: simuation, getPointCloud
  // 23.81Hz: simuation, getPointCloud, writeBinary
  // 14.28Hz: simuation, addNoise, getPointCloud, writeBinary
  // MODULE        TIME      FRACTION
  // simulation     0.02222   31%
  // addNoise      0.03	     41%
  // getPointCloud 0.008     11%
  // writeBinary   0.012     16%
  // total	   0.07222	

  // Update: march 29: >>> using TriangleMeshModel now <<<
  // 57.00Hz: simulation only
  // 30.61Hz: simulation, getPointCloud
  // 40.00Hz: simulation, getPointCloud, writeBinary (on average
  // 28.50Hz: simulation, addNoise, getPointCloud, writeBinary

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_out (new pcl::PointCloud<pcl::PointXYZRGB>);
  bool write_cloud=true;
  bool demo_other_stuff=false;

  if (write_cloud)
  {
    // Read Color Buffer from the GPU before creating PointCloud:
    // By default the buffers are not read back from the GPU
    range_likelihood_->getColorBuffer ();
    range_likelihood_->getDepthBuffer ();

    // Add noise directly to the CPU depth buffer 
    range_likelihood_->addNoise ();

    // Optional argument to save point cloud in global frame:
    // Save camera relative:
    //range_likelihood_->getPointCloud(pc_out);
    // Save in global frame - applying the camera frame:
    //range_likelihood_->getPointCloud(pc_out,true,camera_->pose());
    // Save in local frame
    range_likelihood_->getPointCloud (pc_out,false,camera_->getPose ());
    // TODO: what to do when there are more than one simulated view?
    std::cout << pc_out->points.size() << " points written to file\n";

    pcl::PCDWriter writer;
    //writer.write (point_cloud_fname, *pc_out,	false);  /// ASCII
    writer.writeBinary (point_cloud_fname, *pc_out);
    //cout << "finished writing file\n";
  }
  if (demo_other_stuff && write_cloud)
  {
    write_score_image (range_likelihood_->getScoreBuffer ());  
    write_rgb_image (range_likelihood_->getColorBuffer ());  
    write_depth_image (range_likelihood_->getDepthBuffer ());  

    // Demo interacton with RangeImage:
    pcl::RangeImagePlanar rangeImage;
    range_likelihood_->getRangeImagePlanar (rangeImage);

    // display viewer: (currently seqfaults on exit of viewer)
    if (1==0){
      boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
      viewer = simpleVis(pc_out);

      while (!viewer->wasStopped ())
      {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
      }
    }
  }
}


// Read in a 3D model
void load_PolygonMesh_model (char* polygon_file)
{
  pcl::PolygonMesh mesh;	// (new pcl::PolygonMesh);
  //pcl::io::loadPolygonFile("/home/mfallon/data/models/dalet/Darlek_modified_works.obj",mesh);
  pcl::io::loadPolygonFile (polygon_file, mesh);
  pcl::PolygonMesh::Ptr cloud (new pcl::PolygonMesh (mesh));

  TriangleMeshModel::Ptr model = TriangleMeshModel::Ptr (new TriangleMeshModel (cloud));
  scene_->add (model);

  std::cout << "Just read " << polygon_file << std::endl;
  std::cout << mesh.polygons.size () << " polygons and "
            << mesh.cloud.data.size () << " triangles\n";
}

void
initialize (int argc, char** argv)
{
  const GLubyte* version = glGetString (GL_VERSION);
  std::cout << "OpenGL Version: " << version << std::endl;

  // works well for MIT CSAIL model 3rd floor:
  //camera_->set(4.04454, 44.9377, 1.1, 0.0, 0.0, -2.00352);

  // works well for MIT CSAIL model 2nd floor:
//  camera_->set (27.4503, 37.383, 4.30908, 0.0, 0.0654498, -2.25802);

  // works for small files:
  //camera_->set(-5.0, 0.0, 1.0, 0.0, 0.0, 0.0);
  camera_->set(0.471703, 1.59862, 3.10937, 0, 0.418879, -12.2129);
  camera_->setPitch(0.418879); // not sure why this is here:

  cout << "About to read: " << argv[2] << endl;
  load_PolygonMesh_model (argv[2]);
}


// A 'halo' camera - a circular ring of poses all pointing at a center point
// @param: focus_center: the center points
// @param: halo_r: radius of the ring
// @param: halo_dz: elevation of the camera above/below focus_center's z value
// @param: n_poses: number of generated poses
void
generate_halo(
  std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d> > &poses,
  Eigen::Vector3d focus_center,double halo_r,double halo_dz,int n_poses){

//  std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d> > poses;
  for (double t=0;t < (2*M_PI);t =t + (2*M_PI)/ ((double) n_poses) ){
    double x = halo_r*cos(t);
    double y = halo_r*sin(t);
    double z = halo_dz;
    double pitch =atan2( halo_dz,halo_r); 
    double yaw = atan2(-y,-x);

    Eigen::Isometry3d pose;
    pose.setIdentity();
    Eigen::Matrix3d m;
    m = AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * AngleAxisd(pitch, Eigen::Vector3d::UnitY()) * AngleAxisd(0, Eigen::Vector3d::UnitZ());

    pose *=m;
    Vector3d v(x,y,z);
    v += focus_center;
    pose.translation() = v;
    poses.push_back(pose);  
  }
  return ;//poses;
}

int
main (int argc, char** argv)
{
  int width = 640;
  int height = 480;
  window_width_ = width * 2;
  window_height_ = height * 2;

  print_info ("Manually generate a simulated RGB-D point cloud using pcl::simulation. For more information, use: %s -h\n", argv[0]);

  if (argc < 3)
  {
    printHelp (argc, argv);
    return (-1);
  }
  int mode=atoi(argv[1]); 

  for (int i=0; i<2048; i++)
  {
    float v = i/2048.0;
    v = powf(v, 3)* 6;
    t_gamma[i] = v*6*256;
  }

  glutInit (&argc, argv);
  glutInitDisplayMode (GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGB);// was GLUT_RGBA
  glutInitWindowPosition (10, 10);
  glutInitWindowSize (10, 10);
  //glutInitWindowSize (window_width_, window_height_);
  glutCreateWindow ("OpenGL range likelihood");

  GLenum err = glewInit ();
  if (GLEW_OK != err)
  {
    std::cerr << "Error: " << glewGetErrorString (err) << std::endl;
    exit (-1);
  }

  std::cout << "Status: Using GLEW " << glewGetString (GLEW_VERSION) << std::endl;

  if (glewIsSupported ("GL_VERSION_2_0"))
    std::cout << "OpenGL 2.0 supported" << std::endl;
  else
  {
    std::cerr << "Error: OpenGL 2.0 not supported" << std::endl;
    exit(1);
  }

  std::cout << "GL_MAX_VIEWPORTS: " << GL_MAX_VIEWPORTS << std::endl;

  camera_ = Camera::Ptr (new Camera ());
  scene_ = Scene::Ptr (new Scene ());

  range_likelihood_ = RangeLikelihood::Ptr (new RangeLikelihood (1, 1, height, width, scene_));
  // range_likelihood_ = RangeLikelihood::Ptr(new RangeLikelihood(10, 10, 96, 96, scene_));
  // range_likelihood_ = RangeLikelihood::Ptr(new RangeLikelihood(1, 1, 480, 640, scene_));

  // Actually corresponds to default parameters:
  range_likelihood_->setCameraIntrinsicsParameters (640,480, 576.09757860,
                                                    576.09757860, 321.06398107, 242.97676897);
  range_likelihood_->setComputeOnCPU (false);
  range_likelihood_->setSumOnCPU (true);
  range_likelihood_->setUseColor (true);
  initialize (argc, argv);

  // simulation mode:
  // 0 100 fixed poses 
  // 1 a 'halo' camera 
  // 2 slerp between two different poses
  std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d> > poses;
  if (mode==0)
  {
    // Create a pose:
    Eigen::Isometry3d pose;
    pose.setIdentity();
    Matrix3d m;
    //ypr:
    m = AngleAxisd(-9.14989, Vector3d::UnitZ())     * AngleAxisd(0.20944, Vector3d::UnitY())    * AngleAxisd(0, Vector3d::UnitX());  
    pose *= m;
    Vector3d v;
    v << 1.31762, 0.382931, 1.89533;
    pose.translation() = v;  
    for (int i=0;i< 100;i++)
    { // duplicate the pose 100 times
      poses.push_back(pose); 
    }
  }
  else if(mode==1)
  {
    Eigen::Vector3d focus_center(0,0,1.3);
    double halo_r = 4;
    double halo_dz = 2;
    int n_poses=16;
    generate_halo(poses,focus_center,halo_r,halo_dz,n_poses);
  }
  else if (mode==2)
  {
    Eigen::Isometry3d pose1;
    pose1.setIdentity();
    pose1.translation() << 1,0.75,2;
    Eigen::Matrix3d rot1;
    rot1 = AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()) * AngleAxisd(M_PI/10, Eigen::Vector3d::UnitY()) * AngleAxisd(0.0, Eigen::Vector3d::UnitZ()); // ypr
    pose1.rotate(rot1);
    Eigen::Isometry3d pose2;
    pose2.setIdentity();
    pose2.translation() << 1,-1,3;
    Eigen::Matrix3d rot2;
    rot2 = AngleAxisd(3*M_PI/4, Eigen::Vector3d::UnitZ()) * AngleAxisd(M_PI/4, Eigen::Vector3d::UnitY()) * AngleAxisd(0.0, Eigen::Vector3d::UnitZ()); // ypr
    pose2.rotate(rot2);

    int n_poses = 20;
    for (double i=0; i<=1;i+= 1/((double) n_poses -1) )
    {
      Eigen::Quaterniond rot3; 
      Eigen::Quaterniond r1(pose1.rotation());
      Eigen::Quaterniond r2(pose2.rotation());
      rot3 = r1.slerp(i,r2);
      Eigen::Isometry3d pose;
      pose.setIdentity();
      Eigen::Vector3d trans3 = (1-i)*pose1.translation() +  i*pose2.translation();
      pose.translation() <<  trans3[0] ,trans3[1] ,trans3[2];
      pose.rotate(rot3);
      poses.push_back(pose);
    }
  }

  // Do the simulation:
  double tic_main = getTime();
  for (size_t i=0;i < poses.size();i++)
  {
    std::stringstream ss;
    ss.precision(20);
    ss << "simulated_pcl_" << i << ".pcd";
    double tic = getTime();
    capture(poses[i],ss.str());
    cout << (getTime() -tic) << " sec\n";
  }
  cout << poses.size() << " poses simulated in " << (getTime() -tic_main) << "seconds\n";
  cout << (poses.size()/ (getTime() -tic_main) ) << "Hz on average\n";

  return 0;
}
