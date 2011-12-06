#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>

#include <vtkLidarScanner.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkTransform.h>

#include "proctor/proctor.h"
#include "proctor/scanner.h"

#if ((VTK_MAJOR_VERSION == 5) && (VTK_MINOR_VERSION >= 8))
#include "SyntheticLidarScanner/vtkRay.cxx"
#include "SyntheticLidarScanner/vtkLidarPoint.cxx"
#include "SyntheticLidarScanner/vtkLidarScanner.cxx"
#endif

using namespace pcl;
using namespace pcl::proctor;

/** convert radians to degrees. thanks a lot, vtk */
template <typename T>
static inline T deg(T rad) {
  return rad / M_PI * 180;
}

/** prepare the transform to use for a scan (camera, not object) */
static vtkSmartPointer<vtkTransform> compute_transform(pcl::proctor::Scanner::Scan scan, Model &model) {
  vtkSmartPointer<vtkTransform> spt = vtkSmartPointer<vtkTransform>::New();
  spt->Translate(model.cx, model.cy, model.cz);
  spt->RotateY(-deg(scan.phi));
  spt->RotateX(-deg(scan.theta));
  spt->Translate(0, 0, Scanner::distance_multiplier * model.scale);
  spt->RotateX(-90); // the default is looking in the +y direction. rotate to -z direction.
  return spt;
}

/** simulate lidar scanning to get a point cloud (without normals) */
static pcl::PointCloud<pcl::PointXYZ>::Ptr compute_pcxyz(Model &model, vtkSmartPointer<vtkTransform> transform) {
  // TODO: investigate replacing vtkLidarScanner with vtkRenderWindow::GetZbufferData
  // I think this function leaks memory.
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcxyz (new pcl::PointCloud<pcl::PointXYZ>());
  vtkLidarScanner *ls = vtkLidarScanner::New();
  vtkPolyData *pd = vtkPolyData::New();

  ls->SetThetaSpan(pcl::proctor::Scanner::fov_y);
  ls->SetPhiSpan(pcl::proctor::Scanner::fov_x);
  ls->SetNumberOfThetaPoints(pcl::proctor::Scanner::res_y);
  ls->SetNumberOfPhiPoints(pcl::proctor::Scanner::res_x);
  ls->SetTransform(transform);
  ls->SetInputConnection(model.mesh->GetProducerPort());
  ls->Update();

  ls->GetValidOutputPoints(pd);
  float (*points)[3] = reinterpret_cast<float (*)[3]>(vtkFloatArray::SafeDownCast(pd->GetPoints()->GetData())->GetPointer(0));
  int num_points = pd->GetPoints()->GetData()->GetNumberOfTuples();
  pcxyz->points.resize(num_points);
  for (int i = 0; i < num_points; i++) {
    pcxyz->points[i].x = points[i][0];
    pcxyz->points[i].y = points[i][1];
    pcxyz->points[i].z = points[i][2];
  }

  ls->Delete();
  pd->Delete();
  return pcxyz;
}

/** estimate the normals of a point cloud */
static pcl::PointCloud<pcl::Normal>::Ptr compute_pcn(pcl::PointCloud<pcl::PointXYZ>::ConstPtr in, float vx, float vy, float vz) {
  pcl::PointCloud<pcl::Normal>::Ptr pcn (new pcl::PointCloud<pcl::Normal>());
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr kdt (new pcl::search::KdTree<pcl::PointXYZ>());
  ne.setInputCloud(in);
  ne.setSearchMethod(kdt);
  ne.setKSearch(20);
  ne.setViewPoint(vx, vy, vz);
  ne.compute(*pcn);
  return pcn;
}

/** Scanner */
const float pcl::proctor::Scanner::distance_multiplier = 3.0f;
const double pcl::proctor::Scanner::fov_x = M_PI / 3;
const double pcl::proctor::Scanner::fov_y = M_PI / 4;
const unsigned int pcl::proctor::Scanner::res_x = 320;
const unsigned int pcl::proctor::Scanner::res_y = 240;

pcl::PointCloud<pcl::PointNormal>::Ptr pcl::proctor::Scanner::getCloud(Scan scan, Model &model) {
  PointCloud<PointNormal>::Ptr cloud (new PointCloud<PointNormal>());
  vtkSmartPointer<vtkTransform> transform = compute_transform(scan, model);
  PointCloud<PointXYZ>::Ptr pcxyz = compute_pcxyz(model, transform);
  float v[3];
  transform->GetPosition(v);
  PointCloud<Normal>::Ptr pcn = compute_pcn(pcxyz, v[0], v[1], v[2]);
  concatenateFields(*pcxyz, *pcn, *cloud);
  return cloud;
}

PointCloud<PointNormal>::Ptr pcl::proctor::Scanner::getCloudCached(int mi, int ti, int pi, pcl::proctor::Model &model) {
  Scan scan = {
    mi,
    pcl::proctor::Proctor::theta_start + ti * Proctor::theta_step,
    pcl::proctor::Proctor::phi_start + pi * Proctor::phi_step
  };
  char name[22];
  sprintf(name, "scan_%04d_%03.0f_%03.0f.pcd", mi, deg(scan.theta), deg(scan.phi));
  if (ifstream(name)) {
    PointCloud<PointNormal>::Ptr cloud (new PointCloud<PointNormal>());
    io::loadPCDFile(name, *cloud);
    return cloud;
  } else {
    PointCloud<PointNormal>::Ptr cloud = getCloud(scan, Proctor::models[mi]);
    io::savePCDFileBinary(name, *cloud);
    return cloud;
  }
}
