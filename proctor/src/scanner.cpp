#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>

#include <vtkLidarScanner.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkTransform.h>

#include "proctor/proctor.h"
#include "proctor/scanner.h"

#include "SyntheticLidarScanner/vtkRay.cxx"
#include "SyntheticLidarScanner/vtkLidarPoint.cxx"
#include "SyntheticLidarScanner/vtkLidarScanner.cxx"

/** convert radians to degrees. thanks a lot, vtk */
template <typename T>
static inline T deg(T rad) {
  return rad / M_PI * 180;
}

/** prepare the transform to use for a scan (camera, not object) */
static vtkSmartPointer<vtkTransform> compute_transform(Scanner::Scan scan) {
  vtkSmartPointer<vtkTransform> spt = vtkSmartPointer<vtkTransform>::New();
  Proctor::Model &model = Proctor::models[scan.mi];
  spt->Translate(model.cx, model.cy, model.cz);
  spt->RotateY(-deg(scan.phi));
  spt->RotateX(-deg(scan.theta));
  spt->Translate(0, 0, Scanner::distance_multiplier * model.scale);
  spt->RotateX(-90); // the default is looking in the +y direction. rotate to -z direction.
  return spt;
}

/** simulate lidar scanning to get a point cloud (without normals) */
static PointCloud<PointXYZ>::Ptr compute_pcxyz(int model, vtkSmartPointer<vtkTransform> transform) {
  // TODO: investigate replacing vtkLidarScanner with vtkRenderWindow::GetZbufferData
  // I think this function leaks memory.
  vtkLidarScanner *ls = vtkLidarScanner::New();
  vtkPolyData *pd = vtkPolyData::New();

  ls->SetThetaSpan(Scanner::fov_y);
  ls->SetPhiSpan(Scanner::fov_x);
  ls->SetNumberOfThetaPoints(Scanner::res_y);
  ls->SetNumberOfPhiPoints(Scanner::res_x);
  ls->SetTransform(transform);
  ls->SetInputConnection(Proctor::models[model].mesh->GetProducerPort());
  ls->Update();

  PointCloud<PointXYZ>::Ptr pcxyz (new PointCloud<PointXYZ>());
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
static PointCloud<Normal>::Ptr compute_pcn(PointCloud<PointXYZ>::ConstPtr in, float vx, float vy, float vz) {
  PointCloud<Normal>::Ptr pcn (new PointCloud<Normal>());
  NormalEstimation<PointXYZ, Normal> ne;
  search::KdTree<PointXYZ>::Ptr kdt (new search::KdTree<PointXYZ>());
  ne.setInputCloud(in);
  ne.setSearchMethod(kdt);
  ne.setKSearch(20);
  ne.setViewPoint(vx, vy, vz);
  ne.compute(*pcn);
  return pcn;
}

PointCloud<PointNormal>::Ptr Scanner::getCloud(Scan scan) {
  PointCloud<PointNormal>::Ptr cloud (new PointCloud<PointNormal>());
  vtkSmartPointer<vtkTransform> transform = compute_transform(scan);
  PointCloud<PointXYZ>::Ptr pcxyz = compute_pcxyz(scan.mi, transform);
  float v[3];
  transform->GetPosition(v);
  PointCloud<Normal>::Ptr pcn = compute_pcn(pcxyz, v[0], v[1], v[2]);
  concatenateFields(*pcxyz, *pcn, *cloud);
  return cloud;
}

PointCloud<PointNormal>::Ptr Scanner::getCloudCached(int mi, int ti, int pi) {
  Scan scan = {
    mi,
    Proctor::theta_start + ti * Proctor::theta_step,
    Proctor::phi_start + pi * Proctor::phi_step
  };
  char name[22];
  sprintf(name, "scan_%04d_%03.0f_%03.0f.pcd", Proctor::models[scan.mi].id, deg(scan.theta), deg(scan.phi));
  if (ifstream(name)) {
    PointCloud<PointNormal>::Ptr cloud (new PointCloud<PointNormal>());
    io::loadPCDFile(name, *cloud);
    return cloud;
  } else {
    PointCloud<PointNormal>::Ptr cloud = getCloud(scan);
    io::savePCDFileBinary(name, *cloud);
    return cloud;
  }
}
