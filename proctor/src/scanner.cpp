#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>

#include <sstream>

#if ((VTK_MAJOR_VERSION == 5) && (VTK_MINOR_VERSION >= 8))
#include <vtkLidarScanner.h>
#endif
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


namespace pcl
{

  namespace proctor
  {

    const float Scanner::distance_multiplier = 3.0f;
    const double Scanner::fov_x = M_PI / 3;
    const double Scanner::fov_y = M_PI / 4;
    const unsigned int Scanner::res_x = 320;
    const unsigned int Scanner::res_y = 240;

    // TODO Find out if step is supposed to be bigger
    const float Scanner::theta_start = M_PI / 12;
    const float Scanner::theta_step = 0.0f;
    const int Scanner::theta_count = 1;
    const float Scanner::phi_start = 0.0f;
    const float Scanner::phi_step = float (M_PI) / 6.0f;
    const int Scanner::phi_count = 12;

    const float Scanner::theta_min = 0.0f;
    const float Scanner::theta_max = float (M_PI) / 6;
    const float Scanner::phi_min = 0.0f;
    const float Scanner::phi_max = float (M_PI) * 2.0f;

    /** convert radians to degrees. thanks a lot, vtk */
    template <typename T>
    static inline T deg(T rad)
    {
      return rad / M_PI * 180;
    }

    /** prepare the transform to use for a scan (camera, not object) */
    static vtkSmartPointer<vtkTransform>
    compute_transform(Scanner::Scan scan, Model &model)
    {
      vtkSmartPointer<vtkTransform> spt = vtkSmartPointer<vtkTransform>::New();
      spt->Translate(model.cx, model.cy, model.cz);
      spt->RotateY(-deg(scan.phi));
      spt->RotateX(-deg(scan.theta));
      spt->Translate(0, 0, Scanner::distance_multiplier * model.scale);
      spt->RotateX(-90); // the default is looking in the +y direction. rotate to -z direction.
      return spt;
    }

    /** simulate lidar scanning to get a point cloud (without normals) */
    static PointCloud<PointXYZ>::Ptr
    compute_pcxyz(Model &model, vtkSmartPointer<vtkTransform> transform)
    {
      // TODO: investigate replacing vtkLidarScanner with vtkRenderWindow::GetZbufferData
      // I think this function leaks memory.
      PointCloud<PointXYZ>::Ptr pcxyz (new PointCloud<PointXYZ>());
#if ((VTK_MAJOR_VERSION == 5) && (VTK_MINOR_VERSION >= 8))
      vtkLidarScanner *ls = vtkLidarScanner::New();
      vtkPolyData *pd = vtkPolyData::New();

      ls->SetThetaSpan(Scanner::fov_y);
      ls->SetPhiSpan(Scanner::fov_x);
      ls->SetNumberOfThetaPoints(Scanner::res_y);
      ls->SetNumberOfPhiPoints(Scanner::res_x);
      ls->SetTransform(transform);
      ls->SetInputConnection(model.mesh);
      ls->SetCreateMesh(false);
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
#endif
      return pcxyz;
    }

    /** estimate the normals of a point cloud */
    static PointCloud<Normal>::Ptr
    compute_pcn(PointCloud<PointXYZ>::ConstPtr in, float vx, float vy, float vz)
    {
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

    /** Scanner */
    PointCloud<PointNormal>::Ptr
    Scanner::getCloud(Scan scan, Model &model)
    {
      PointCloud<PointNormal>::Ptr cloud (new PointCloud<PointNormal>());
      vtkSmartPointer<vtkTransform> transform = compute_transform(scan, model);
      PointCloud<PointXYZ>::Ptr pcxyz = compute_pcxyz(model, transform);
      float v[3];
      transform->GetPosition(v);
      PointCloud<Normal>::Ptr pcn = compute_pcn(pcxyz, v[0], v[1], v[2]);
      concatenateFields(*pcxyz, *pcn, *cloud);

      return cloud;
    }

    PointCloud<PointNormal>::Ptr
    Scanner::getCloudCached(float theta, float phi, Model &model)
    {
      Scan scan = { theta, phi };

      std::stringstream ss;
      ss << "scan_" << model.id << "_" << deg(scan.theta) << "_" << deg(scan.phi) << ".pcd";
      std::string name = ss.str();
      if (ifstream(name.c_str())) {
        PointCloud<PointNormal>::Ptr cloud (new PointCloud<PointNormal>());
        io::loadPCDFile(name, *cloud);
        return cloud;
      } else {
        PointCloud<PointNormal>::Ptr cloud = getCloud(scan, model);
        io::savePCDFileBinary(name, *cloud);
        return cloud;
      }
    }
  }
}
