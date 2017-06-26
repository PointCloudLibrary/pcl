/*
 * uniform_sampling.h
 *
 *  Created on: Mar 25, 2012
 *      Author: aitor
 */

#ifndef REC_FRAMEWORK_UNIFORM_SAMPLING_H_
#define REC_FRAMEWORK_UNIFORM_SAMPLING_H_

#include <vtkPolyData.h>
#include <vtkTriangle.h>
#include <vtkSmartPointer.h>
#include <vtkCellArray.h>
#include <vtkPLYReader.h>
#include <vtkPolyDataMapper.h>
#include <vtkTransform.h>
#include <vtkTransformFilter.h>
#include <pcl/common/common.h>

namespace pcl
{
  namespace rec_3d_framework
  {

    inline double
    uniform_deviate (int seed)
    {
      double ran = seed * (1.0 / (RAND_MAX + 1.0));
      return ran;
    }

    inline void
    randomPointTriangle (double a1, double a2, double a3, double b1, double b2, double b3, double c1, double c2, double c3, Eigen::Vector4f& p)
    {
      float r1 = static_cast<float> (uniform_deviate (rand ()));
      float r2 = static_cast<float> (uniform_deviate (rand ()));
      float r1sqr = std::sqrt (r1);
      float OneMinR1Sqr = (1 - r1sqr);
      float OneMinR2 = (1 - r2);
      a1 *= OneMinR1Sqr;
      a2 *= OneMinR1Sqr;
      a3 *= OneMinR1Sqr;
      b1 *= OneMinR2;
      b2 *= OneMinR2;
      b3 *= OneMinR2;
      c1 = r1sqr * (r2 * c1 + b1) + a1;
      c2 = r1sqr * (r2 * c2 + b2) + a2;
      c3 = r1sqr * (r2 * c3 + b3) + a3;
      p[0] = static_cast<float> (c1);
      p[1] = static_cast<float> (c2);
      p[2] = static_cast<float> (c3);
      p[3] = 0.f;
    }

    inline void
    randPSurface (vtkPolyData * polydata, std::vector<double> * cumulativeAreas, double totalArea, Eigen::Vector4f& p)
    {
      float r = static_cast<float> (uniform_deviate (rand ()) * totalArea);

      std::vector<double>::iterator low = std::lower_bound (cumulativeAreas->begin (), cumulativeAreas->end (), r);
      vtkIdType el = static_cast<vtkIdType> (low - cumulativeAreas->begin ());

      double A[3], B[3], C[3];
      vtkIdType npts = 0;
      vtkIdType *ptIds = NULL;
      polydata->GetCellPoints (el, npts, ptIds);

      if (ptIds == NULL)
        return;

      polydata->GetPoint (ptIds[0], A);
      polydata->GetPoint (ptIds[1], B);
      polydata->GetPoint (ptIds[2], C);
      randomPointTriangle (A[0], A[1], A[2], B[0], B[1], B[2], C[0], C[1], C[2], p);
    }

    template<typename PointT>
      inline void
      uniform_sampling (vtkSmartPointer<vtkPolyData> polydata, size_t n_samples, typename pcl::PointCloud<PointT> & cloud_out)
      {
        polydata->BuildCells ();
        vtkSmartPointer < vtkCellArray > cells = polydata->GetPolys ();

        double p1[3], p2[3], p3[3], totalArea = 0;
        std::vector<double> cumulativeAreas (cells->GetNumberOfCells (), 0);
        size_t i = 0;
        vtkIdType npts = 0, *ptIds = NULL;
        for (cells->InitTraversal (); cells->GetNextCell (npts, ptIds); i++)
        {
          polydata->GetPoint (ptIds[0], p1);
          polydata->GetPoint (ptIds[1], p2);
          polydata->GetPoint (ptIds[2], p3);
          totalArea += vtkTriangle::TriangleArea (p1, p2, p3);
          cumulativeAreas[i] = totalArea;
        }

        cloud_out.points.resize (n_samples);
        cloud_out.width = static_cast<int> (n_samples);
        cloud_out.height = 1;

        for (i = 0; i < n_samples; i++)
        {
          Eigen::Vector4f p (0.f, 0.f, 0.f, 0.f);
          randPSurface (polydata, &cumulativeAreas, totalArea, p);
          cloud_out.points[i].x = static_cast<float> (p[0]);
          cloud_out.points[i].y = static_cast<float> (p[1]);
          cloud_out.points[i].z = static_cast<float> (p[2]);
        }
      }

    template<typename PointT>
      inline void
      uniform_sampling (std::string & file, size_t n_samples, typename pcl::PointCloud<PointT> & cloud_out, float scale = 1.f)
      {

        vtkSmartPointer < vtkPLYReader > reader = vtkSmartPointer<vtkPLYReader>::New ();
        reader->SetFileName (file.c_str ());

        vtkSmartPointer < vtkPolyDataMapper > mapper = vtkSmartPointer<vtkPolyDataMapper>::New ();

        if (scale == 1.f)
        {
          mapper->SetInputConnection (reader->GetOutputPort ());
          mapper->Update ();
        }
        else
        {
          vtkSmartPointer<vtkTransform> trans = vtkSmartPointer<vtkTransform>::New ();
          trans->Scale (scale, scale, scale);
          vtkSmartPointer < vtkTransformFilter > trans_filter = vtkSmartPointer<vtkTransformFilter>::New ();
          trans_filter->SetTransform (trans);
          trans_filter->SetInputConnection (reader->GetOutputPort ());
          trans_filter->Update ();
          mapper->SetInputConnection (trans_filter->GetOutputPort ());
          mapper->Update ();
        }

        vtkSmartPointer<vtkPolyData> poly = mapper->GetInput ();

        uniform_sampling (poly, n_samples, cloud_out);

      }

    inline void
    getVerticesAsPointCloud (vtkSmartPointer<vtkPolyData> polydata, pcl::PointCloud<pcl::PointXYZ> & cloud_out)
    {
      vtkPoints *points = polydata->GetPoints ();
      cloud_out.points.resize (points->GetNumberOfPoints ());
      cloud_out.width = static_cast<int> (cloud_out.points.size ());
      cloud_out.height = 1;
      cloud_out.is_dense = false;

      for (int i = 0; i < points->GetNumberOfPoints (); i++)
      {
        double p[3];
        points->GetPoint (i, p);
        cloud_out.points[i].x = static_cast<float> (p[0]);
        cloud_out.points[i].y = static_cast<float> (p[1]);
        cloud_out.points[i].z = static_cast<float> (p[2]);
      }
    }
  }
}

#endif /* REC_FRAMEWORK_UNIFORM_SAMPLING_H_ */
