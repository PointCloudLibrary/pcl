/*
 * render_views_tesselated_sphere.h
 *
 *  Created on: Dec 23, 2011
 *      Author: aitor
 */

#ifndef RENDER_VIEWS_TESSELATED_SPHERE_H_
#define RENDER_VIEWS_TESSELATED_SPHERE_H_

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <pcl/common/common.h>
#include <boost/function.hpp>

namespace pcl
{
  namespace apps
  {
    /** \brief @b Class to render synthetic views of a 3D mesh using a tesselated sphere
     * NOTE: This class should replace renderViewTesselatedSphere from pcl::visualization.
     * Some extensions are planned in the near future to this class like removal of duplicated views for
     * symmetrical objects, generation of RGB synthetic clouds when RGB available on mesh, etc.
     * \author Aitor Aldoma
     * \ingroup apps
     */
    class PCL_EXPORTS RenderViewsTesselatedSphere
    {
    private:
      std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > poses_;
      std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> generated_views_;
      std::vector<float> entropies_;
      int resolution_;
      int tesselation_level_;
      bool use_vertices_;
      float view_angle_;
      float radius_sphere_;
      bool compute_entropy_;
      vtkSmartPointer<vtkPolyData> polydata_;
      bool gen_organized_;
      boost::function<bool
      (const Eigen::Vector3f &)> campos_constraints_func_;

      struct camPosConstraintsAllTrue
      {
        bool
        operator() (const Eigen::Vector3f & /*pos*/) const
        {
          return true;
        }
        ;
      };

    public:
      RenderViewsTesselatedSphere ()
      {
        resolution_ = 150;
        tesselation_level_ = 1;
        use_vertices_ = false;
        view_angle_ = 57;
        radius_sphere_ = 1.f;
        compute_entropy_ = false;
        gen_organized_ = false;
        campos_constraints_func_ = camPosConstraintsAllTrue ();
      }

      void
      setCamPosConstraints (boost::function<bool (const Eigen::Vector3f &)> & bb)
      {
        campos_constraints_func_ = bb;
      }

      /* \brief Indicates wether to generate organized or unorganized data
       * \param b organized/unorganized
       */
      void
      setGenOrganized (bool b)
      {
        gen_organized_ = b;
      }

      /* \brief Sets the size of the render window
       * \param res resolution size
       */
      void
      setResolution (int res)
      {
        resolution_ = res;
      }

      /* \brief Wether to use the vertices or triangle centers of the tesselated sphere
       * \param use true indicates to use vertices, false triangle centers
       */

      void
      setUseVertices (bool use)
      {
        use_vertices_ = use;
      }

      /* \brief Radius of the sphere where the virtual camera will be placed
       * \param use true indicates to use vertices, false triangle centers
       */
      void
      setRadiusSphere (float radius)
      {
        radius_sphere_ = radius;
      }

      /* \brief Wether to compute the entropies (level of occlusions for each view)
       * \param compute true to compute entropies, false otherwise
       */
      void
      setComputeEntropies (bool compute)
      {
        compute_entropy_ = compute;
      }

      /* \brief How many times the icosahedron should be tesselated. Results in more or less camera positions and generated views.
       * \param level amount of tesselation
       */
      void
      setTesselationLevel (int level)
      {
        tesselation_level_ = level;
      }

      /* \brief Sets the view angle of the virtual camera
       * \param angle view angle in degrees
       */
      void
      setViewAngle (float angle)
      {
        view_angle_ = angle;
      }

      /* \brief adds the mesh to be used as a vtkPolyData
       * \param polydata vtkPolyData object
       */
      void
      addModelFromPolyData (vtkSmartPointer<vtkPolyData> &polydata)
      {
        polydata_ = polydata;
      }

      /* \brief performs the rendering and stores the generated information
       */
      void
      generateViews ();

      /* \brief Get the generated poses for the generated views
       * \param poses 4x4 matrices representing the pose of the cloud relative to the model coordinate system
       */
      void
      getPoses (std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > & poses)
      {
        poses = poses_;
      }

      /* \brief Get the generated views
       * \param views generated pointclouds in camera coordinates
       */
      void
      getViews (std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> & views)
      {
        views = generated_views_;
      }

      /* \brief Get the entropies (level of occlusions) for the views
       * \param entropies level of occlusions
       */
      void
      getEntropies (std::vector<float> & entropies)
      {
        entropies = entropies_;
      }
    };

  }
}

#endif /* RENDER_VIEWS_TESSELATED_SPHERE_H_ */
