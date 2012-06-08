/*
 * scene.hpp
 *
 *  Created on: Aug 16, 2011
 *      Author: Hordur Johannsson
 */

#ifndef PCL_SIMULATION_SCENE_HPP_
#define PCL_SIMULATION_SCENE_HPP_

#include <boost/shared_ptr.hpp>

#include <pcl/pcl_macros.h>
//#include <pcl/win32_macros.h>

#include <pcl/simulation/camera.h>
#include <pcl/simulation/model.h>

namespace pcl
{
  namespace simulation
  {
    class PCL_EXPORTS Scene
    {
    public:
      typedef boost::shared_ptr<Scene> Ptr;
      typedef boost::shared_ptr<Scene> ConstPtr;

      void
      draw ();

      void
      add (Model::Ptr model);

      void
      addCompleteModel (std::vector<Model::Ptr> model);

      void
      clear ();

    private:
      std::vector<Model::Ptr> models_;
    };
  
  } // namespace - simulation
} // namespace - pcl

#endif
