#ifndef PCL_GPU_TRACKING_PARTICLE_FILTER_H_
#define PCL_GPU_TRACKING_PARTICLE_FILTER_H_

#include <pcl/pcl_macros.h>
#include <pcl/gpu/containers/device_array.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PointIndices.h>
#include <pcl/pcl_macros.h>

#include <pcl/gpu/kinfu/pixel_rgb.h>
#include <pcl/tracking/particle_filter.h>

#include <Eigen/Dense>

#include "internal.h"

namespace pcl
{
  namespace gpu
  {	
    class ParticleFilterGPUTracker
    {
    public:
    	/** \brief Point type supported */
    	typedef pcl::PointXYZ PointType;
			//typedef pcl::Normal		NormalType;
			typedef pcl::RGB			PixelRGB;
			
			typedef pcl::PointXYZ		StateXYZ;
			typedef pcl::PointXYZ		StateRPY;
			
			typedef pcl::tracking::ParticleXYZRPY StateType;

			/** \brief Empty constructor. */
			ParticleFilterGPUTracker ()
			//: ParticleFilterTracker<PointInT, StateT> ()
			{
				tracker_name_ = "ParticleFilterGPUTracker";
			}
			
			/** \brief set the number of the particles.
			* \param particle_num the number of the particles.
			*/
			inline void
				setParticleNum (const int particle_num) { particle_num_ = particle_num; }
			
			/** \brief get the number of the particles. */
			inline int
				getParticleNum () const { return particle_num_; }

			 /** \brief set a pointer to a reference dataset to be tracked.
			 * \param ref a pointer to a PointCloud message
			 */
			inline void
				setReferenceCloud (const DeviceArray2D<PointType> &ref) { ref_ = ref; }
			
			/** \brief get a pointer to a reference dataset to be tracked. */
			inline DeviceArray2D<PointType> const
				getReferenceCloud () { return ref_; }

			int
				cols ();

			int
				rows ();

			virtual bool 
				operator() (const DeviceArray2D<PointType>& input, const DeviceArray2D<PixelRGB>& input_colors)
			{

			}

			virtual void
				setMotion (StateType motion)
			{ motion_ = motion; }
						
			virtual StateType
				getResult();						

    protected:
			std::string tracker_name_;

			virtual bool 
			initCompute()
			{

				//pcl::device::initParticles(particle_num_, particle_xyz_, particle_rpy_, particle_weight_ );
			}
			
			virtual void 
			computeTracking()
			{

			}						
			
			virtual void
				allocateBuffers()
			{
				particles_.create( particle_num_ );				

				random_number_generator_.create( particle_num_ );
				
			}
			
			// reference point cloud
			DeviceArray2D<PointType> ref_;

			DeviceArray2D<PixelRGB> ref_colors_;

			//DeviceArray2D<NormalType> ref_normals_;

			// input point cloud
			DeviceArray2D<PointType> input_;

			DeviceArray2D<PixelRGB> input_colors_;

			//DeviceArray2D<NormalType> input_normals_;
						
			//StateCloud particles_;
			DeviceArray<StateType> particles_;

			// random number generate state
			DeviceArray<curandState> rng_states;
						
			int particle_num_;

			std::vector<float> step_noise_covariance_;

      std::vector<float> initial_noise_covariance_;
        
      std::vector<float> initial_noise_mean_;

			StateType motion_;

			float motion_ratio_;

			bool use_colors_;

			StateType representative_state_;			

			/** \brief Height of input depth image. */
			int rows_;
			/** \brief Width of input depth image. */
			int cols_;

		};
  }
}

#endif // PCL_GPU_TRACKING_PARTICLE_FILTER_H_
