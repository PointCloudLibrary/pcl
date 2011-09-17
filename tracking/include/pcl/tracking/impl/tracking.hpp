#ifndef PCL_TRACKING_IMPL_TRACKING_H_
#define PCL_TRACKING_IMPL_TRACKING_H_

#include <boost/random.hpp>
#include <pcl/common/transform.h>
#include <ctime>

namespace pcl
{
  namespace tracking
  {
    double
    SampleNormal (double mean, double sigma)
    {
      using namespace boost;
      static mt19937 rng(static_cast<unsigned> (std::time (0)));
      
      normal_distribution<double> norm_dist (mean, sqrt (sigma));
      
      variate_generator<mt19937&, normal_distribution<double> >
        normal_sampler (rng, norm_dist);
      
      return normal_sampler ();
    }
    
    struct _ParticleXYZRPY
    {
      PCL_ADD_POINT4D;
      union
      {
        struct
        {
          float roll;
          float pitch;
          float yaw;
          float weight;
        };
        float data_c[4];
      };
    };

    // particle definition
    struct EIGEN_ALIGN16 ParticleXYZRPY : public _ParticleXYZRPY
    {
      inline ParticleXYZRPY ()
      {
        x = y = z = roll = pitch = yaw = 0.0;
        data[3] = 1.0f;
      }

      inline ParticleXYZRPY (float _x, float _y, float _z)
      {
        x = _x; y = _y; z = _z;
        roll = pitch = yaw = 0.0;
        data[3] = 1.0f;
      }

      inline ParticleXYZRPY (float _x, float _y, float _z, float _roll, float _pitch, float _yaw)
      {
        x = _x; y = _y; z = _z;
        roll = _roll; pitch = _pitch; yaw = _yaw;
        data[3] = 1.0f;
      }

      inline static int
      stateDimension () { return 6; }
      
      void
      sample (const std::vector<double>& mean, const std::vector<double>& cov)
      {
        x += SampleNormal (mean[0], cov[0]);
        y += SampleNormal (mean[1], cov[1]);
        z += SampleNormal (mean[2], cov[2]);
        roll += SampleNormal (mean[3], cov[3]);
        pitch += SampleNormal (mean[4], cov[4]);
        yaw += SampleNormal (mean[5], cov[5]);
      }

      void
      zero ()
      {
        x = 0.0;
        y = 0.0;
        z = 0.0;
        roll = 0.0;
        pitch = 0.0;
        yaw = 0.0;
      }

      inline Eigen::Affine3f
      toEigenMatrix () const
      {
        return getTransformation(x, y, z, roll, pitch, yaw);
      }

      static pcl::tracking::ParticleXYZRPY
      toState (const Eigen::Affine3f &trans)
      {
        float trans_x, trans_y, trans_z, trans_roll, trans_pitch, trans_yaw;
        getTranslationAndEulerAngles (trans,
                                      trans_x, trans_y, trans_z,
                                      trans_roll, trans_pitch, trans_yaw);
        return pcl::tracking::ParticleXYZRPY (trans_x, trans_y, trans_z, trans_roll, trans_pitch, trans_yaw);
      }
      
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
    
    inline std::ostream& operator << (std::ostream& os, const ParticleXYZRPY& p)
    {
      os << "(" << p.x << "," << p.y << "," << p.z << ","
         << p.roll << "," << p.pitch << "," << p.yaw << ")";
      return (os);
    }
    
    inline pcl::tracking::ParticleXYZRPY operator * (const ParticleXYZRPY& p, double val)
    {
      pcl::tracking::ParticleXYZRPY newp;
      newp.x = p.x * val;
      newp.y = p.y * val;
      newp.z = p.z * val;
      newp.roll = p.roll * val;
      newp.pitch = p.pitch * val;
      newp.yaw = p.yaw * val;
      return (newp);
    }

    inline pcl::tracking::ParticleXYZRPY operator + (const ParticleXYZRPY& a, const ParticleXYZRPY& b)
    {
      pcl::tracking::ParticleXYZRPY newp;
      newp.x = a.x + b.x;
      newp.y = a.y + b.y;
      newp.z = a.z + b.z;
      newp.roll = a.roll + b.roll;
      newp.pitch = a.pitch + b.pitch;
      newp.yaw = a.yaw + b.yaw;
      return (newp);
    }
    
  }
}

template <typename PointInT, typename StateT> bool
pcl::tracking::Tracker<PointInT, StateT>::initCompute ()
{
  if (!PCLBase<PointInT>::initCompute ())
  {
    PCL_ERROR ("[pcl::%s::initCompute] PCLBase::Init failed.\n", getClassName ().c_str ());
    return (false);
  }

  // If the dataset is empty, just return
  if (input_->points.empty ())
  {
    PCL_ERROR ("[pcl::%s::compute] input_ is empty!\n", getClassName ().c_str ());
    // Cleanup
    deinitCompute ();
    return (false);
  }

  return (true);
}


template <typename PointInT, typename StateT> void
pcl::tracking::Tracker<PointInT, StateT>::compute ()
{
  if (!initCompute ())
    return;
  
  computeTracking ();
  deinitCompute ();
}


#endif  // 
