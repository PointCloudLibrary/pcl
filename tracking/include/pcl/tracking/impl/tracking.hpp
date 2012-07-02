#ifndef PCL_TRACKING_IMPL_TRACKING_H_
#define PCL_TRACKING_IMPL_TRACKING_H_

#include <boost/random.hpp>
#include <pcl/common/eigen.h>
#include <ctime>

namespace pcl
{
  namespace tracking
  {
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
        x     += static_cast<float> (sampleNormal (mean[0], cov[0]));
        y     += static_cast<float> (sampleNormal (mean[1], cov[1]));
        z     += static_cast<float> (sampleNormal (mean[2], cov[2]));
        roll  += static_cast<float> (sampleNormal (mean[3], cov[3]));
        pitch += static_cast<float> (sampleNormal (mean[4], cov[4]));
        yaw   += static_cast<float> (sampleNormal (mean[5], cov[5]));
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

      // a[i]
      inline float operator [] (unsigned int i)
      {
        switch (i)
        {
          case 0: return x;
          case 1: return y;
          case 2: return z;
          case 3: return roll;
          case 4: return pitch;
          case 5: return yaw;
          default: return 0.0;
        }
      }
      
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
    
    inline std::ostream& operator << (std::ostream& os, const ParticleXYZRPY& p)
    {
      os << "(" << p.x << "," << p.y << "," << p.z << ","
         << p.roll << "," << p.pitch << "," << p.yaw << ")";
      return (os);
    }
    
    // a * k
    inline pcl::tracking::ParticleXYZRPY operator * (const ParticleXYZRPY& p, double val)
    {
      pcl::tracking::ParticleXYZRPY newp;
      newp.x     = static_cast<float> (p.x * val);
      newp.y     = static_cast<float> (p.y * val);
      newp.z     = static_cast<float> (p.z * val);
      newp.roll  = static_cast<float> (p.roll * val);
      newp.pitch = static_cast<float> (p.pitch * val);
      newp.yaw   = static_cast<float> (p.yaw * val);
      return (newp);
    }
    
    // a + b
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

    // a - b
    inline pcl::tracking::ParticleXYZRPY operator - (const ParticleXYZRPY& a, const ParticleXYZRPY& b)
    {
      pcl::tracking::ParticleXYZRPY newp;
      newp.x = a.x - b.x;
      newp.y = a.y - b.y;
      newp.z = a.z - b.z;
      newp.roll = a.roll - b.roll;
      newp.pitch = a.pitch - b.pitch;
      newp.yaw = a.yaw - b.yaw;
      return (newp);
    }
    
  }
}

#define PCL_STATE_POINT_TYPES \
  (pcl::tracking::ParticleXYZRPY)

#endif  // 
