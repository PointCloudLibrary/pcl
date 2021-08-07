#ifndef PCL_TRACKING_IMPL_TRACKING_H_
#define PCL_TRACKING_IMPL_TRACKING_H_

#include <pcl/common/eigen.h>
#include <pcl/tracking/tracking.h>
#include <pcl/memory.h>
#include <pcl/pcl_macros.h>

namespace pcl {
namespace tracking {
struct _ParticleXYZRPY {
  PCL_ADD_POINT4D;
  union {
    struct {
      float roll;
      float pitch;
      float yaw;
      float weight;
    };
    float data_c[4];
  };
};

// particle definition
struct EIGEN_ALIGN16 ParticleXYZRPY : public _ParticleXYZRPY {
  inline ParticleXYZRPY()
  {
    x = y = z = roll = pitch = yaw = 0.0;
    data[3] = 1.0f;
  }

  inline ParticleXYZRPY(float _x, float _y, float _z)
  {
    x = _x;
    y = _y;
    z = _z;
    roll = pitch = yaw = 0.0;
    data[3] = 1.0f;
  }

  inline ParticleXYZRPY(
      float _x, float _y, float _z, float _roll, float _pitch, float _yaw)
  {
    x = _x;
    y = _y;
    z = _z;
    roll = _roll;
    pitch = _pitch;
    yaw = _yaw;
    data[3] = 1.0f;
  }

  inline static int
  stateDimension()
  {
    return 6;
  }

  void
  sample(const std::vector<double>& mean, const std::vector<double>& cov)
  {
    x += static_cast<float>(sampleNormal(mean[0], cov[0]));
    y += static_cast<float>(sampleNormal(mean[1], cov[1]));
    z += static_cast<float>(sampleNormal(mean[2], cov[2]));

    // The roll, pitch, yaw space is not Euclidean, so if we sample roll,
    // pitch, and yaw independently, we bias our sampling in a complicated
    // way that depends on where in the space we are sampling.
    //
    // A solution is to always sample around mean = 0 and project our
    // samples onto the space of rotations, SO(3).  We rely on the fact
    // that we are sampling with small variance, so we do not bias
    // ourselves too much with this projection.  We then rotate our
    // samples by the user's requested mean.  The benefit of this approach
    // is that our distribution's properties are consistent over the space
    // of rotations.
    Eigen::Matrix3f current_rotation;
    current_rotation = getTransformation(x, y, z, roll, pitch, yaw).rotation();
    Eigen::Quaternionf q_current_rotation(current_rotation);

    Eigen::Matrix3f mean_rotation;
    mean_rotation =
        getTransformation(mean[0], mean[1], mean[2], mean[3], mean[4], mean[5])
            .rotation();
    Eigen::Quaternionf q_mean_rotation(mean_rotation);

    // Scales 1.0 radians of variance in RPY sampling into equivalent units for
    // quaternion sampling.
    const float scale_factor = 0.2862;

    float a = sampleNormal(0, scale_factor * cov[3]);
    float b = sampleNormal(0, scale_factor * cov[4]);
    float c = sampleNormal(0, scale_factor * cov[5]);

    Eigen::Vector4f vec_sample_mean_0(a, b, c, 1);
    Eigen::Quaternionf q_sample_mean_0(vec_sample_mean_0);
    q_sample_mean_0.normalize();

    Eigen::Quaternionf q_sample_user_mean =
        q_sample_mean_0 * q_mean_rotation * q_current_rotation;

    Eigen::Affine3f affine_R(q_sample_user_mean.toRotationMatrix());
    pcl::getEulerAngles(affine_R, roll, pitch, yaw);
  }

  void
  zero()
  {
    x = 0.0;
    y = 0.0;
    z = 0.0;
    roll = 0.0;
    pitch = 0.0;
    yaw = 0.0;
  }

  inline Eigen::Affine3f
  toEigenMatrix() const
  {
    return getTransformation(x, y, z, roll, pitch, yaw);
  }

  static ParticleXYZRPY
  toState(const Eigen::Affine3f& trans)
  {
    float trans_x, trans_y, trans_z, trans_roll, trans_pitch, trans_yaw;
    getTranslationAndEulerAngles(
        trans, trans_x, trans_y, trans_z, trans_roll, trans_pitch, trans_yaw);
    return {trans_x, trans_y, trans_z, trans_roll, trans_pitch, trans_yaw};
  }

  // a[i]
  inline float
  operator[](unsigned int i)
  {
    switch (i) {
    case 0:
      return x;
    case 1:
      return y;
    case 2:
      return z;
    case 3:
      return roll;
    case 4:
      return pitch;
    case 5:
      return yaw;
    default:
      return 0.0;
    }
  }

  PCL_MAKE_ALIGNED_OPERATOR_NEW
};

inline std::ostream&
operator<<(std::ostream& os, const ParticleXYZRPY& p)
{
  os << "(" << p.x << "," << p.y << "," << p.z << "," << p.roll << "," << p.pitch << ","
     << p.yaw << ")";
  return (os);
}

// a * k
inline ParticleXYZRPY
operator*(const ParticleXYZRPY& p, double val)
{
  pcl::tracking::ParticleXYZRPY newp;
  newp.x = static_cast<float>(p.x * val);
  newp.y = static_cast<float>(p.y * val);
  newp.z = static_cast<float>(p.z * val);
  newp.roll = static_cast<float>(p.roll * val);
  newp.pitch = static_cast<float>(p.pitch * val);
  newp.yaw = static_cast<float>(p.yaw * val);
  return (newp);
}

// a + b
inline ParticleXYZRPY
operator+(const ParticleXYZRPY& a, const ParticleXYZRPY& b)
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
inline ParticleXYZRPY
operator-(const ParticleXYZRPY& a, const ParticleXYZRPY& b)
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

} // namespace tracking
} // namespace pcl

//########################################################################33

namespace pcl {
namespace tracking {
struct _ParticleXYZR {
  PCL_ADD_POINT4D;
  union {
    struct {
      float roll;
      float pitch;
      float yaw;
      float weight;
    };
    float data_c[4];
  };
};

// particle definition
struct EIGEN_ALIGN16 ParticleXYZR : public _ParticleXYZR {
  inline ParticleXYZR()
  {
    x = y = z = roll = pitch = yaw = 0.0;
    data[3] = 1.0f;
  }

  inline ParticleXYZR(float _x, float _y, float _z)
  {
    x = _x;
    y = _y;
    z = _z;
    roll = pitch = yaw = 0.0;
    data[3] = 1.0f;
  }

  inline ParticleXYZR(float _x, float _y, float _z, float, float _pitch, float)
  {
    x = _x;
    y = _y;
    z = _z;
    roll = 0;
    pitch = _pitch;
    yaw = 0;
    data[3] = 1.0f;
  }

  inline static int
  stateDimension()
  {
    return 6;
  }

  void
  sample(const std::vector<double>& mean, const std::vector<double>& cov)
  {
    x += static_cast<float>(sampleNormal(mean[0], cov[0]));
    y += static_cast<float>(sampleNormal(mean[1], cov[1]));
    z += static_cast<float>(sampleNormal(mean[2], cov[2]));
    roll = 0;
    pitch += static_cast<float>(sampleNormal(mean[4], cov[4]));
    yaw = 0;
  }

  void
  zero()
  {
    x = 0.0;
    y = 0.0;
    z = 0.0;
    roll = 0.0;
    pitch = 0.0;
    yaw = 0.0;
  }

  inline Eigen::Affine3f
  toEigenMatrix() const
  {
    return getTransformation(x, y, z, roll, pitch, yaw);
  }

  static ParticleXYZR
  toState(const Eigen::Affine3f& trans)
  {
    float trans_x, trans_y, trans_z, trans_roll, trans_pitch, trans_yaw;
    getTranslationAndEulerAngles(
        trans, trans_x, trans_y, trans_z, trans_roll, trans_pitch, trans_yaw);
    return (pcl::tracking::ParticleXYZR(trans_x, trans_y, trans_z, 0, trans_pitch, 0));
  }

  // a[i]
  inline float
  operator[](unsigned int i)
  {
    switch (i) {
    case 0:
      return x;
    case 1:
      return y;
    case 2:
      return z;
    case 3:
      return roll;
    case 4:
      return pitch;
    case 5:
      return yaw;
    default:
      return 0.0;
    }
  }

  PCL_MAKE_ALIGNED_OPERATOR_NEW
};

inline std::ostream&
operator<<(std::ostream& os, const ParticleXYZR& p)
{
  os << "(" << p.x << "," << p.y << "," << p.z << "," << p.roll << "," << p.pitch << ","
     << p.yaw << ")";
  return (os);
}

// a * k
inline ParticleXYZR
operator*(const ParticleXYZR& p, double val)
{
  pcl::tracking::ParticleXYZR newp;
  newp.x = static_cast<float>(p.x * val);
  newp.y = static_cast<float>(p.y * val);
  newp.z = static_cast<float>(p.z * val);
  newp.roll = static_cast<float>(p.roll * val);
  newp.pitch = static_cast<float>(p.pitch * val);
  newp.yaw = static_cast<float>(p.yaw * val);
  return (newp);
}

// a + b
inline ParticleXYZR
operator+(const ParticleXYZR& a, const ParticleXYZR& b)
{
  pcl::tracking::ParticleXYZR newp;
  newp.x = a.x + b.x;
  newp.y = a.y + b.y;
  newp.z = a.z + b.z;
  newp.roll = 0;
  newp.pitch = a.pitch + b.pitch;
  newp.yaw = 0.0;
  return (newp);
}

// a - b
inline ParticleXYZR
operator-(const ParticleXYZR& a, const ParticleXYZR& b)
{
  pcl::tracking::ParticleXYZR newp;
  newp.x = a.x - b.x;
  newp.y = a.y - b.y;
  newp.z = a.z - b.z;
  newp.roll = 0.0;
  newp.pitch = a.pitch - b.pitch;
  newp.yaw = 0.0;
  return (newp);
}

} // namespace tracking
} // namespace pcl

//########################################################################33

namespace pcl {
namespace tracking {
struct _ParticleXYRPY {
  PCL_ADD_POINT4D;
  union {
    struct {
      float roll;
      float pitch;
      float yaw;
      float weight;
    };
    float data_c[4];
  };
};

// particle definition
struct EIGEN_ALIGN16 ParticleXYRPY : public _ParticleXYRPY {
  inline ParticleXYRPY()
  {
    x = y = z = roll = pitch = yaw = 0.0;
    data[3] = 1.0f;
  }

  inline ParticleXYRPY(float _x, float, float _z)
  {
    x = _x;
    y = 0;
    z = _z;
    roll = pitch = yaw = 0.0;
    data[3] = 1.0f;
  }

  inline ParticleXYRPY(float _x, float, float _z, float _roll, float _pitch, float _yaw)
  {
    x = _x;
    y = 0;
    z = _z;
    roll = _roll;
    pitch = _pitch;
    yaw = _yaw;
    data[3] = 1.0f;
  }

  inline static int
  stateDimension()
  {
    return 6;
  }

  void
  sample(const std::vector<double>& mean, const std::vector<double>& cov)
  {
    x += static_cast<float>(sampleNormal(mean[0], cov[0]));
    y = 0;
    z += static_cast<float>(sampleNormal(mean[2], cov[2]));
    roll += static_cast<float>(sampleNormal(mean[3], cov[3]));
    pitch += static_cast<float>(sampleNormal(mean[4], cov[4]));
    yaw += static_cast<float>(sampleNormal(mean[5], cov[5]));
  }

  void
  zero()
  {
    x = 0.0;
    y = 0.0;
    z = 0.0;
    roll = 0.0;
    pitch = 0.0;
    yaw = 0.0;
  }

  inline Eigen::Affine3f
  toEigenMatrix() const
  {
    return getTransformation(x, y, z, roll, pitch, yaw);
  }

  static ParticleXYRPY
  toState(const Eigen::Affine3f& trans)
  {
    float trans_x, trans_y, trans_z, trans_roll, trans_pitch, trans_yaw;
    getTranslationAndEulerAngles(
        trans, trans_x, trans_y, trans_z, trans_roll, trans_pitch, trans_yaw);
    return (pcl::tracking::ParticleXYRPY(
        trans_x, 0, trans_z, trans_roll, trans_pitch, trans_yaw));
  }

  // a[i]
  inline float
  operator[](unsigned int i)
  {
    switch (i) {
    case 0:
      return x;
    case 1:
      return y;
    case 2:
      return z;
    case 3:
      return roll;
    case 4:
      return pitch;
    case 5:
      return yaw;
    default:
      return 0.0;
    }
  }

  PCL_MAKE_ALIGNED_OPERATOR_NEW
};

inline std::ostream&
operator<<(std::ostream& os, const ParticleXYRPY& p)
{
  os << "(" << p.x << "," << p.y << "," << p.z << "," << p.roll << "," << p.pitch << ","
     << p.yaw << ")";
  return (os);
}

// a * k
inline ParticleXYRPY
operator*(const ParticleXYRPY& p, double val)
{
  pcl::tracking::ParticleXYRPY newp;
  newp.x = static_cast<float>(p.x * val);
  newp.y = static_cast<float>(p.y * val);
  newp.z = static_cast<float>(p.z * val);
  newp.roll = static_cast<float>(p.roll * val);
  newp.pitch = static_cast<float>(p.pitch * val);
  newp.yaw = static_cast<float>(p.yaw * val);
  return (newp);
}

// a + b
inline ParticleXYRPY
operator+(const ParticleXYRPY& a, const ParticleXYRPY& b)
{
  pcl::tracking::ParticleXYRPY newp;
  newp.x = a.x + b.x;
  newp.y = 0;
  newp.z = a.z + b.z;
  newp.roll = a.roll + b.roll;
  newp.pitch = a.pitch + b.pitch;
  newp.yaw = a.yaw + b.yaw;
  return (newp);
}

// a - b
inline ParticleXYRPY
operator-(const ParticleXYRPY& a, const ParticleXYRPY& b)
{
  pcl::tracking::ParticleXYRPY newp;
  newp.x = a.x - b.x;
  newp.z = a.z - b.z;
  newp.y = 0;
  newp.roll = a.roll - b.roll;
  newp.pitch = a.pitch - b.pitch;
  newp.yaw = a.yaw - b.yaw;
  return (newp);
}

} // namespace tracking
} // namespace pcl

//########################################################################33

namespace pcl {
namespace tracking {
struct _ParticleXYRP {
  PCL_ADD_POINT4D;
  union {
    struct {
      float roll;
      float pitch;
      float yaw;
      float weight;
    };
    float data_c[4];
  };
};

// particle definition
struct EIGEN_ALIGN16 ParticleXYRP : public _ParticleXYRP {
  inline ParticleXYRP()
  {
    x = y = z = roll = pitch = yaw = 0.0;
    data[3] = 1.0f;
  }

  inline ParticleXYRP(float _x, float, float _z)
  {
    x = _x;
    y = 0;
    z = _z;
    roll = pitch = yaw = 0.0;
    data[3] = 1.0f;
  }

  inline ParticleXYRP(float _x, float, float _z, float, float _pitch, float _yaw)
  {
    x = _x;
    y = 0;
    z = _z;
    roll = 0;
    pitch = _pitch;
    yaw = _yaw;
    data[3] = 1.0f;
  }

  inline static int
  stateDimension()
  {
    return 6;
  }

  void
  sample(const std::vector<double>& mean, const std::vector<double>& cov)
  {
    x += static_cast<float>(sampleNormal(mean[0], cov[0]));
    y = 0;
    z += static_cast<float>(sampleNormal(mean[2], cov[2]));
    roll = 0;
    pitch += static_cast<float>(sampleNormal(mean[4], cov[4]));
    yaw += static_cast<float>(sampleNormal(mean[5], cov[5]));
  }

  void
  zero()
  {
    x = 0.0;
    y = 0.0;
    z = 0.0;
    roll = 0.0;
    pitch = 0.0;
    yaw = 0.0;
  }

  inline Eigen::Affine3f
  toEigenMatrix() const
  {
    return getTransformation(x, y, z, roll, pitch, yaw);
  }

  static ParticleXYRP
  toState(const Eigen::Affine3f& trans)
  {
    float trans_x, trans_y, trans_z, trans_roll, trans_pitch, trans_yaw;
    getTranslationAndEulerAngles(
        trans, trans_x, trans_y, trans_z, trans_roll, trans_pitch, trans_yaw);
    return (
        pcl::tracking::ParticleXYRP(trans_x, 0, trans_z, 0, trans_pitch, trans_yaw));
  }

  // a[i]
  inline float
  operator[](unsigned int i)
  {
    switch (i) {
    case 0:
      return x;
    case 1:
      return y;
    case 2:
      return z;
    case 3:
      return roll;
    case 4:
      return pitch;
    case 5:
      return yaw;
    default:
      return 0.0;
    }
  }

  PCL_MAKE_ALIGNED_OPERATOR_NEW
};

inline std::ostream&
operator<<(std::ostream& os, const ParticleXYRP& p)
{
  os << "(" << p.x << "," << p.y << "," << p.z << "," << p.roll << "," << p.pitch << ","
     << p.yaw << ")";
  return (os);
}

// a * k
inline ParticleXYRP
operator*(const ParticleXYRP& p, double val)
{
  pcl::tracking::ParticleXYRP newp;
  newp.x = static_cast<float>(p.x * val);
  newp.y = static_cast<float>(p.y * val);
  newp.z = static_cast<float>(p.z * val);
  newp.roll = static_cast<float>(p.roll * val);
  newp.pitch = static_cast<float>(p.pitch * val);
  newp.yaw = static_cast<float>(p.yaw * val);
  return (newp);
}

// a + b
inline ParticleXYRP
operator+(const ParticleXYRP& a, const ParticleXYRP& b)
{
  pcl::tracking::ParticleXYRP newp;
  newp.x = a.x + b.x;
  newp.y = 0;
  newp.z = a.z + b.z;
  newp.roll = 0;
  newp.pitch = a.pitch + b.pitch;
  newp.yaw = a.yaw + b.yaw;
  return (newp);
}

// a - b
inline ParticleXYRP
operator-(const ParticleXYRP& a, const ParticleXYRP& b)
{
  pcl::tracking::ParticleXYRP newp;
  newp.x = a.x - b.x;
  newp.z = a.z - b.z;
  newp.y = 0;
  newp.roll = 0.0;
  newp.pitch = a.pitch - b.pitch;
  newp.yaw = a.yaw - b.yaw;
  return (newp);
}

} // namespace tracking
} // namespace pcl

//########################################################################33

namespace pcl {
namespace tracking {
struct _ParticleXYR {
  PCL_ADD_POINT4D;
  union {
    struct {
      float roll;
      float pitch;
      float yaw;
      float weight;
    };
    float data_c[4];
  };
};

// particle definition
struct EIGEN_ALIGN16 ParticleXYR : public _ParticleXYR {
  inline ParticleXYR()
  {
    x = y = z = roll = pitch = yaw = 0.0;
    data[3] = 1.0f;
  }

  inline ParticleXYR(float _x, float, float _z)
  {
    x = _x;
    y = 0;
    z = _z;
    roll = pitch = yaw = 0.0;
    data[3] = 1.0f;
  }

  inline ParticleXYR(float _x, float, float _z, float, float _pitch, float)
  {
    x = _x;
    y = 0;
    z = _z;
    roll = 0;
    pitch = _pitch;
    yaw = 0;
    data[3] = 1.0f;
  }

  inline static int
  stateDimension()
  {
    return 6;
  }

  void
  sample(const std::vector<double>& mean, const std::vector<double>& cov)
  {
    x += static_cast<float>(sampleNormal(mean[0], cov[0]));
    y = 0;
    z += static_cast<float>(sampleNormal(mean[2], cov[2]));
    roll = 0;
    pitch += static_cast<float>(sampleNormal(mean[4], cov[4]));
    yaw = 0;
  }

  void
  zero()
  {
    x = 0.0;
    y = 0.0;
    z = 0.0;
    roll = 0.0;
    pitch = 0.0;
    yaw = 0.0;
  }

  inline Eigen::Affine3f
  toEigenMatrix() const
  {
    return getTransformation(x, y, z, roll, pitch, yaw);
  }

  static ParticleXYR
  toState(const Eigen::Affine3f& trans)
  {
    float trans_x, trans_y, trans_z, trans_roll, trans_pitch, trans_yaw;
    getTranslationAndEulerAngles(
        trans, trans_x, trans_y, trans_z, trans_roll, trans_pitch, trans_yaw);
    return (pcl::tracking::ParticleXYR(trans_x, 0, trans_z, 0, trans_pitch, 0));
  }

  // a[i]
  inline float
  operator[](unsigned int i)
  {
    switch (i) {
    case 0:
      return x;
    case 1:
      return y;
    case 2:
      return z;
    case 3:
      return roll;
    case 4:
      return pitch;
    case 5:
      return yaw;
    default:
      return 0.0;
    }
  }

  PCL_MAKE_ALIGNED_OPERATOR_NEW
};

inline std::ostream&
operator<<(std::ostream& os, const ParticleXYR& p)
{
  os << "(" << p.x << "," << p.y << "," << p.z << "," << p.roll << "," << p.pitch << ","
     << p.yaw << ")";
  return (os);
}

// a * k
inline ParticleXYR
operator*(const ParticleXYR& p, double val)
{
  pcl::tracking::ParticleXYR newp;
  newp.x = static_cast<float>(p.x * val);
  newp.y = static_cast<float>(p.y * val);
  newp.z = static_cast<float>(p.z * val);
  newp.roll = static_cast<float>(p.roll * val);
  newp.pitch = static_cast<float>(p.pitch * val);
  newp.yaw = static_cast<float>(p.yaw * val);
  return (newp);
}

// a + b
inline ParticleXYR
operator+(const ParticleXYR& a, const ParticleXYR& b)
{
  pcl::tracking::ParticleXYR newp;
  newp.x = a.x + b.x;
  newp.y = 0;
  newp.z = a.z + b.z;
  newp.roll = 0;
  newp.pitch = a.pitch + b.pitch;
  newp.yaw = 0.0;
  return (newp);
}

// a - b
inline ParticleXYR
operator-(const ParticleXYR& a, const ParticleXYR& b)
{
  pcl::tracking::ParticleXYR newp;
  newp.x = a.x - b.x;
  newp.z = a.z - b.z;
  newp.y = 0;
  newp.roll = 0.0;
  newp.pitch = a.pitch - b.pitch;
  newp.yaw = 0.0;
  return (newp);
}

} // namespace tracking
} // namespace pcl

#define PCL_STATE_POINT_TYPES                                                          \
  (pcl::tracking::ParticleXYR)(pcl::tracking::ParticleXYZRPY)(                         \
      pcl::tracking::ParticleXYZR)(pcl::tracking::ParticleXYRPY)(                      \
      pcl::tracking::ParticleXYRP)

#endif //
