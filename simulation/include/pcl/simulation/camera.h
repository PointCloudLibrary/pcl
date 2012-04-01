#ifndef PCL_SIMULATION_CAMERA_HPP_
#define PCL_SIMULATION_CAMERA_HPP_

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <pcl/pcl_macros.h>

namespace pcl
{
  namespace simulation
  {
    class PCL_EXPORTS Camera
    {
    public:
      typedef boost::shared_ptr<Camera> Ptr;
      typedef boost::shared_ptr<const Camera> ConstPtr;

      Camera () : x_(0), y_(0), z_(0), roll_(0), pitch_(0), yaw_(0)
      {
        updatePose();
      }

      Camera (double x, double y, double z,
              double roll, double pitch, double yaw) : x_ (x),
                                                       y_ (y),
                                                       z_ (z),
                                                       roll_ (roll),
                                                       pitch_ (pitch),
                                                       yaw_ (yaw)
      {
        updatePose ();
      }

      double x () const { return x_; }
      void set_x (double x) { x_ = x; updatePose (); }

      double y () const { return y_; }
      void set_y (double y) { y_ = y; updatePose(); }

      double z () const { return z_; }
      void set_z (double z) { z_ = z; updatePose(); }

      double
      roll () const { return roll_; }
      void
      set_roll (double roll) { roll_ = roll; updatePose (); }

      double
      pitch() const {return pitch_;}
      void
      set_pitch(double pitch) { pitch_ = pitch; updatePose (); }

      double
      yaw () const { return yaw_; }
      void
      set_yaw (double yaw) { yaw_ = yaw; updatePose (); }

      Eigen::Isometry3d
      pose () const { return pose_; }

      void set (double x, double y, double z, double roll, double pitch, double yaw)
      {
        x_ = x; y_ = y; z_ = z;
        roll_ = roll; pitch_ = pitch; yaw_ = yaw;
        updatePose();
      }

      void move (double vx, double vy, double vz);

      // Return the pose of the camera:
      Eigen::Vector3d get_ypr ()
      {
        return Eigen::Vector3d (yaw_, pitch_, roll_);
      }

      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    private:
      void
      updatePose ();
    
      double x_,y_,z_;
      double roll_,pitch_,yaw_;
    
      Eigen::Isometry3d pose_;
    };
  } // namespace - simulation
} // namespace - pcl

#endif /* PCL_SIMULATION_CAMERA_HPP_ */
