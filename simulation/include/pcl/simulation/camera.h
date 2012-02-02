#ifndef PCL_CAMERA_HPP_
#define PCL_CAMERA_HPP_

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <pcl/pcl_macros.h>

namespace pcl
{

class PCL_EXPORTS Camera
{
public:
  Camera() : x_(0), y_(0), z_(0), roll_(0), pitch_(0), yaw_(0) {
    update_pose();
  }
  Camera(double x, double y, double z, double roll, double pitch, double yaw) : x_(x), y_(y), z_(z), roll_(roll), pitch_(pitch), yaw_(yaw) {
    update_pose();
  }

  double x() const {return x_;}
  void set_x(double x) {x_ = x; update_pose();}

  double y() const {return y_;}
  void set_y(double y) {y_ = y; update_pose();}

  double z() const {return z_;}
  void set_z(double z) {z_ = z; update_pose();}

  double roll() const {return roll_;}
  void set_roll(double roll) {roll_ = roll; update_pose();}

  double pitch() const {return pitch_;}
  void set_pitch(double pitch) {pitch_ = pitch; update_pose();}

  double yaw() const {return yaw_;}
  void set_yaw(double yaw) {yaw_ = yaw; update_pose();}

  Eigen::Isometry3d pose() const {return pose_; }

  void set(double x, double y, double z, double roll, double pitch, double yaw) {
    x_ = x; y_ = y; z_ = z;
    roll_ = roll; pitch_ = pitch; yaw_ = yaw;
    update_pose();
  }

  void move(double vx, double vy, double vz);
  
  // Return the pose of the camera:
  Eigen::Vector3d get_ypr() {
    
/*    // Convert Euler Angles to Quaternion
    double sy = sin(yaw_*0.5);
    double cy = cos(yaw_*0.5);
    double sp = sin(pitch_*0.5);
    double cp = cos(pitch_*0.5);
    double sr = sin(roll_*0.5);
    double cr = cos(roll_*0.5);
    double quat_w = cr*cp*cy + sr*sp*sy;
    double quat_x = sr*cp*cy - cr*sp*sy;
    double quat_y = cr*sp*cy + sr*cp*sy;
    double quat_z = cr*cp*sy - sr*sp*cy;
    
    Eigen::Isometry3f cpose;
    cpose.setIdentity();
    cpose.translation() << x_, y_ , z_ ;
    Eigen::Quaternionf m;
//    m  = 
    cpose.rotate(m);  */  
    
    return Eigen::Vector3d(yaw_,pitch_,roll_);
  }
  
  // Opposite Direction (for reference)
  // static void quat_to_euler(Eigen::Quaterniond q, double& yaw, double& pitch, double& roll) {
  //     const double q0 = q.w();
  //     const double q1 = q.x();
  //     const double q2 = q.y();
  //     const double q3 = q.z();
  //     roll = atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2));
  //     pitch = asin(2*(q0*q2-q3*q1));
  //     yaw = atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3));
  // }
  

  typedef boost::shared_ptr<Camera> Ptr;
  typedef boost::shared_ptr<const Camera> ConstPtr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
  void update_pose();
  double x_,y_,z_;
  double roll_,pitch_,yaw_;

  Eigen::Isometry3d pose_;
  
  
};
}
#endif /* PCL_CAMERA_HPP_ */
