#ifndef MRG_CAMERA_HPP_
#define MRG_CAMERA_HPP_

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include <Eigen/StdVector>

namespace mrg
{

class Camera
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
#endif /* MRG_CAMERA_HPP_ */
