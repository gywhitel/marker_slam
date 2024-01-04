// @author: Gao Yinghao
// By Xiaomi Robotics Lab
// email: gaoyinghao@xiaomi.com

# ifndef CV_MATH
# define CV_MATH

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <vector>
#include <eigen3/Eigen/Geometry>
#include <tf2/LinearMath/Transform.h>

/// @brief transform cv frame with respect to frame{rot_vec, trans_vec}
/// @param trans_vec 
/// @param rot_vec 
/// @param trans translation from marker board origin to center, computed according to markerLength and markerSeperation of the board. 
/// @return 
cv::Vec3d transform_cv_frame(cv::Vec3d trans_vec, cv::Vec3d rot_vec, Eigen::Vector2d trans)
{
  double angle = sqrt(rot_vec[0]*rot_vec[0] + rot_vec[1]*rot_vec[1] + rot_vec[2]*rot_vec[2]);
  using namespace Eigen;
  Vector3d translate(trans_vec[0], trans_vec[1], trans_vec[2]);
  Vector3d axis(rot_vec[0]/angle, rot_vec[1]/angle, rot_vec[2]/angle);
  AngleAxisd angle_axis(angle, axis);
  Isometry3d pose = Isometry3d::Identity();
  pose.translate(translate);
  pose.rotate(angle_axis);
  pose.translate(Vector3d(trans[0], trans[1], 0));
  translate = pose.translation();
  return cv::Vec3d(translate(0), translate(1), translate(2));
} 

cv::Vec3d marker_boadr_center_frame(cv::Vec3d trans_vec, cv::Vec3d rot_vec, cv::Ptr<cv::aruco::GridBoard> board)
{
  double angle = sqrt(rot_vec[0]*rot_vec[0] + rot_vec[1]*rot_vec[1] + rot_vec[2]*rot_vec[2]);
  using namespace Eigen;
  Vector3d translate(trans_vec[0], trans_vec[1], trans_vec[2]);
  Vector3d axis(rot_vec[0]/angle, rot_vec[1]/angle, rot_vec[2]/angle);
  AngleAxisd angle_axis(angle, axis);
  Isometry3d pose = Isometry3d::Identity();
  pose.translate(translate);
  pose.rotate(angle_axis);
  cv::Size size = board->getGridSize();
  double marker_length = board->getMarkerLength();
  double marker_seperation = board->getMarkerSeparation();
  
  double x, y;
  if (size.width % 2)   // odd
    x = size.width/double(2) * marker_length + size.width/2 * marker_seperation; 
  else  // even
    x = size.width/2 * marker_length + (size.width/2 - 0.5) * marker_seperation; 
  
  if (size.height % 2)
    y = size.height/double(2) * marker_length + size.height/2 * marker_seperation;
  else
    y = size.height/2 * marker_length + (size.height/2 - 0.5) * marker_seperation;

  pose.translate(Vector3d(x, y, 0));
  translate = pose.translation();
  return cv::Vec3d(translate(0), translate(1), translate(2));
} 


/// @brief Convert 3D point to pixel point
/// @param trans_vec 3D point
/// @param camera_matrix camera intrinsics matrix
/// @return The corresponding pixel point in image 
cv::Point2i trans_vec2pixel(cv::Vec3d trans_vec, cv::Mat camera_matrix)
{
  double fx = camera_matrix.at<double>(0,0);
  double fy = camera_matrix.at<double>(1,1);
  double ppx = camera_matrix.at<double>(0,2);
  double ppy = camera_matrix.at<double>(1,2);
  double px = trans_vec[0] * fx / trans_vec[2] + ppx;
  double py = trans_vec[1] * fx / trans_vec[2] + ppy;
  return cv::Point2i(px, py);
}

/// @brief Convert rotation vector (angle-axis) to quaternion
/// @param rot_vec 
/// @return 
Eigen::Quaterniond rotation_vector2quaternion(const cv::Vec3d& rot_vec)
{
  double angle = sqrt(rot_vec[0]*rot_vec[0] + rot_vec[1]*rot_vec[1] + rot_vec[2]*rot_vec[2]);
  using namespace Eigen;
  Vector3d axis(rot_vec[0]/angle, rot_vec[1]/angle, rot_vec[2]/angle);
  AngleAxisd angle_axis(angle, axis);
  
  return Quaterniond(angle_axis);
}

tf2::Quaternion rotVec2Quaternion(const cv::Vec3d& rot_vec){
  double angle = sqrt(rot_vec[0]*rot_vec[0] + rot_vec[1]*rot_vec[1] + rot_vec[2]*rot_vec[2]);
  auto rot_vec_normal = rot_vec / angle;
  tf2::Vector3 axis(rot_vec_normal(0), rot_vec_normal(1), rot_vec_normal(2));
  tf2::Quaternion q(axis, angle);
  return q;
}

double quaternion_angular_difference(Eigen::Quaterniond q1, Eigen::Quaterniond q2)
{
  return q1.dot(q2);
}

Eigen::AngleAxisd cv_rotation_vector2angle_axis(cv::Vec3d rot_vec)
{
  double angle = sqrt(rot_vec[0]*rot_vec[0] + rot_vec[1]*rot_vec[1] + rot_vec[2]*rot_vec[2]);
  using namespace Eigen;
  Vector3d axis(rot_vec[0]/angle, rot_vec[1]/angle, rot_vec[2]/angle);
  AngleAxisd angle_axis(angle, axis);
  return angle_axis;
}

cv::Vec3d angle_axis2rotation_vector(const Eigen::AngleAxisd& angleaxis)
{
  Eigen::Vector3d rot_vec = angleaxis.angle() * angleaxis.axis();
  return cv::Vec3d(rot_vec[0], rot_vec[1], rot_vec[2]);
}

/// @brief Convert frame (rotation vector) to right-hand frame
/// @return 
cv::Vec3d right_hand_frame(const cv::Vec3d& rot_vec)
{
  Eigen::AngleAxisd angle_axis = cv_rotation_vector2angle_axis(rot_vec);
  Eigen::Matrix3d rot_mat = angle_axis.toRotationMatrix();
  rot_mat.col(2) = rot_mat.col(0).cross(rot_mat.col(1));
  Eigen::AngleAxisd right_hand_angle_axis(rot_mat);
  return angle_axis2rotation_vector(right_hand_angle_axis);
}

# endif