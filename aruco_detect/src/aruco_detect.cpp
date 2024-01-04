/*
 * Copyright (c) 2017-20, Ubiquity Robotics Inc., Austin Hendrix
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of the FreeBSD Project.
 *
 */

#include <assert.h>
#include <sys/time.h>
#include <unistd.h>
#include <math.h>

#include <rclcpp/rclcpp.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/msg/marker.h>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>


#include <std_srvs/srv/set_bool.hpp>
#include <std_msgs/msg/string.hpp>

#include "fiducial_msgs/msg/fiducial.hpp"
#include "fiducial_msgs/msg/fiducial_array.hpp"
#include "fiducial_msgs/msg/fiducial_transform.hpp"
#include "fiducial_msgs/msg/fiducial_transform_array.hpp"

#include "aruco_detect/math.hpp"
#include "aruco_detect/color.h"
// #include <dynamic_reconfigure/server.h>
// #include "aruco_detect/DetectorParamsConfig.h"

#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>

#include <list>
#include <string>
#include <boost/algorithm/string.hpp>
#include <boost/shared_ptr.hpp>
#include <array>
#include <algorithm>

using std::vector;

/// @brief Detect single ArUco markers
class FiducialsNode : public rclcpp::Node {
  private:
    rclcpp::Publisher<fiducial_msgs::msg::FiducialTransformArray>::SharedPtr pose_pub;

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr caminfo_sub;
    image_transport::Subscriber img_sub;
    image_transport::Publisher image_pub;
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster;

    // if set, we publish the images that contain fiducials
    bool publish_images = true;
    bool vis_msgs;
    bool verbose;

    double fiducial_len; // default marker size m

    bool haveCamInfo;
    vector <vector <cv::Point2f> > corners;
    std::vector<int> detected_ids_;
    cv_bridge::CvImagePtr cv_ptr;

    cv::Mat cameraMatrix_;
    cv::Mat distortionCoeffs_;
    std::string frameId;
    std::vector<int> ignoreIds;
    std::map<int, double> marker_size_dict_;   // specify marker size for certain marker ids

    // log spam prevention
    int prev_detected_count;

    cv::Ptr<cv::aruco::DetectorParameters> detectorParams;
    cv::Ptr<cv::aruco::Dictionary> dictionary;

    void handleIgnoreString(const std::string& str);

    void estimatePoseSingleMarkers(float markerLength,
                                   const cv::Mat &cameraMatrix,
                                   const cv::Mat &distCoeffs,
                                   vector<cv::Vec3d>& rvecs, vector<cv::Vec3d>& tvecs,
                                   vector<double>& reprojectionError);


    void ignoreCallback(const std_msgs::msg::String &msg);

    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
    
    void camInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    
    /*
    void configCallback(aruco_detect::DetectorParamsConfig &config, uint32_t level);
    dynamic_reconfigure::Server<aruco_detect::DetectorParamsConfig> configServer;
    dynamic_reconfigure::Server<aruco_detect::DetectorParamsConfig>::CallbackType callbackType;
    */


public:

  /// @brief Constructor of aruco_detect
  FiducialsNode() : Node("aruco_detector")
  {
    // declare ROS parameters
    this->declare_parameter("verbose", false);
    this->declare_parameter("ignored_markers", std::vector<int64_t>({}));
    this->declare_parameter("marker_size", 0.14);
    this->declare_parameter("camera_frame", "camera");
    this->declare_parameter("special_marker_size", vector<std::string>({""}));
    this->declare_parameter("camera_calibration_file", "");

    prev_detected_count = -1;

    haveCamInfo = true;

    verbose = this->get_parameter("verbose").as_bool();
    std::cout<<GREEN<<"Verbosity: "<<(verbose ? "True" : "False")<<RESET<<std::endl;

    fiducial_len = this->get_parameter("marker_size").as_double();
    std::cout<<GREEN<<"Default Marker size: "<<fiducial_len<<" m"<<RESET<<std::endl;

    vector<std::string> special_marker_size = this->get_parameter("special_marker_size").as_string_array();
    for (const std::string& special_size : special_marker_size){
      if (special_size == ""){
        continue;
      }
      vector<std::string> marker_size;
      // id_range:size, for example 1-40:0.04
      boost::split(marker_size, special_size, boost::is_any_of(":"));
      if (marker_size.size() == 2){
        double size = std::stod(marker_size[1]);
        // set marker id range
        vector<std::string> range;
        boost::split(range, marker_size[0], boost::is_any_of("-"));
        if (range.size() == 2){
          int start = std::stoi(range[0]);
          int end = std::stoi(range[1]);
          std::cout<<GREEN<<"Set the size of marker id range "<<start<<"~"<<end<<" to "<<size<<" m\n"<<RESET;
          for (uint i = start; i <= end; i++){
            marker_size_dict_[i] = size;
          }
        }
      }
      else{
        RCLCPP_ERROR(this->get_logger(), "Incorrect special_marker_size format!");
      }
    }

    frameId = this->get_parameter("camera_frame").as_string();
    std::cout<<GREEN<<"Camera frame: "<<frameId<<RESET<<std::endl;

    const std::vector<int64_t> ignored_markers = this->get_parameter("ignored_markers").as_integer_array();
    for (uint i = 0; i < ignored_markers.size(); i++){
      std::cout<<GREEN<<"Ignored markers: "<<ignored_markers[i]<<RESET<<std::endl;
      ignoreIds.emplace_back(ignored_markers[i]);
    }

    // Use camera intrisics
    std::string camera_calibration = this->get_parameter("camera_calibration_file").as_string();
    if (camera_calibration == ""){
      caminfo_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>("camera_info", 1, std::bind(&FiducialsNode::camInfoCallback, this, std::placeholders::_1) );
      std::cout<<YELLOW<<"NO camera calibration file given, use camera_info topic"<<RESET<<std::endl;
    }else{
      std::cout<<GREEN<<"Camera calibration file given: "<<camera_calibration<<RESET<<std::endl;
      cv::FileStorage fs(camera_calibration, cv::FileStorage::READ);
      cameraMatrix_ = fs["cam_mat"].mat();
      distortionCoeffs_ = fs["distort"].mat();
      std::cout<<"Camera matrix:\n"<<cameraMatrix_<<std::endl;
      std::cout<<"Distortion coefficients:\n"<<distortionCoeffs_<<std::endl;
    }


    broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // fetch image and detect aruco markers
    img_sub = image_transport::create_subscription(this, "/camera/image", std::bind(&FiducialsNode::imageCallback, this, std::placeholders::_1), "raw");
    // publish image drawn with detected markers 
    image_pub = image_transport::create_publisher(this, "/fiducial_images");    

    pose_pub = this->create_publisher<fiducial_msgs::msg::FiducialTransformArray>("fiducial_transforms", 1);

    // -------------------- ArUco configuration ------------------------
    int dicno = cv::aruco::DICT_4X4_100;  // dictionary index
    dictionary = cv::aruco::getPredefinedDictionary(dicno);
    detectorParams = new cv::aruco::DetectorParameters();
    // Marker detection refinement
    detectorParams->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;

    // callbackType = boost::bind(&FiducialsNode::configCallback, this, boost::placeholders::_1, boost::placeholders::_2);
    // configServer.setCallback(callbackType);
    // pnh.param<double>("adaptiveThreshConstant", detectorParams->adaptiveThreshConstant, 7);
    // pnh.param<int>("adaptiveThreshWinSizeMax", detectorParams->adaptiveThreshWinSizeMax, 53); /* defailt 23 */
    // pnh.param<int>("adaptiveThreshWinSizeMin", detectorParams->adaptiveThreshWinSizeMin, 3);
    // pnh.param<int>("adaptiveThreshWinSizeStep", detectorParams->adaptiveThreshWinSizeStep, 4); /* default 10 */
    // pnh.param<int>("cornerRefinementMaxIterations", detectorParams->cornerRefinementMaxIterations, 30);
    // pnh.param<double>("cornerRefinementMinAccuracy", detectorParams->cornerRefinementMinAccuracy, 0.01); /* default 0.1 */
    // pnh.param<int>("cornerRefinementWinSize", detectorParams->cornerRefinementWinSize, 5);
    // pnh.param<double>("errorCorrectionRate", detectorParams->errorCorrectionRate , 0.6);
    // pnh.param<double>("minCornerDistanceRate", detectorParams->minCornerDistanceRate , 0.05);
    // pnh.param<int>("markerBorderBits", detectorParams->markerBorderBits, 1);
    // pnh.param<double>("maxErroneousBitsInBorderRate", detectorParams->maxErroneousBitsInBorderRate, 0.04);
    // pnh.param<int>("minDistanceToBorder", detectorParams->minDistanceToBorder, 3);
    // pnh.param<double>("minMarkerDistanceRate", detectorParams->minMarkerDistanceRate, 0.05);
    // pnh.param<double>("minMarkerPerimeterRate", detectorParams->minMarkerPerimeterRate, 0.1); /* default 0.3 */
    // pnh.param<double>("maxMarkerPerimeterRate", detectorParams->maxMarkerPerimeterRate, 4.0);
    // pnh.param<double>("minOtsuStdDev", detectorParams->minOtsuStdDev, 5.0);
    // pnh.param<double>("perspectiveRemoveIgnoredMarginPerCell", detectorParams->perspectiveRemoveIgnoredMarginPerCell, 0.13);
    // pnh.param<int>("perspectiveRemovePixelPerCell", detectorParams->perspectiveRemovePixelPerCell, 8);
    // pnh.param<double>("polygonalApproxAccuracyRate", detectorParams->polygonalApproxAccuracyRate, 0.01); /* default 0.05 */

    RCLCPP_INFO(this->get_logger(), "%sAruco detection ready%s", BOLDGREEN, RESET);
  }
};


void FiducialsNode::estimatePoseSingleMarkers(float markerLength,
                                const cv::Mat &cameraMatrix,
                                const cv::Mat &distCoeffs,
                                vector<cv::Vec3d>& rvecs, vector<cv::Vec3d>& tvecs,
                                vector<double>& reprojectionError) {

  CV_Assert(markerLength > 0);

  vector<cv::Point3f> markerObjPoints;
  int nMarkers = (int)corners.size();
  rvecs.reserve(nMarkers);
  tvecs.reserve(nMarkers);
  reprojectionError.reserve(nMarkers);

  // for each marker, calculate its pose
  for (int i = 0; i < nMarkers; i++) {
    double fiducialSize = markerLength;

    std::map<int, double>::iterator it = marker_size_dict_.find(detected_ids_[i]);
    if (it != marker_size_dict_.end()) {
      fiducialSize = it->second;
    }
  
  getSingleMarkerObjectPoints(fiducialSize, markerObjPoints);
  // bool solved = cv::solvePnP(markerObjPoints, corners[i], cameraMatrix, distCoeffs, rvecs[i], tvecs[i], cv::SOLVEPNP_IPPE);
  bool solved = cv::solvePnP(markerObjPoints, corners[i], cameraMatrix, distCoeffs, rvecs[i], tvecs[i], false, cv::SOLVEPNP_IPPE_SQUARE);
  if (!solved){
    RCLCPP_ERROR(this->get_logger(), "%sFail to solvePnp%s", RED, RESET);
  }else{
    reprojectionError[i] = getReprojectionError(markerObjPoints, corners[i], cameraMatrix, distCoeffs, rvecs[i], tvecs[i]);
  }
  
  }
}


/// @brief detect markers in received images
/// @param msg 
void FiducialsNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  try {
      fiducial_msgs::msg::FiducialTransformArray trans_array;
      // cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
      if (msg->encoding == "rgb8")
        cv::cvtColor(cv_ptr->image, cv_ptr->image, cv::COLOR_RGB2BGR);

      cv::aruco::detectMarkers(cv_ptr->image, dictionary, corners, detected_ids_, detectorParams);

      int detected_count = (int)detected_ids_.size();
      if(verbose){
        prev_detected_count = detected_count;
        RCLCPP_INFO(this->get_logger(), "Detected %d markers", detected_count);
      }

      if(detected_ids_.size() > 0)
      {
        cv::aruco::drawDetectedMarkers(cv_ptr->image, corners, detected_ids_);

        vector <cv::Vec3d>  rvecs, tvecs;
        vector <double> reprojectionError;
        estimatePoseSingleMarkers(fiducial_len, cameraMatrix_, distortionCoeffs_, rvecs, tvecs, reprojectionError);

        for (uint i = 0; i < detected_ids_.size(); i++) {
        // filter ignored markers
          if (std::count(ignoreIds.begin(), ignoreIds.end(), detected_ids_[i]) != 0) {
            if(verbose){
              RCLCPP_INFO(this->get_logger(), "Ignoring marker id <%d>", detected_ids_[i]);                    
            }
            continue;
          }

          cv::aruco::drawAxis(cv_ptr->image, cameraMatrix_, distortionCoeffs_, rvecs[i], tvecs[i], 0.1);

          double angle = norm(rvecs[i]);
          cv::Vec3d axis = rvecs[i] / angle;
          tf2::Quaternion q;
          q.setRotation(tf2::Vector3(axis[0], axis[1], axis[2]), angle);
          {
            if (std::isnan(q.x()) || std::isnan(q.y()) || std::isnan(q.z())){
              continue;
            }
          }
          geometry_msgs::msg::TransformStamped ts;
          ts.transform.translation.x = tvecs[i][0];
          ts.transform.translation.y = tvecs[i][1];
          ts.transform.translation.z = tvecs[i][2];
          ts.transform.rotation.w = q.w();
          ts.transform.rotation.x = q.x();
          ts.transform.rotation.y = q.y();
          ts.transform.rotation.z = q.z();
          
          ts.header.frame_id = frameId;
          ts.header.stamp = msg->header.stamp;
          ts.child_frame_id = "marker_" + std::to_string(detected_ids_[i]); // camera_color_optical_frame
          broadcaster->sendTransform(ts);

          // publish FiducialTransformArray
          fiducial_msgs::msg::FiducialTransform ft;
          ft.fiducial_id = detected_ids_[i];
          ft.transform.translation.x = tvecs[i][0];
          ft.transform.translation.y = tvecs[i][1];
          ft.transform.translation.z = tvecs[i][2];
          ft.transform.rotation.w = q.w();
          ft.transform.rotation.x = q.x();
          ft.transform.rotation.y = q.y();
          ft.transform.rotation.z = q.z();
          ft.image_error = reprojectionError[i];
          ft.object_error = (reprojectionError[i] / dist(corners[i][0], corners[i][2])) * (norm(tvecs[i]) / fiducial_len);
          trans_array.transforms.push_back(ft);
        }
      }

      if (publish_images){
        cv_ptr->encoding = "bgr8";
        image_pub.publish(cv_ptr->toImageMsg());
      }
      
      trans_array.header.frame_id = frameId;
      trans_array.header.stamp = msg->header.stamp;
      pose_pub->publish(trans_array);
        
    }
    catch(cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
    catch(cv::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv exception: %s", e.what());
    }
    catch(std::exception &e){
      RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    }
}



int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<FiducialsNode>());

    return 0;
}



/// @brief Get camera intrinsics
/// @param msg 
void FiducialsNode::camInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  if (msg->k != std::array<double, 9>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0})) {
      // printf("Get camera info\n");
      for (int i=0; i<3; i++) {
          for (int j=0; j<3; j++) {
              cameraMatrix_.at<double>(i, j) = msg->k[i*3+j];
          }
      }

      for (int i=0; i<5; i++) {
          distortionCoeffs_.at<double>(0,i) = msg->d[i];
      }

      haveCamInfo = true;
      if (frameId == "" && verbose){
        std::cout<<"Camera frame not given, use frame in camera_info";
        frameId = msg->header.frame_id;
      }

  }
  else {
      std::cerr<<RED<<"CameraInfo message has invalid intrinsics, K matrix all zeros.\n"<<RESET;
  }
}
