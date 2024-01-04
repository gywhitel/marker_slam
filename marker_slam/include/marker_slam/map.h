/*
 * Copyright (c) 2017-9, Ubiquity Robotics
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
#ifndef MAP_H
#define MAP_H

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/buffer.h>

#include <list>
#include <string>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <std_srvs/srv/empty.hpp>

#include <fiducial_msgs/msg/fiducial_map_entry.hpp>
#include <fiducial_msgs/msg/fiducial_map_entry_array.hpp>
#include <marker_slam/transform_with_variance.h>
#include <marker_slam/helpers.h>

// An observation of a single fiducial in a single image. Record certain marker's pose in camera frame
class Observation {
  public:

    int fid;  // marker id
    tf2::Stamped<TransformWithVariance> T_fidCam; // camera pose in <id> marker frame
    tf2::Stamped<TransformWithVariance> T_camFid; // <id> marker pose in camera frame

    Observation(){};

    // Constructor for observation of camera pose with respect to <fid> marker
    Observation(int fid, const tf2::Stamped<TransformWithVariance> &camFid) {
      this->fid = fid;

      T_camFid = camFid;
      T_fidCam = T_camFid;
      T_fidCam.transform = T_camFid.transform.inverse(); 
    }
};

// A single fiducial marker that is in the map. Record marker status in the map.
// The fields variance and numObservations represent how good the pose estimate is considered to be, and how many observations were used to generate it.
class Fiducial {
  public:

    int id;   // marker id
    int numObs; // how many observations were used to generate it.
    bool visible;
    // a list of the ids of fiducials that have been observed at the same time as the current fiducial.
    std::set<int> links;  

    tf2::Stamped<TransformWithVariance> pose; // fid.pose is {marker <fid>} in {map origin}
    rclcpp::Time lastPublished;

    void update(const tf2::Stamped<TransformWithVariance> &newPose);

    Fiducial() {}

    // Create a fiducial from an estimate of its position in the map
    Fiducial(int id, const tf2::Stamped<TransformWithVariance> &pose) {
      this->id = id;
      this->pose = pose;
      this->lastPublished = rclcpp::Time(0);
      this->numObs = 0;
      this->visible = false;
    }
};

// Class containing map data
class Map{
  private:
    rclcpp::Node::SharedPtr node_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
    std::shared_ptr<tf2_ros::TransformListener> listener_;

    tf2::Transform T_camBase, T_baseCam;
    
    std::unique_ptr<EMA> pose_ema;
    bool ema;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr markerPub_;
    rclcpp::Publisher<fiducial_msgs::msg::FiducialMapEntryArray>::SharedPtr mapPub_;
    // rclcpp::Publisher<> robotPosePub;
    // rclcpp::Publisher cameraPosePub;
/*
    ros::ServiceServer clearSrv;
    ros::ServiceServer addSrv;
    bool clearCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool addFiducialCallback(fiducial_slam::AddFiducial::Request &req, fiducial_slam::AddFiducial::Response &res);
*/

    std::string mapFilename;
    std::string mapFrame;
    std::string odomFrame;
    std::string cameraFrame;  
    std::string baseFrame;  // robot base frame
    std::string estFrame;   // estimated robot base frame

    double future_date_transforms;
    bool publish_6dof_pose;
    double multiErrorThreshold;
    float tfPublishInterval;
    bool publishPoseTf;

    bool isInitializingMap;
    // if true, the map is not modified, and only localization is performed.
    bool readOnly; 
    int frameNum;
    int initialFrameNum;
    int originFid;  // marker id of map origin

    bool overridePublishedCovariance;
    std::vector<double> covarianceDiagonal;

    bool havePose;
    rclcpp::Time tfPublishTime;
    rclcpp::Clock::SharedPtr timer;
    geometry_msgs::msg::TransformStamped poseTf;   // {base_frame} TF

    // <id, marker pose> store markers in the map
    std::map<int, Fiducial> fiducials;  
    int fiducialToAdd;

    bool verbose;

  public:

    Map(const rclcpp::Node::SharedPtr &node);

    void update();
    void update(std::vector<Observation> &obs, const rclcpp::Time &time);
    
    void autoInit(const std::vector<Observation> &obs, const rclcpp::Time &time);
    
    int updatePose(std::vector<Observation> &obs, const rclcpp::Time &time,
                   tf2::Stamped<TransformWithVariance> &cameraPose);
    
    void updateMap(const std::vector<Observation> &obs, const rclcpp::Time &time,
                   const tf2::Stamped<TransformWithVariance> &cameraPose);
    
    void handleAddFiducial(const std::vector<Observation> &obs);

    bool loadMap();
    bool loadMap(std::string filename);
    
    bool saveMap();
    bool saveMap(std::string filename);

    void publishTf();
    
    void publishMap();
    
    void publishMarker(Fiducial &fid);
    void publishMarkers();
    
    void drawLine(const tf2::Vector3 &p0, const tf2::Vector3 &p1);

    bool lookupTransform(const std::string &from, const std::string &to, const rclcpp::Time &time,
                         tf2::Transform &T) const;
    bool lookupTransform(const std::string &from, const std::string &to, const tf2::TimePoint &time, tf2::Transform &T);

};

#endif
