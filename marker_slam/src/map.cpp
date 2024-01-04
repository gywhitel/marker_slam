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

#include "marker_slam/map.h"
#include "marker_slam/color_print.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <string>

#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/string.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <boost/filesystem.hpp>


static double systematic_error = 0.01;


// Update a fiducial position in map with a new estimate
void Fiducial::update(const tf2::Stamped<TransformWithVariance> &newPose) {
    pose.update(newPose);
    numObs++;
}



// Constructor for map
Map::Map(const rclcpp::Node::SharedPtr& node) {
    
  node_ = node;
  std::cout<<"Get Node handle "<<node_<<std::endl;
  timer = node_->get_clock();

  broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
  tfBuffer_ = std::make_shared<tf2_ros::Buffer> (node_->get_clock());
  listener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
  
  node_->declare_parameter("map_frame", "map");
  node_->declare_parameter("base_frame", "base");
  node_->declare_parameter("camera_frame", "camera");
  node_->declare_parameter("estimate_frame", "slam_est");
  node_->declare_parameter("verbose", false);
  node_->declare_parameter("map_file", "");

    // nh.param<std::string>("map_frame", mapFrame, "map");
    // nh.param<std::string>("odom_frame", odomFrame, "odom");
    // nh.param<std::string>("base_frame", baseFrame, "base");
    // nh.param<std::string>("camera_frame", cameraFrame, "camera");
    // nh.param<std::string>("est_frame", estFrame, "SLAM_est");
    // nh.param<float>("tf_publish_interval", tfPublishInterval, 1.0);
    // nh.param<bool>("publish_tf", publishPoseTf, true);
    // nh.param<double>("systematic_error", systematic_error, 0.01);
    // nh.param<double>("future_date_transforms", future_date_transforms, 0.1);
    // nh.param<bool>("publish_6dof_pose", publish_6dof_pose, false);
    // nh.param<bool>("publish_6dof_pose", ema, true);
    // nh.param<bool>("read_only_map", readOnly, false);
    // nh.param<double>("multi_error_theshold", multiErrorThreshold, -1);
    // nh.param<std::string>("map_file", mapFilename, std::string(getenv("HOME")) + "/.ros/slam/map.txt");
    // std::string initialMap; // specify user-specified map file
    // nh.param<std::string>("initial_map_file", initialMap, "");

  mapFrame = node_->get_parameter("map_frame").as_string();
  baseFrame = node_->get_parameter("base_frame").as_string();
  cameraFrame = node_->get_parameter("camera_frame").as_string();
  estFrame = node_->get_parameter("estimate_frame").as_string();
  verbose = node_->get_parameter("verbose").as_bool();
  mapFilename = node_->get_parameter("map_file").as_string();

  if (mapFilename.empty()){
    mapFilename = "/home/yinghao/ros2_ws/src/fiducials_ros2/marker_slam/launch/map.txt";
  }
  std::cout<<GREEN<<"map frame: "<<mapFrame
              <<"\nrobot base frame: "<<baseFrame
              <<"\ncamera frame: "<<cameraFrame
              <<"\nestimated robot base frame: "<<estFrame
              <<"\nVerbose: "<< (verbose ? "True" :"False")
              <<"\nMap file: "<< mapFilename
              <<RESET<<std::endl;

  frameNum = 0;
  initialFrameNum = 0;
  originFid = -1;
  isInitializingMap = false;
  havePose = false;
  fiducialToAdd = -1;
  tfPublishInterval = 0.01;
  publishPoseTf = true;
  systematic_error = 0.01;
  publish_6dof_pose = false;
  ema = true;
  readOnly = false;
  multiErrorThreshold = -1;

  markerPub_ = node_->create_publisher<visualization_msgs::msg::Marker>("/fiducials", 10);
  mapPub_ = node_->create_publisher<fiducial_msgs::msg::FiducialMapEntryArray>("fiducial_map", 1);
  
    // robotPosePub = ros::Publisher(nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/fiducial_pose", 1));
    // cameraPosePub = ros::Publisher(nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/fiducial_slam/camera_pose", 1));
    // ros::Publisher(nh.advertise<fiducial_msgs::FiducialMapEntryArray>("/fiducial_map", 1));
    // clearSrv = nh.advertiseService("clear_map", &Map::clearCallback, this);
    // addSrv = nh.advertiseService("add_fiducial", &Map::addFiducialCallback, this);

    std::fill(covarianceDiagonal.begin(), covarianceDiagonal.end(), 0);
  /*
    overridePublishedCovariance = nh.getParam("covariance_diagonal", covarianceDiagonal);
    if (overridePublishedCovariance) {
    if (covarianceDiagonal.size() != 6) {
      RCLCPP_WARN("ignoring covariance_diagonal because it has %ld elements, not 6", covarianceDiagonal.size());
      overridePublishedCovariance = false;
    }
    // Check to make sure that the diagonal is non-zero
    for (auto variance : covarianceDiagonal) {
      if (variance == 0) {
        RCLCPP_WARN("ignoring covariance_diagonal because it has 0 values");
        std::fill(covarianceDiagonal.begin(), covarianceDiagonal.end(), 0);
        break;
      }
    }
    }
  */

    loadMap(mapFilename);
  
    // get fixed camera base transformation
     while (! lookupTransform(baseFrame, cameraFrame, timer->now(), T_baseCam)){ }
    std::cout<<GREEN<<"Get camera to robot base transformation ("
      <<T_baseCam.getOrigin().x()<<" "<<T_baseCam.getOrigin().y()<<" "<<T_baseCam.getOrigin().z()<<
      ")\n"<<RESET;

    while (! lookupTransform(cameraFrame, baseFrame, timer->now(), T_camBase)){ }
    std::cout<<GREEN<<"Get robot base to camera transformation T_camera_base("
      <<T_camBase.getOrigin().x()<<" "<<T_camBase.getOrigin().y()<<" "<<T_camBase.getOrigin().z()<<
      ")\n"<<RESET;
      
    // publishMarkers();

    pose_ema = std::make_unique<EMA>(0.1);  // initiate EMA

    std::cout<<GREEN<<"Map initiated.\n"<<RESET;
}

// Update map with a set of observations KEY callback in loop
void Map::update(std::vector<Observation> &obs, const rclcpp::Time &time) {

  frameNum++;

  // if observing markers and no markers are stored in the map file, this means the map is being initialized.
  if (obs.size() > 0 && fiducials.size() == 0) {
    isInitializingMap = true;
  }

  if (isInitializingMap) {
    // create map origin or relocate map origin
    autoInit(obs, time);  
  } 
  else {
    tf2::Stamped<TransformWithVariance> T_mapCam;
    T_mapCam.frame_id_ = mapFrame;

    // update robot base pose in the map (w.r.t map origin)
    if (updatePose(obs, time, T_mapCam) > 0 && obs.size() > 1 && !readOnly) {
      updateMap(obs, time, T_mapCam);
      // RCLCPP_INFO("Updating map with %d observations. Map has %d fiducials", (int)obs.size(), (int)fiducials.size());
      if (verbose)
        std::cout<<CYAN<<"[MAPPING] Update map frame {"<<mapFrame<<"} with "<<obs.size()<<". The map has "<<fiducials.size()<<" landmarks.\n"<<RESET;
    }
  }

//   handleAddFiducial(obs);

  publishMap();
}

// update estimates of observed fiducials from previously estimated camera pose
void Map::updateMap(const std::vector<Observation> &obs, const rclcpp::Time &time,
                    const tf2::Stamped<TransformWithVariance> &T_mapCam) {
    
    for (auto &map_pair : fiducials) {
        Fiducial &f = map_pair.second;
        f.visible = false;
    }

    for (const Observation &o : obs) {
        // This should take into account the variances from both
        tf2::Stamped<TransformWithVariance> T_mapFid = T_mapCam * o.T_camFid;
        T_mapFid.frame_id_ = mapFrame;

        // New scope for logging vars
        {
          tf2::Vector3 trans = T_mapFid.transform.getOrigin();

          // RCLCPP_INFO("Estimate of %d %lf %lf %lf var %lf %lf", o.fid, trans.x(), trans.y(),trans.z(), o.T_camFid.variance, T_mapFid.variance);

          if (std::isnan(trans.x()) || std::isnan(trans.y()) || std::isnan(trans.z())) {
              RCLCPP_WARN(node_->get_logger(), "Skipping NAN estimate\n");
              continue;
            };
        }

        // if marker is not found in the map, add the marker to the map as new landmark
        if (fiducials.find(o.fid) == fiducials.end()) {
            RCLCPP_INFO(node_->get_logger(), "%s[UPDATE MAP] New fiducial marker <%d>%s", BOLDGREEN, o.fid, RESET);
            fiducials[o.fid] = Fiducial(o.fid, T_mapFid);
        }

        Fiducial &f = fiducials[o.fid];
        f.visible = true;
        if (f.pose.variance != 0) {
            f.update(T_mapFid);
            f.numObs++;
        }

        for (const Observation &observation : obs) {
            int fid = observation.fid;
            if (f.id != fid) {
                f.links.insert(fid);
            }
        }
        publishMarker(fiducials[o.fid]);
    }
}

/// @brief Get {to} pose with respect to {from} frame
/// @param from target frame
/// @param to source frame
/// @param time 
/// @param T 
/// @return true if find the transformation.
bool Map::lookupTransform(const std::string &from, const std::string &to, const rclcpp::Time &time,
                          tf2::Transform &T) const {

    geometry_msgs::msg::TransformStamped transform;
    rclcpp::Duration timeout = rclcpp::Duration(15e6) ; 
    try {
      transform = tfBuffer_->lookupTransform(from, to, time, timeout);
      tf2::fromMsg(transform.transform, T);
      return true;
    } catch (tf2::TransformException &ex) {
      if (verbose){
        RCLCPP_WARN(node_->get_logger(), "Could not transform %s to %s: %s",from, to, ex.what());
      }
      return false;
    }
}

bool Map::lookupTransform(const std::string &from, const std::string &to, const tf2::TimePoint &time, tf2::Transform &T){
    rclcpp::Time timestamp = tf2_ros::toRclcpp(time);
		return lookupTransform(from, to, timestamp, T);
}

/// @brief Update pose estimate of robot. We combine the camera->base_link tf to each estimate 
/// so we can evaluate how good they are.  A good estimate would have z == roll == pitch == 0.
/// @param obs list of observed markers
/// @param time 
/// @param T_mapCam {camera} pose with respect to {map} frame
/// @return number of observations 
int Map::updatePose(std::vector<Observation> &obs, const rclcpp::Time &time,
                    tf2::Stamped<TransformWithVariance> &T_mapCam) {
    int numEsts = 0;
    tf2::Stamped<TransformWithVariance> T_mapBase;  // robot base in {map}

    if (obs.size() == 0) {
      return 0;
    }

    for (Observation &o : obs) {
      if (fiducials.find(o.fid) != fiducials.end()) {
        const Fiducial &fid = fiducials[o.fid];
        // T_map_cam camera pose in {map}. fid.pose is {marker <fid>} in {map origin}
        tf2::Stamped<TransformWithVariance> p = fid.pose * o.T_fidCam;

        p.frame_id_ = mapFrame;
        p.stamp_ = o.T_fidCam.stamp_;

        p.setData(p * T_camBase);   // compute {base} pose in {marker <fid>}
        auto position = p.transform.getOrigin();  // get translation vector
        double roll, pitch, yaw;
        p.transform.getBasis().getRPY(roll, pitch, yaw);  // get rotation matrix

        // Create variance according to how well the robot is upright on the ground
        // TODO: Create variance for each DOF
        // TODO: Take into account position according to odom
        auto cam_f = o.T_camFid.transform.getOrigin();
        double s1 = std::pow(position.z() / cam_f.z(), 2) *
                    (std::pow(cam_f.x(), 2) + std::pow(cam_f.y(), 2));
        double s2 = position.length2() * std::pow(std::sin(roll), 2);
        double s3 = position.length2() * std::pow(std::sin(pitch), 2);
        p.variance = s1 + s2 + s3 + systematic_error;
        o.T_camFid.variance = p.variance;

        // RCLCPP_INFO("Marker <%d>, Camera pose (%lf %lf %lf) (%lf %lf %lf) %lf", o.fid, position.x(), position.y(), position.z(), roll, pitch, yaw, p.variance);

        // drawLine(fid.pose.transform.getOrigin(), o.position);

        if (std::isnan(position.x()) || std::isnan(position.y()) || std::isnan(position.z())) {
            RCLCPP_WARN(node_->get_logger(), "Skipping NAN estimate\n");
            continue;
        };

        // compute base_link pose based on this estimate
        if (numEsts == 0) {
          // only a single marker is observed
            T_mapBase = p;
        } else {
          // multiple markers are observed. Average multiple observation.
            T_mapBase.setData(averageTransforms(T_mapBase, p));
            T_mapBase.stamp_ = p.stamp_;
        }
        numEsts++;
      }
    }

    if (numEsts == 0) {
        RCLCPP_INFO(node_->get_logger(), "Finished frame - no estimates\n");
        return numEsts;
    }

    // New scope for logging vars
    {
        tf2::Vector3 trans = T_mapBase.transform.getOrigin();
        double r, p, y;
        T_mapBase.transform.getBasis().getRPY(r, p, y);

        // RCLCPP_INFO("Pose ALL (%lf %lf %lf) (%lf %lf %lf) %f", trans.x(), trans.y(), trans.z(), r, p, y, T_mapBase.variance);
    }

    tf2::Stamped<TransformWithVariance> basePose = T_mapBase;
    basePose.frame_id_ = mapFrame;
    /*
    auto robotPose = toPose(basePose);

    if (overridePublishedCovariance) {
        for (int i = 0; i <= 5; i++) {
            robotPose.pose.covariance[i * 6 + i] = covarianceDiagonal[i];  // Fill the diagonal
        }
    }
    robotPosePub.publish(robotPose);  // publish estimated robot base pose
    */

    T_mapCam = T_mapBase * T_baseCam;

    tf2::Stamped<TransformWithVariance> outPose = basePose;
    outPose.frame_id_ = mapFrame;
    
    if (!odomFrame.empty()) {
        tf2::Transform odomTransform;
        if (lookupTransform(odomFrame, baseFrame, tf2_ros::toRclcpp(outPose.stamp_), odomTransform)) {
            outPose.setData(basePose * odomTransform.inverse());
            estFrame = odomFrame;

            tf2::Vector3 c = odomTransform.getOrigin();
            RCLCPP_INFO(node_->get_logger(), "odom   %lf %lf %lf", c.x(), c.y(), c.z());
        }
        else {
            // Don't publish anything if map->odom was requested and is unavailaable
            return numEsts;
        }
    }

    // Make outgoing transform make sense - ie only consist of x, y, yaw
    // This can be disabled via the publish_6dof_pose param, mainly for debugging
    if (!publish_6dof_pose) {
        tf2::Vector3 translation = outPose.transform.getOrigin();
        translation.setZ(0);
        outPose.transform.setOrigin(translation);
        double roll, pitch, yaw;
        outPose.transform.getBasis().getRPY(roll, pitch, yaw);
        outPose.transform.getBasis().setRPY(0, 0, yaw);
    }


    // TODO fuse historical data according to variance
    // Exponential Moving Average filter on pose
    outPose.transform = pose_ema->update(outPose.transform);

    poseTf = toMsg(outPose);
    poseTf.child_frame_id = estFrame;
    havePose = true;

    if (publishPoseTf) {
      publishTf();
    }

    // std::cout<<BLUE<<"[LOCATION] Update {"<<estFrame<<"} pose estimation with "<<numEsts<<" estimates.\n"<<RESET;
    // RCLCPP_INFO("Finished {%s} pose estimation update. Estimates %d\n", estFrame, numEsts);
    return numEsts;
}


// Publish estimated robot base tf ({base_frame}) w.r.t {map}
void Map::publishTf() {
    rclcpp::Time now = node_->get_clock()->now();
    tfPublishTime = now;
    poseTf.header.stamp = now;
    broadcaster_->sendTransform(poseTf);
}

// publish latest tf if enough time has elapsed
void Map::update() {
    rclcpp::Time now = node_->get_clock()->now();
    if (publishPoseTf && havePose && tfPublishInterval != 0.0 &&
        (now - tfPublishTime).seconds() > tfPublishInterval) {
        publishTf();
        tfPublishTime = now;
    }
    publishMarkers();
}

// Find closest fiducial to camera
static int findClosestObs(const std::vector<Observation> &obs) {
    double smallestDist = -1;
    int closestIdx = -1;

    for (size_t i = 0; i < obs.size(); i++) {
        const Observation &o = obs[i];
        double d = o.T_camFid.transform.getOrigin().length2();
        if (smallestDist < 0 || d < smallestDist) {
            smallestDist = d;
            closestIdx = i;
        }
    }

    return closestIdx;
}

// Initialize a map from the closest observed fiducial. The cloest marker is selected as map origin. 
// Figure out the closest marker, and then figure out the
// pose of that marker such that base_link is at the origin of the map frame
void Map::autoInit(const std::vector<Observation> &obs, const rclcpp::Time &time) {
    RCLCPP_INFO(node_->get_logger(), "Auto init map frame [%d/10]", frameNum);

    tf2::Transform T_baseCam;

    // map haven't been built, create map origin marker
    if (fiducials.size() == 0) {  
        // find the closest marker as map origin
      int idx = findClosestObs(obs);

      if (idx == -1) {
	     std::cerr<<BOLDYELLOW<<"Could not find a fiducial to initialize map from\n"<<RESET;
	     return;
      }
      const Observation &o = obs[idx];
      originFid = o.fid;  // origin marker id 

      printf("%sInitializing map from fiducial <%d>%s", BOLDGREEN, o.fid, RESET);

      tf2::Stamped<TransformWithVariance> T = o.T_camFid; // marker pose in {camera}

      // get transformation from {camera} to {base}
      if (lookupTransform(baseFrame, o.T_camFid.frame_id_, o.T_camFid.stamp_, T_baseCam)) {
        T.setData(T_baseCam * T);
      }
      // add origin marker <o.fid> to the map
      fiducials[o.fid] = Fiducial(o.fid, T);
    } 
    else {  // The map contains at least 1 marker. Refresh the origin location.
      for (const Observation &o : obs) {
        if (o.fid == originFid) {
          tf2::Stamped<TransformWithVariance> T = o.T_camFid;

          tf2::Vector3 trans = T.transform.getOrigin();
          RCLCPP_INFO(node_->get_logger(), "Estimate marker <%d> pose from map (%lf %lf %lf) with err %lf", o.fid, trans.x(),
                    trans.y(), trans.z(), o.T_camFid.variance);

          if (lookupTransform(baseFrame, o.T_camFid.frame_id_, o.T_camFid.stamp_, T_baseCam)) {
              T.setData(T_baseCam * T);
          }

          fiducials[originFid].update(T);
          break;
        }
      }
    }

    if (frameNum - initialFrameNum > 10 && originFid != -1) {
        isInitializingMap = false;

        fiducials[originFid].pose.variance = 0.0;
    }
}

// Attempt to add the specified fiducial to the map
void Map::handleAddFiducial(const std::vector<Observation> &obs) {

    if (fiducialToAdd == -1) {
        return;
    }

    if (fiducials.find(fiducialToAdd) != fiducials.end()) {
        RCLCPP_INFO(node_->get_logger(), "Fiducial %d is already in map - ignoring add request",
                 fiducialToAdd);
        fiducialToAdd = -1;
        return;
    }

    for (const Observation &o : obs) {
        if (o.fid == fiducialToAdd) {
            RCLCPP_INFO(node_->get_logger(), "Adding fiducial_id %d to map", fiducialToAdd);

            tf2::Stamped<TransformWithVariance> T = o.T_camFid;
						T.setData(T_baseCam * T);

            // Take into account position of robot in the world if known
            tf2::Transform T_mapBase;
            if (lookupTransform(mapFrame, baseFrame, rclcpp::Time(0), T_mapBase)) {
                T.setData(T_mapBase * T);
            }
            else {
                RCLCPP_INFO(node_->get_logger(), "Placing robot at the origin");
            }

            fiducials[o.fid] = Fiducial(o.fid, T);
            fiducials[originFid].pose.variance = 0.0;
            isInitializingMap = false;

            fiducialToAdd = -1;
            return;
        }
    }

    RCLCPP_INFO(node_->get_logger(), "Unable to add fiducial %d to map", fiducialToAdd);
}

// save map to file

bool Map::saveMap() { return saveMap(mapFilename); }

bool Map::saveMap(std::string filename) {
    RCLCPP_INFO(node_->get_logger(), "Saving map with %d fiducials to file %s\n", (int)fiducials.size(), filename.c_str());

    FILE *fp = fopen(filename.c_str(), "w");
    if (fp == NULL) {
        RCLCPP_WARN(node_->get_logger(), "Could not open %s for write\n", filename.c_str());
        return false;
    }

    for (auto &map_pair : fiducials) {
        Fiducial &f = map_pair.second;
        tf2::Vector3 trans = f.pose.transform.getOrigin();
        double rx, ry, rz;
        f.pose.transform.getBasis().getRPY(rx, ry, rz);

        fprintf(fp, "%d %lf %lf %lf %lf %lf %lf %lf %d", f.id, trans.x(), trans.y(), trans.z(),
                rad2deg(rx), rad2deg(ry), rad2deg(rz), f.pose.variance, f.numObs);

        for (const auto linked_fid : f.links) {
            fprintf(fp, " %d", linked_fid);
        }
        fprintf(fp, "\n");
    }
    fclose(fp);
    return true;
}

// Load map from predefined file
// bool Map::loadMap() { return loadMap(mapFilename); }

/// @brief Load map from user defined map
/// @param filename map file path
/// @return true if load map successfully, and false if failed.
bool Map::loadMap(std::string filename) {
    int numRead = 0;

    RCLCPP_INFO(node_->get_logger(), "Load map %s", filename.c_str());

    FILE *fp = fopen(filename.c_str(), "r");
    if (fp == NULL) {
        RCLCPP_WARN(node_->get_logger(), "Could not open %s for read\n", filename.c_str());
        return false;
    }

    const int BUFSIZE = 2048;
    char linebuf[BUFSIZE];
    char linkbuf[BUFSIZE];

    while (!feof(fp)) {
      if (fgets(linebuf, BUFSIZE - 1, fp) == NULL) break;

      int id;
      double tx, ty, tz, rx, ry, rz, var;
      int numObs = 0;

      linkbuf[0] = '\0';
      int nElems = sscanf(linebuf, "%d %lf %lf %lf %lf %lf %lf %lf %d%[^\t\n]*s", &id, &tx, &ty,
                          &tz, &rx, &ry, &rz, &var, &numObs, linkbuf);
      if (nElems == 9 || nElems == 10) {
          tf2::Vector3 tvec(tx, ty, tz);
          tf2::Quaternion q;
          q.setRPY(deg2rad(rx), deg2rad(ry), deg2rad(rz));

          auto twv = TransformWithVariance(tvec, q, var);
          tf2::TimePoint tf_timestamp = tf2_ros::fromRclcpp(node_->get_clock()->now());
          Fiducial f = Fiducial(id, tf2::Stamped<TransformWithVariance>(twv, tf_timestamp, mapFrame));
          f.numObs = numObs;

          std::istringstream ss(linkbuf);
          std::string s;
          while (getline(ss, s, ' ')) {
              if (!s.empty()) {
                  f.links.insert(stoi(s));
              }
          }
          fiducials[id] = f;
          numRead++;
      } else {
          RCLCPP_WARN(node_->get_logger(), "Invalid line: %s", linebuf);
      }
    }

    fclose(fp);
    RCLCPP_INFO(node_->get_logger(), "Load map %s read %d entries", filename.c_str(), numRead);
    return true;
}

// Publish the map
void Map::publishMap() {
    fiducial_msgs::msg::FiducialMapEntryArray fmea;
    std::map<int, Fiducial>::iterator it;

    for (const auto &map_pair : fiducials) {
        const Fiducial &f = map_pair.second;

        fiducial_msgs::msg::FiducialMapEntry fme;
        fme.fiducial_id = f.id;

        tf2::Vector3 t = f.pose.transform.getOrigin();
        fme.x = t.x();
        fme.y = t.y();
        fme.z = t.z();

        double rx, ry, rz;
        f.pose.transform.getBasis().getRPY(rx, ry, rz);
        fme.rx = rx;
        fme.ry = ry;
        fme.rz = rz;

        fmea.fiducials.push_back(fme);
    }

    mapPub_->publish(fmea);
}

// Publish the next marker visualization messages that hasn't been published recently
void Map::publishMarkers() {
    rclcpp::Time now = node_->get_clock()->now();
    std::map<int, Fiducial>::iterator it;

    for (auto &map_pair : fiducials) {
        Fiducial &f = map_pair.second;
        if ((now - f.lastPublished).seconds() > 1.0) {
            publishMarker(f);
        }
    }
}

// Publish visualization messages for a single fiducial
void Map::publishMarker(Fiducial &fid) {
    fid.lastPublished = node_->get_clock()->now();

    // Flattened cube
    visualization_msgs::msg::Marker marker;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    toMsg(fid.pose.transform, marker.pose);

    marker.scale.x = 0.15;
    marker.scale.y = 0.15;
    marker.scale.z = 0.01;
    if (fid.visible) {
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;
    } else {
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;
    }
    marker.id = fid.id;
    marker.ns = "fiducial";
    marker.header.frame_id = mapFrame;
    markerPub_->publish(marker);

    // cylinder scaled by stddev
    visualization_msgs::msg::Marker cylinder;
    cylinder.type = visualization_msgs::msg::Marker::CYLINDER;
    cylinder.action = visualization_msgs::msg::Marker::ADD;
    cylinder.header.frame_id = mapFrame;
    cylinder.color.r = 0.0f;
    cylinder.color.g = 0.0f;
    cylinder.color.b = 1.0f;
    cylinder.color.a = 0.5f;
    cylinder.id = fid.id + 10000;
    cylinder.ns = "sigma";
    cylinder.scale.x = cylinder.scale.y = std::max(std::sqrt(fid.pose.variance), 0.1);
    cylinder.scale.z = 0.01;
    cylinder.pose.position.x = marker.pose.position.x;
    cylinder.pose.position.y = marker.pose.position.y;
    cylinder.pose.position.z = marker.pose.position.z;
    cylinder.pose.position.z += (marker.scale.z / 2.0) + 0.05;
    markerPub_->publish(cylinder);

    // Text
    visualization_msgs::msg::Marker text;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;
    text.header.frame_id = mapFrame;
    text.color.r = text.color.g = text.color.b = text.color.a = 1.0f;
    text.id = fid.id;
    text.scale.x = text.scale.y = text.scale.z = 0.1;
    text.pose.position.x = marker.pose.position.x;
    text.pose.position.y = marker.pose.position.y;
    text.pose.position.z = marker.pose.position.z;
    text.pose.position.z += (marker.scale.z / 2.0) + 0.1;
    text.id = fid.id + 30000;
    text.ns = "text";
    text.text = std::to_string(fid.id);
    markerPub_->publish(text);

    // Links
    visualization_msgs::msg::Marker links;
    links.type = visualization_msgs::msg::Marker::LINE_LIST;
    links.action = visualization_msgs::msg::Marker::ADD;
    links.header.frame_id = mapFrame;
    links.color.r = 0.0f;
    links.color.g = 0.0f;
    links.color.b = 1.0f;
    links.color.a = 1.0f;
    links.id = fid.id + 40000;
    links.ns = "links";
    links.scale.x = links.scale.y = links.scale.z = 0.02;
    links.pose.position.x = 0;
    links.pose.position.y = 0;
    links.pose.position.z = 0;

    geometry_msgs::msg::Point gp0, gp1;
    tf2::Vector3 p0 = fid.pose.transform.getOrigin();
    gp0.x = p0.x();
    gp0.y = p0.y();
    gp0.z = p0.z();

    std::map<int, int>::iterator lit;
    for (const auto linked_fid : fid.links) {
      // only draw links in one direction
      if (fid.id < linked_fid) {
        if (fiducials.find(linked_fid) != fiducials.end()) {
          tf2::Vector3 p1 = fiducials[linked_fid].pose.transform.getOrigin();
          gp1.x = p1.x();
          gp1.y = p1.y();
          gp1.z = p1.z();
          links.points.push_back(gp0);
          links.points.push_back(gp1);
        }
      }
    }

    markerPub_->publish(links);
}

// Service to clear the map and enable auto initialization
/*
bool Map::clearCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    RCLCPP_INFO("Clearing fiducial map from service call");

    fiducials.clear();
    initialFrameNum = frameNum;
    originFid = -1;

    return true;
}

// Service to add a fiducial to the map

bool Map::addFiducialCallback(fiducial_slam::AddFiducial::Request &req,
                              fiducial_slam::AddFiducial::Response &res)
{
   RCLCPP_INFO("Request to add fiducial %d to map", req.fiducial_id);
   fiducialToAdd = req.fiducial_id;

   return true;
}
*/
