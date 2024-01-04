/*
 * Copyright (c) 2017-20, Ubiquity Robotics
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

#include <marker_slam/helpers.h>
#include <marker_slam/map.h>

#include <assert.h>
#include <signal.h>
#include <sys/time.h>
#include <unistd.h>
#include <list>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer_interface.h>

#include "fiducial_msgs/msg/fiducial.hpp"
#include "fiducial_msgs/msg/fiducial_array.hpp"
#include "fiducial_msgs/msg/fiducial_transform.hpp"
#include "fiducial_msgs/msg/fiducial_transform_array.hpp"
#include "marker_slam/color_print.h"


using std::vector;
// using namespace cv;

class FiducialSlam : public rclcpp::Node {
  private:
    rclcpp::Subscription<fiducial_msgs::msg::FiducialTransformArray>::SharedPtr ft_sub;

    bool use_fiducial_area_as_weight = false;
    double weighting_scale = 1e9;
    std::vector<int64_t> landmarks_;

    void transformCallback(const fiducial_msgs::msg::FiducialTransformArray::ConstSharedPtr msg);


  public:
    std::shared_ptr<Map> markerMap;  // marker map
    int pose_publish_rate = 30;

    FiducialSlam();

    void init_map();

};

void FiducialSlam::transformCallback(const fiducial_msgs::msg::FiducialTransformArray::ConstSharedPtr msg) {
    // convert std_msgs::Header::stamp -> ROS time
    tf2::TimePoint timestamp = tf2_ros::fromMsg(msg->header.stamp);
    vector<Observation> observations;

    for (size_t i = 0; i < msg->transforms.size(); i++) {
      const fiducial_msgs::msg::FiducialTransform &ft = msg->transforms[i];
      // Only use marker ID after 50 for SLAM
      if (ft.fiducial_id < 50){
        continue;
      }
      // skip markers not specified in landmark ID
      if (std::count(landmarks_.begin(), landmarks_.end(), ft.fiducial_id) == 0){
        continue;
      }

      double variance;
      if (use_fiducial_area_as_weight) {
        variance = weighting_scale / ft.fiducial_area;
      } else {
        variance = weighting_scale * ft.object_error;
      }
      // observed marker info in {camera} frame
     
      Observation obs(ft.fiducial_id, tf2::Stamped<TransformWithVariance>(TransformWithVariance(ft.transform, variance), timestamp, msg->header.frame_id));
      observations.push_back(obs);
    }
    
    markerMap->update(observations, tf2_ros::toRclcpp(timestamp));
}

FiducialSlam::FiducialSlam() : Node("marker_SLAM") {
  this->declare_parameter("landmark", std::vector<int64_t>({}));
  landmarks_ = this->get_parameter("landmark").as_integer_array();
  std::cout<<"Map landmarks:";
  for (uint i = 0; i < landmarks_.size(); i++){
    std::cout<<landmarks_[i]<<", ";
  }
  std::cout<<std::endl;

  ft_sub = this->create_subscription<fiducial_msgs::msg::FiducialTransformArray>("/fiducial_transforms", 1, std::bind(&FiducialSlam::transformCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Fiducial SLAM ready");
}

void FiducialSlam::init_map(){
try{
    markerMap = std::make_shared<Map>(this->shared_from_this());
  }
  catch(const std::exception& e){
    std::cerr<<RED<<e.what()<<RESET<<std::endl;
  }
  std::cout<<GREEN<<"Map initiated successfully.\n"<<std::endl;
}

auto node = std::shared_ptr<FiducialSlam>(nullptr);

// must be global? Cannot be a member function
void exitHandler(int sig){
  if (node != nullptr){
    node->markerMap->saveMap();
    RCLCPP_INFO(node->get_logger(), "%sMap file saved%s", GREEN, RESET);
  }
}


int main(int argc, char **argv) {

  rclcpp::init(argc, argv);

  node = std::make_shared<FiducialSlam>();
  node->init_map();

  signal(SIGINT, exitHandler);    

  rclcpp::spin(node);


  return 0;
}
