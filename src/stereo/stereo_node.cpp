#include "flea3/stereo_node.h"

namespace flea3 {

StereoNode::StereoNode(ros::NodeHandle& pnh)
    : CameraNodeBase(pnh), left_ros_(pnh, "left"), right_ros_(pnh, "right") {
    sub_mavros_ = pnh.subscribe("cam_imu_stamp", 10, &StereoNode::MavrosCb, this);
}

void StereoNode::Acquire() {
  ROS_ERROR("Should not happend");
  while (is_acquire() && ros::ok()) {
    if (left_ros_.RequestSingle() && right_ros_.RequestSingle()) {
      const auto expose_duration =
          ros::Duration(left_ros_.camera().GetShutterTimeSec() / 2);
      const auto time = ros::Time::now() + expose_duration;
      left_ros_.PublishCamera(time);
      right_ros_.PublishCamera(time);
      Sleep();
    }
  }
}  

void StereoNode::MavrosCb(
    const mavros_msgs::CamIMUStampConstPtr &cam_imu_stamp) {
    const auto start_time = ros::Time::now().toSec();
    left_ros_.PublishCamera(cam_imu_stamp->frame_stamp);
    right_ros_.PublishCamera(cam_imu_stamp->frame_stamp);
    const auto duration = ros::Time::now().toSec() - start_time;
    ROS_INFO("Time publish: %f ms", duration * 1000);
}

void StereoNode::Setup(Flea3DynConfig& config) {
  left_ros_.Stop();
  right_ros_.Stop();
  left_ros_.set_fps(config.fps);
  right_ros_.set_fps(config.fps);
  Flea3DynConfig config_cpy = config;
  left_ros_.camera().Configure(config_cpy);
  right_ros_.camera().Configure(config);
  left_ros_.Start();
  right_ros_.Start();
}

}  // namespace flea3
