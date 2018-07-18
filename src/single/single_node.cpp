#include "flea3/single_node.h"

namespace flea3 {

SingleNode::SingleNode(const ros::NodeHandle &pnh)
    : CameraNodeBase(pnh), flea3_ros_(pnh) {
  sub_mavros_ = pnh.subscribe("~cam_imu_stamp", 1, &SingleNode::MavrosCb);
}

void SingleNode::Acquire() {
  ROS_ERROR("Should not happend");
  while (is_acquire() && ros::ok()) {
    if (flea3_ros_.RequestSingle()) {
      const auto expose_duration =
          ros::Duration(flea3_ros_.camera().GetShutterTimeSec() / 2);
      const auto time = ros::Time::now() + expose_duration;
      flea3_ros_.PublishCamera(time);
      //      flea3_ros_.PublishImageMetadata(time);
      Sleep();
    }
  }
}

void SingleNode::MavrosCb(
    const mavros_msgs::CamIMUStampConstPtr &cam_imu_stamp) {
  const auto start_time = ros::Time::now().toSec();
  flea3_ros_.PublishCamera(cam_imu_stamp->frame_stamp);
  const auto duration = ros::Time::now().toSec() - start_time;
  ROS_INFO("Time publish: %f ms", duration * 1000);
}

void SingleNode::Setup(Config &config) {
  flea3_ros_.Stop();
  flea3_ros_.camera().Configure(config);
  flea3_ros_.set_fps(config.fps);
  flea3_ros_.Start();
}

} // namespace flea3
