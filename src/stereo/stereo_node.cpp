#include "flea3/stereo_node.h"

#include <ros/callback_queue.h>

namespace flea3 {

StereoNode::StereoNode(ros::NodeHandle& pnh)
    : CameraNodeBase(pnh), left_ros_(pnh, "left"), right_ros_(pnh, "right") {
}

void StereoNode::Acquire() {
  while (is_acquire() && ros::ok()) {
    if (right_ros_.RequestSingle() && left_ros_.RequestSingle()) {
      
      const auto right_image_msg = boost::make_shared<sensor_msgs::Image>();
      const auto  left_image_msg = boost::make_shared<sensor_msgs::Image>();
      
      left_ros_.Grab(left_image_msg, nullptr);      
      right_ros_.Grab(right_image_msg, nullptr);      

      right_ros_.Publish(right_image_msg);
      left_ros_.Publish(left_image_msg);

      Sleep();
    }
  }
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
