#include "flea3/triple_node.h"

#include <ros/callback_queue.h>

namespace flea3 {

TripleNode::TripleNode(ros::NodeHandle& pnh)
    : CameraNodeBase(pnh), right_ros_(pnh, "right"), middle_ros_(pnh, "middle"), left_ros_(pnh, "left") {
}

void TripleNode::Acquire() {
  while (is_acquire() && ros::ok()) {
    if (right_ros_.RequestSingle() && middle_ros_.RequestSingle() && left_ros_.RequestSingle()) {
      
      const auto  right_image_msg = boost::make_shared<sensor_msgs::Image>();
      const auto middle_image_msg = boost::make_shared<sensor_msgs::Image>();
      const auto   left_image_msg = boost::make_shared<sensor_msgs::Image>();
      
      right_ros_.Grab(right_image_msg, nullptr);      
      middle_ros_.Grab(middle_image_msg, nullptr);      
      left_ros_.Grab(left_image_msg, nullptr);      
      
      right_ros_.Publish(right_image_msg);
      middle_ros_.Publish(middle_image_msg);
      left_ros_.Publish(left_image_msg);

      Sleep();
    }
  }
}  

void TripleNode::Setup(Flea3DynConfig& config) {
  right_ros_.Stop();
  middle_ros_.Stop();
  left_ros_.Stop();
  right_ros_.set_fps(config.fps);
  middle_ros_.set_fps(config.fps);
  left_ros_.set_fps(config.fps);
  Flea3DynConfig config_cpy0 = config;
  Flea3DynConfig config_cpy1 = config;
  right_ros_.camera().Configure(config_cpy0);
  middle_ros_.camera().Configure(config_cpy1);
  left_ros_.camera().Configure(config);
  right_ros_.Start();
  middle_ros_.Start();
  left_ros_.Start();
}

}  // namespace flea3
