#include "flea3/quadruple_node.h"

namespace flea3 {

QuadrupleNode::QuadrupleNode(ros::NodeHandle& pnh)
    : CameraNodeBase(pnh),   top_right_ros_(pnh, "top_right")  , bottom_right_ros_(pnh, "bottom_right"),
                           bottom_left_ros_(pnh, "bottom_left"),     top_left_ros_(pnh, "top_left") {
    }

void QuadrupleNode::Acquire() {
  
  while (is_acquire() && ros::ok()) {
    if (  top_right_ros_.RequestSingle() && bottom_right_ros_.RequestSingle() &&
        bottom_left_ros_.RequestSingle() &&     top_left_ros_.RequestSingle()) {
      
      const auto  top_right_image_msg    = boost::make_shared<sensor_msgs::Image>();
      const auto  bottom_right_image_msg = boost::make_shared<sensor_msgs::Image>();
      const auto  bottom_left_image_msg  = boost::make_shared<sensor_msgs::Image>();
      const auto  top_left_image_msg     = boost::make_shared<sensor_msgs::Image>();
      
      top_right_ros_.Grab(top_right_image_msg, nullptr);      
      bottom_right_ros_.Grab(bottom_right_image_msg);
      bottom_left_ros_.Grab(bottom_left_image_msg);
      top_left_ros_.Grab(top_left_image_msg);

      top_right_ros_.Publish(top_right_image_msg);
      bottom_right_ros_.Publish(bottom_right_image_msg);
      bottom_left_ros_.Publish(bottom_left_image_msg);
      top_left_ros_.Publish(top_left_image_msg);
      Sleep();
    }
  }
}

void QuadrupleNode::Setup(Flea3DynConfig& config) {
  top_left_ros_.Stop();
  top_right_ros_.Stop();
  bottom_left_ros_.Stop();
  bottom_right_ros_.Stop();
  top_left_ros_.set_fps(config.fps);
  top_right_ros_.set_fps(config.fps);
  bottom_left_ros_.set_fps(config.fps);
  bottom_right_ros_.set_fps(config.fps);
  Flea3DynConfig config_cpy = config;
  top_left_ros_.camera().Configure(config_cpy);
  top_right_ros_.camera().Configure(config);
  bottom_left_ros_.camera().Configure(config_cpy);
  bottom_right_ros_.camera().Configure(config);
  top_left_ros_.Start();
  top_right_ros_.Start();
  bottom_left_ros_.Start();
  bottom_right_ros_.Start();

}

}  // namespace flea3
