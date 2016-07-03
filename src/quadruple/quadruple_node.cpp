#include "flea3/quadruple_node.h"

namespace flea3 {

QuadrupleNode::QuadrupleNode(const ros::NodeHandle& pnh)
    : CameraNodeBase(pnh), top_left_ros_(pnh, "top_left")   ,    top_right_ros_(pnh, "top_right"), 
                        bottom_left_ros_(pnh, "bottom_left"), bottom_right_ros_(pnh, "bottom_right") {}

void QuadrupleNode::Acquire() {
  while (is_acquire() && ros::ok()) {
    if (   top_left_ros_.RequestSingle() &&    top_right_ros_.RequestSingle() &&
        bottom_left_ros_.RequestSingle() && bottom_right_ros_.RequestSingle()) {
      const auto expose_duration =
          ros::Duration(top_left_ros_.camera().GetShutterTimeSec() / 2);
      const auto time = ros::Time::now() + expose_duration;
      top_left_ros_.PublishCamera(time);
      top_right_ros_.PublishCamera(time);
      bottom_left_ros_.PublishCamera(time);
      bottom_right_ros_.PublishCamera(time);
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
