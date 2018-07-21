#include "flea3/single_node.h"
#include <mavros_msgs/CamIMUStamp.h> 

namespace flea3 {

SingleNode::SingleNode(ros::NodeHandle &pnh)
    : CameraNodeBase(pnh), flea3_ros_(pnh) {
}

void SingleNode::Acquire() {
  
  while (is_acquire() && ros::ok()) {
    // In external trigger mode, this request function does not have any effect.
    // Hence using the time at which this function is called is not the right approach for
    // the multi-cam synch. purpose.
    if (flea3_ros_.RequestSingle()) {

      const auto image_msg = boost::make_shared<sensor_msgs::Image>();
      flea3_ros_.Grab(image_msg, nullptr);
      flea3_ros_.Publish(image_msg);
      Sleep();
    }
  }
}

void SingleNode::Setup(Config &config) {
  flea3_ros_.Stop();
  flea3_ros_.camera().Configure(config);
  flea3_ros_.set_fps(config.fps);
  flea3_ros_.Start();
}

} // namespace flea3
