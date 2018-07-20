#include "flea3/single_node.h"
#include <mavros_msgs/CamIMUStamp.h> 

namespace flea3 {

SingleNode::SingleNode(ros::NodeHandle &pnh)
    : CameraNodeBase(pnh), flea3_ros_(pnh) {
}

void SingleNode::Acquire() {
  
  while (is_acquire() && ros::ok()) {

    // printf("In %s:%d\n", __FUNCTION__, __LINE__);

    // ### 
    // In external trigger mode, this request function does not have any effect.
    // Hence using the time at which this function is called is not the right approach for
    // the multi-cam synch. purpose.
    if (flea3_ros_.RequestSingle()) {

      // const auto expose_duration = flea3_ros_.camera().GetShutterTimeSec();
      const auto image_msg = boost::make_shared<sensor_msgs::Image>();

      // const auto time00 = GetClosestTriggerStamp(ros::Time::now());
      // const auto time01 = GetClosestTriggerStamp(ros::Time::now());

      flea3_ros_.Grab(image_msg, nullptr);

      /*
      ros::Time time = ros::Time::now();
      if(flea3_ros_.camera().UseExtTriggerStamp() == true)
        time = GetClosestTriggerStamp(time, - (expose_duration + 7) / 1.0e+3);

      image_msg->header.frame_id = flea3_ros_.frame_id();
      image_msg->header.stamp = time;
      */
      // flea3_ros_.PublishCamera(time);

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

void ExtTriggerCb(const mavros_msgs::CamIMUStampConstPtr &cam_imu_stamp) {

  }

} // namespace flea3
