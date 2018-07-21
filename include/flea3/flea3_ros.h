#ifndef FLEA3_FLEA3_ROS_H_
#define FLEA3_FLEA3_ROS_H_

#include <camera_base/camera_ros_base.h>
#include "flea3/flea3_camera.h"
#include <cv_bridge/cv_bridge.h>

// ###->
#include <mutex>
#include <deque>
#include <mavros_msgs/CamIMUStamp.h> 
// ###-<

namespace flea3 {

  class Flea3Ros : public camera_base::CameraRosBase {
  public:
    explicit Flea3Ros(const ros::NodeHandle& pnh,
      const std::string& prefix = std::string());

    Flea3Camera& camera();

    bool RequestSingle();

    bool Grab(const sensor_msgs::ImagePtr& image_msg,
      const sensor_msgs::CameraInfoPtr& cinfo_msg = nullptr) override;
    void PublishImageMetadata(const ros::Time& time);

    void Stop();
    void Start();

  private:

    // ###->
    void ClearTriggerHist();
    void TrimTriggerHist(double max_hist_len = 1.0);
    void ExtTriggerCb(const mavros_msgs::CamIMUStampConstPtr &cam_imu_stamp);
    ros::Time GetClosestTriggerStamp(const ros::Time &stamp, double offset = 0);
    // bool UseExtTriggerStamp() const { return flea3_.UseExtTriggerStamp(); } 

    std::deque<mavros_msgs::CamIMUStamp> ext_trigger_msgs_;
    std::mutex      ext_trigger_mtx_;
    ros::Subscriber sub_ext_trigger_;
    // ###<-

    Flea3Camera flea3_;
    ros::NodeHandle pnh_;
    // ros::Publisher image_metadata_pub_;
  };

}  // namespace flea3

#endif  // FLEA3_FLEA3_ROS_H_
