#include "flea3/flea3_ros.h"

namespace flea3 {

  Flea3Ros::Flea3Ros(const ros::NodeHandle& pnh, const std::string& prefix)
  : CameraRosBase(pnh, prefix), flea3_(identifier()), pnh_(pnh) {
    SetHardwareId(flea3_.serial());
  }

  Flea3Camera& Flea3Ros::camera() { return flea3_; }

  bool Flea3Ros::Grab(const sensor_msgs::ImagePtr& image_msg,
    const sensor_msgs::CameraInfoPtr& cinfo_msg) {

     auto t00 = ros::Time::now().toSec() * 1e3;
    auto suc = flea3_.GrabImage(*image_msg);
     auto t01 = ros::Time::now().toSec() * 1e3;

     printf("Time to grab : %lf\n", t01 - t00);

    if(suc == false)
      return suc;

    const auto expose_duration = flea3_.GetShutterTimeSec();

    // The right time stamp is [current time - camera exposure - unknown_offset]
    ros::Time time = ros::Time::now(); // - ros::Duration(0, (expose_duration + 0.007) * 1.0e+9);

    if(flea3_.UseExtTriggerStamp() == true)
        time = GetClosestTriggerStamp(time, - (expose_duration + 0.007));

    image_msg->header.frame_id = frame_id();
    image_msg->header.stamp = time;

    return suc;

  // ### return flea3_.GrabImage(*image_msg);
  }

  void Flea3Ros::Stop() { 
    // ###->
    sub_ext_trigger_.shutdown();
    ClearTriggerHist();
    // ###-<
    flea3_.StopCapture(); 
  }

  void Flea3Ros::Start() { 

    // ###->
    if(flea3_.UseExtTriggerStamp() == true){
      sub_ext_trigger_ = pnh_.subscribe("cam_imu_stamp", 1, &Flea3Ros::ExtTriggerCb, this);
    } else {
      sub_ext_trigger_.shutdown();
      ClearTriggerHist();
    }
    // ###-<

    flea3_.StartCapture(); 
  }

  bool Flea3Ros::RequestSingle() { return flea3_.RequestSingle(); }

  void Flea3Ros::ClearTriggerHist(){
    ext_trigger_mtx_.lock();
    ext_trigger_msgs_.clear();
    ext_trigger_mtx_.unlock();
  }

  void Flea3Ros::TrimTriggerHist(double max_hist_len){

    ext_trigger_mtx_.lock();

    auto erase_fun = [&](const mavros_msgs::CamIMUStamp &msg){ 
      return ext_trigger_msgs_.front().frame_stamp.toSec() - msg.frame_stamp.toSec() > max_hist_len; 
    };

    auto new_end = std::remove_if(ext_trigger_msgs_.begin(), ext_trigger_msgs_.end(), erase_fun);

    ext_trigger_msgs_.erase(new_end, ext_trigger_msgs_.end());

    ext_trigger_mtx_.unlock();
  }

  void Flea3Ros::ExtTriggerCb(const mavros_msgs::CamIMUStampConstPtr &cam_imu_stamp) {

    ext_trigger_mtx_.lock();
    ext_trigger_msgs_.push_front(*cam_imu_stamp);    
    ext_trigger_mtx_.unlock();

    TrimTriggerHist(1.0);

    /*
    std::cout << "-------------------------" << std::endl;
    for(auto m : ext_trigger_msgs_)
      std::cout << m;
    std::cout << "-------------------------" << std::endl;
    */
  }
  ros::Time Flea3Ros::GetClosestTriggerStamp(const ros::Time &stamp, double offset){

    auto delta_stamp = [&](const ros::Time &s){ return (stamp.toSec() + offset - s.toSec());};

    ros::Time closest_stamp(0, 0);
    double min_delta_stamp = 1.0e+10;

    for(auto &s : ext_trigger_msgs_){

      double temp_delta_stamp = delta_stamp(s.frame_stamp);

      if(temp_delta_stamp >= 0 && temp_delta_stamp < min_delta_stamp){
        closest_stamp = s.frame_stamp;
        min_delta_stamp = temp_delta_stamp;
      }
    }
    
    return closest_stamp;
  }

}  // namespace flea3
