#ifndef FLEA3_QUADRUPLE_NODE_H_
#define FLEA3_QUADRUPLE_NODE_H_

#include "flea3/flea3_ros.h"
#include "flea3/Flea3DynConfig.h"
#include "camera_base/camera_node_base.h"
#include <mavros_msgs/CamIMUStamp.h>

namespace flea3 {

class QuadrupleNode : public camera_base::CameraNodeBase<Flea3DynConfig> {
 public:
  explicit QuadrupleNode(ros::NodeHandle &pnh);

  virtual void Acquire() override;
  virtual void Setup(Flea3DynConfig &config) override;
  
 private:
  Flea3Ros top_left_ros_;
  Flea3Ros top_right_ros_;
  Flea3Ros bottom_left_ros_;
  Flea3Ros bottom_right_ros_;
};

}  // namespace flea3

#endif  // FLEA3_quadruple_NODE_H_
