#include "flea3/quadruple_node.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "flea3_quadruple");
  ros::NodeHandle pnh("~");

  try {
    flea3::QuadrupleNode quadruple_node(pnh);
    quadruple_node.Run();
    ros::spin();
    quadruple_node.End();
  } catch (const std::exception& e) {
    ROS_ERROR("%s: %s", pnh.getNamespace().c_str(), e.what());
  }
}
