#include "flea3/triple_node.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "flea3_triple");
  ros::NodeHandle pnh("~");

  try {
    flea3::TripleNode triple_node(pnh);
    triple_node.Run();
    ros::spin();
    triple_node.End();
  } catch (const std::exception& e) {
    ROS_ERROR("%s: %s", pnh.getNamespace().c_str(), e.what());
  }
}
