#include <chrono>
#include <functional>
#include <memory>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include <tf2/convert.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/transform_datatypes.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

class StaticFrameBroadcaster : public rclcpp::Node
{
public:
  StaticFrameBroadcaster()
  : Node("frame_broadcaster")
  {
    tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // Broadcast the static transforms
    broadcastTransforms();
  }

private:
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;

  void broadcastTransforms() {
    // Broadcast the center transform
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "arm_right_tool_link";
    t.child_frame_id = "pen_tip";
    t.transform.translation.x = 0;
    t.transform.translation.y = 0;
    t.transform.translation.z = 0.25;
    t.transform.rotation.w = 1.0;
    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = 0.0;
    tf_broadcaster_->sendTransform(t);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StaticFrameBroadcaster>());
  rclcpp::shutdown();
  return 0;
}