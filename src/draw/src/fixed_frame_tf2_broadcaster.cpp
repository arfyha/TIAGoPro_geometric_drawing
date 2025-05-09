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
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

using namespace std::chrono_literals;

class FixedFrameBroadcaster : public rclcpp::Node
{
public:
  FixedFrameBroadcaster()
  : Node("fixed_frame_tf2_broadcaster")
  {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    timer_ = this->create_wall_timer(
      100ms, std::bind(&FixedFrameBroadcaster::broadcast_timer_callback, this));

      roll = degToRad(this->declare_parameter("deg", 90.0));
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  tf2Scalar roll;
  const double radius_ = 0.2;
  const double center_x_ = 0.78;
  const double center_y_ = -0.17;
  const double center_z_ = 0.75;
  const int num_points_ = 4;

  void broadcast_timer_callback() {

    roll = degToRad(this->get_parameter("deg").as_double());

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "base_footprint";
    t.child_frame_id = "circle_center";
    t.transform.translation.x = center_x_;
    t.transform.translation.y = center_y_;
    t.transform.translation.z = center_z_;
    tf2::Quaternion q;
    q.setRPY(roll, 0, M_PI / 2);
    q.normalize();
    geometry_msgs::msg::Quaternion q_msg_ = tf2::toMsg(q);
    t.transform.rotation = q_msg_;
    tf_broadcaster_->sendTransform(t);

    for (int i = 0; i < num_points_; ++i) {
      double angle = 2 * M_PI * i / num_points_;
      geometry_msgs::msg::TransformStamped ti;
      ti.header.stamp = this->get_clock()->now();
      ti.header.frame_id = "circle_center";
      ti.child_frame_id = "circle_point_" + std::to_string(i);
      ti.transform.translation.x = radius_ * std::cos(angle);
      ti.transform.translation.y = radius_ * std::sin(angle);
      ti.transform.translation.z = -0.18;
      ti.transform.rotation.w = 1.0;
      ti.transform.rotation.x = 0.0;
      ti.transform.rotation.y = 0.0;
      ti.transform.rotation.z = 0.0;

      tf_broadcaster_->sendTransform(ti);
    }
  }

  double degToRad(double degrees) {
    return degrees * M_PI / 180.0;
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FixedFrameBroadcaster>());
  rclcpp::shutdown();
  return 0;
}