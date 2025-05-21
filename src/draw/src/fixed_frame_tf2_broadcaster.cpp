#include <chrono>
#include <functional>
#include <memory>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <rclcpp/rclcpp.hpp>
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
    pose_array_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "/whiteboard_pose", 10,
      std::bind(&FixedFrameBroadcaster::poseArrayCallback, this, std::placeholders::_1));

      roll = degToRad(this->declare_parameter("deg", 90.0));
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  tf2Scalar roll;
  const double radius_ = 0.2;
  const double center_x_ = 0.75;
  const double center_y_ = -0.0;
  const double center_z_ = 1.0;
  const int num_points_ = 360;

  void poseArrayCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg){

    roll = degToRad(this->get_parameter("deg").as_double());

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "base_footprint";

    t.child_frame_id = "whiteboard_center";
    t.transform.translation.x = msg->poses.at(0).position.x;
    t.transform.translation.y = msg->poses.at(0).position.y;
    t.transform.translation.z = msg->poses.at(0).position.z;
    t.transform.rotation = msg->poses.at(0).orientation;
    tf_broadcaster_->sendTransform(t);

    t.child_frame_id = "whiteboard_min_pt";
    t.transform.translation.x = msg->poses.at(1).position.x;
    t.transform.translation.y = msg->poses.at(1).position.y;
    t.transform.translation.z = msg->poses.at(1).position.z;
    tf_broadcaster_->sendTransform(t);

    t.child_frame_id = "whiteboard_max_pt";
    t.transform.translation.x = msg->poses.at(2).position.x;
    t.transform.translation.y = msg->poses.at(2).position.y;
    t.transform.translation.z = msg->poses.at(2).position.z;
    tf_broadcaster_->sendTransform(t);

    t.child_frame_id = "whiteboard_bb_center";
    t.transform.translation.x = msg->poses.at(3).position.x;
    t.transform.translation.y = msg->poses.at(3).position.y;
    t.transform.translation.z = msg->poses.at(3).position.z;
    tf_broadcaster_->sendTransform(t);

    t.child_frame_id = "circle_center";
    t.transform.translation.x = msg->poses.at(0).position.x;
    t.transform.translation.y = msg->poses.at(0).position.y;
    t.transform.translation.z = msg->poses.at(0).position.z;
    // Convert the original orientation to tf2 quaternion
    tf2::Quaternion orig_q;
    tf2::fromMsg(msg->poses.at(0).orientation, orig_q);
    // Create a quaternion from desired Euler angles (replace roll(z), pitch(x), yaw(y) as needed)
    double roll_angle = M_PI / 2;   // set your desired roll in radians
    double pitch_angle = 0.0;  // set your desired pitch in radians
    double yaw_angle = M_PI;    // set your desired yaw in radians
    tf2::Quaternion rot_q;
    rot_q.setEuler(yaw_angle, pitch_angle, roll_angle);
    // Combine the rotations: new_q = orig_q * rot_q
    tf2::Quaternion new_q = orig_q.operator*=(rot_q);
    // Set the result as the new orientation
    t.transform.rotation = tf2::toMsg(new_q.normalized());
    tf_broadcaster_->sendTransform(t);

    t.child_frame_id = "circle_center_fixed";
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
      ti.transform.translation.z = -0.2;

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