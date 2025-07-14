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

/**
 * @brief Node that broadcasts fixed TF2 frames for whiteboard and function points.
 */
class FixedFrameBroadcaster : public rclcpp::Node
{
public:
  /**
   * @brief Constructor. Initializes TF2 broadcaster and subscribes to pose array.
   */
  FixedFrameBroadcaster()
  : Node("fixed_frame_tf2_broadcaster")
  {
    // TF2 broadcaster for publishing transforms
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    
    // Subscribe to pose array topic for whiteboard poses
    pose_array_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "/whiteboard_pose", 10,
      std::bind(&FixedFrameBroadcaster::poseArrayCallback, this, std::placeholders::_1));

    // Get roll angle from parameter (default 90 degrees)
    // Uncomment to broadcast a fixed frame for the function center
    //roll = degToRad(this->declare_parameter("deg", 90.0));
  }

private:
  // Subscription to whiteboard info and tf2 broadcaster
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Function paramerters
  //tf2Scalar roll;
  const double radius_ = 0.2;
  const double center_x_ = 0.75;
  const double center_y_ = -0.0;
  const double center_z_ = 1.0;
  const int num_points_ = 100;
  double x_start = -M_PI;
  double x_end = M_PI;
  double step = (x_end - x_start) / (num_points_ - 1);
  std::function<double(double)> function = [](double x) { return std::sin(x); };
  double scale_x = 4 * M_PI;
  double scale_y = 10.0;


  /**
   * @brief Callback for incoming whiteboard pose array.
   *        Broadcasts transforms for whiteboard properties and function points.
   * @param msg Shared pointer to PoseArray message.
   */
  void poseArrayCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg){

    // Uncomment the following lines if you want to broadcast a fixed frame for the function center
    //roll = degToRad(this->get_parameter("deg").as_double());

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "base_footprint";

    // Broadcast transforms for whiteboard properties
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

    // Broadcast transform for function center
    t.child_frame_id = "function_center";
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
    tf2::Quaternion new_q = orig_q * rot_q;

    // Set the result as the new orientation
    t.transform.rotation = tf2::toMsg(new_q.normalized());
    tf_broadcaster_->sendTransform(t);

    //RCLCPP_INFO(this->get_logger(), "whiteboard orientation: (%f, %f, %f, %f)", orig_q.x(), orig_q.y(), orig_q.z(), orig_q.w());
    //RCLCPP_INFO(this->get_logger(), "fixed rotation: (%f, %f, %f, %f)", rot_q.x(), rot_q.y(), rot_q.z(), rot_q.w());
    //RCLCPP_INFO(this->get_logger(), "quaternion product: (%f, %f, %f, %f)", new_q.x(), new_q.y(), new_q.z(), new_q.w());

    // Uncomment the following lines if you want to broadcast a fixed frame for the function center
    /*t.child_frame_id = "function_center_fixed";
    t.transform.translation.x = center_x_;
    t.transform.translation.y = center_y_;
    t.transform.translation.z = center_z_;
    tf2::Quaternion q;
    q.setRPY(roll, 0, M_PI / 2);
    q.normalize();
    geometry_msgs::msg::Quaternion q_msg_ = tf2::toMsg(q);
    t.transform.rotation = q_msg_;
    tf_broadcaster_->sendTransform(t);*/

    // Broadcast transforms for each function point
    for (int i = 0; i < num_points_; ++i) {
      geometry_msgs::msg::TransformStamped ti;
      ti.header.stamp = this->get_clock()->now();
      ti.header.frame_id = "function_center";
      ti.child_frame_id = "function_point_" + std::to_string(i);
      std::vector<double> point = getRectangleCoordinates().at(i);
      ti.transform.translation.x = -point.at(0);
      ti.transform.translation.y = point.at(1);
      ti.transform.translation.z = -0.25;

      tf_broadcaster_->sendTransform(ti);
    }
  }

  /**
   * @brief Calculates the i-th point of a scaled function (e.g., sine).
   * @param i Index of the point.
   * @return Vector with x and y coordinates.
   */
  std::vector<double> calculatePointsFunction(int i) {
    double x = x_start + i * step;
    double y = function(x);
    return {x/scale_x, y/scale_y};
  }

  /**
   * @brief Calculates the i-th point on a circle.
   * @param i Index of the point.
   * @return Vector with x and y coordinates.
   */
  std::vector<double> calculateCirclePoints(int i) {
    double angle = 2 * M_PI * i / num_points_;
    double x = radius_ * std::cos(angle);
    double y = radius_ * std::sin(angle);
    return {x, y};
  }

  /**
   * @brief Converts degrees to radians.
   * @param degrees Angle in degrees.
   * @return Angle in radians.
   */
  double degToRad(double degrees) {
    return degrees * M_PI / 180.0;
  }

  /**
   * @brief Returns coordinates for points distributed along a rectangle perimeter.
   * @return Vector of vectors with x and y coordinates.
   */
  std::vector<std::vector<double>> getRectangleCoordinates() {
    double side = 0.2;
    double half = side / 2.0;
    std::vector<std::vector<double>> rect_points;
    double perimeter = 4 * side;
    double step = perimeter / num_points_;

    for (int i = 0; i < num_points_; ++i) {
      double dist = i * step;
      double x, y;
      if (dist < side) { // bottom edge: left to right
        x = -half + dist;
        y = -half;
      } else if (dist < 2 * side) { // right edge: bottom to top
        x = half;
        y = -half + (dist - side);
      } else if (dist < 3 * side) { // top edge: right to left
        x = half - (dist - 2 * side);
        y = half;
      } else { // left edge: top to bottom
        x = -half;
        y = half - (dist - 3 * side);
      }
      rect_points.push_back({x, y});
    }
    return rect_points;
  }
};

/**
 * @brief Main entry point. Initializes ROS, starts the node, and spins.
 */
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FixedFrameBroadcaster>());
  rclcpp::shutdown();
  return 0;
}