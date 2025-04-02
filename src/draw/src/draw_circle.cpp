#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include<tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "draw_circle",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("draw_circle");

  // We spin up a SingleThreadedExecutor so MoveItVisualTools interact with ROS
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "arm_right_torso");

  // Construct and initialize MoveItVisualTools
  auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
    node, "base_footprint", rviz_visual_tools::RVIZ_MARKER_TOPIC,
    move_group_interface.getRobotModel()};
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();

  // Create a closures for visualization
  auto const draw_title = [&moveit_visual_tools](auto text) {
    auto const text_pose = [] {
      auto msg = Eigen::Isometry3d::Identity();
      msg.translation().x() = 1.0;
      msg.translation().z() = 1.0;
      return msg;
    }();
    moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE,
                                    rviz_visual_tools::XLARGE);
  };
  auto const prompt = [&moveit_visual_tools](auto text) {
    moveit_visual_tools.prompt(text);
  };
  auto const draw_trajectory_tool_path =
      [&moveit_visual_tools,
      jmg = move_group_interface.getRobotModel()->getJointModelGroup(
          "arm_right_torso")](auto const trajectory) {
        moveit_visual_tools.publishTrajectoryLine(trajectory, jmg);
      };

  tf2::Quaternion q;
  q.setRPY(0, 3.14/2, 0);
  geometry_msgs::msg::Quaternion q_msg = tf2::toMsg(q);

  // Define circle parameters
  double radius = 0.2; // Radius of the circle
  double center_x = 0.75; // X-coordinate of the circle's center
  double center_y = 0.0;  // Y-coordinate of the circle's center
  double center_z = 0.75; // Z-coordinate of the circle's center
  int num_points = 20;    // Number of points in the circle
  
  // Generate poses along the circle
  for (int i = 0; i < num_points; ++i) {
    double angle = 2 * M_PI * i / num_points; // Angle for this point
    geometry_msgs::msg::Pose pose;
    pose.position.x = center_x ;
    pose.position.y = center_y + radius * cos(angle);
    pose.position.z = center_z + radius * sin(angle);
    pose.orientation = q_msg; // Keep orientation constant#
    

    RCLCPP_INFO(logger, "Moving to position x: %f y: %f z: %f", pose.position.x, pose.position.y, pose.position.z);

    move_group_interface.setPoseTarget(pose);


    // Create a plan to draw the circle
    //prompt("Press 'Next' in the RvizVisualToolsGui window to plan");
    //draw_title("Planning");
    moveit_visual_tools.trigger();
    auto const [success, plan] = [&move_group_interface]{
      moveit::planning_interface::MoveGroupInterface::Plan msg;
      auto const ok = static_cast<bool>(move_group_interface.plan(msg));
      return std::make_pair(ok, msg);
    }();

    // Execute the plan
    if (success) {
      draw_trajectory_tool_path(plan.trajectory_);
      moveit_visual_tools.trigger();
      //7prompt("Press 'Next' in the RvizVisualToolsGui window to execute");
      //draw_title("Executing");
      moveit_visual_tools.trigger();
      move_group_interface.execute(plan);
    } else {
      draw_title("Planning Failed!");
      moveit_visual_tools.trigger();
      RCLCPP_ERROR(logger, "Planing failed!");
    }
  }

  // Shutdown ROS
  rclcpp::shutdown();  // <--- This will cause the spin function in the thread to return
  spinner.join();  // <--- Join the thread before exiting
  return 0;
}