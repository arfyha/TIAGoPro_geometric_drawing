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
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

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

  // Set a target Pose
  auto const target_pose_1 = [q_msg]{
    geometry_msgs::msg::Pose msg;
    msg.orientation = q_msg;
    msg.position.x = 0.75;
    msg.position.y = -0.3;
    msg.position.z = 0.75;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose_1);

  // Create collision object for the robot to avoid
  auto const collision_object = [frame_id =
    move_group_interface.getPlanningFrame()] {
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = frame_id;
  collision_object.id = "box1";
  shape_msgs::msg::SolidPrimitive primitive;

  // Define the size of the box in meters
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 0.5;
  primitive.dimensions[primitive.BOX_Y] = 0.1;
  primitive.dimensions[primitive.BOX_Z] = 0.5;

  // Define the pose of the box (relative to the frame_id)
  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.75;
  box_pose.position.y = 0.0;
  box_pose.position.z = 0.75;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  return collision_object;
  }();

  // Add the collision object to the scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyCollisionObject(collision_object);

  // Create a plan to that target pose
  prompt("Press 'Next' in the RvizVisualToolsGui window to plan");
  draw_title("Planning Pos 1");
  moveit_visual_tools.trigger();
  auto const [success_1, plan_1] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan to Pos 1
  if (success_1) {
    draw_trajectory_tool_path(plan_1.trajectory_);
    moveit_visual_tools.trigger();
    prompt("Press 'Next' in the RvizVisualToolsGui window to execute");
    draw_title("Executing Pos 1");
    moveit_visual_tools.trigger();
    move_group_interface.execute(plan_1);
  } else {
    draw_title("Planning Failed Pos 1!");
    moveit_visual_tools.trigger();
    RCLCPP_ERROR(logger, "Planing failed Pos 1!");
  }

  // Set a new target Pose
  auto const target_pose_2 = [q_msg]{
    geometry_msgs::msg::Pose msg;
    msg.orientation = q_msg;
    msg.position.x = 0.75;
    msg.position.y = 0.3;
    msg.position.z = 0.75;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose_2);

  // Create a plan to target pose 2
  prompt("Press 'Next' in the RvizVisualToolsGui window to plan");
  draw_title("Planning Pos 2");
  moveit_visual_tools.trigger();
  auto const [success_2, plan_2] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan to Pos 2
  if (success_2) {
    draw_trajectory_tool_path(plan_2.trajectory_);
    moveit_visual_tools.trigger();
    prompt("Press 'Next' in the RvizVisualToolsGui window to execute");
    draw_title("Executing Pos 2");
    moveit_visual_tools.trigger();
    move_group_interface.execute(plan_2);
  } else {
    draw_title("Planning Failed Pos 2!");
    moveit_visual_tools.trigger();
    RCLCPP_ERROR(logger, "Planing failed Pos 2!");
  }

  // Shutdown ROS
  rclcpp::shutdown();  // <--- This will cause the spin function in the thread to return
  spinner.join();  // <--- Join the thread before exiting
  return 0;
}