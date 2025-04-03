#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include<tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>

static const std::string DRAW_CRICLE = "draw_circle";
static const std::string PLANNING_GROUP = "arm_right_torso";
static const std::string BASE_FRAME = "base_footprint";
static const std::string END_EFFECTOR_LINK = "arm_right_tool_link";

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    DRAW_CRICLE,
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger(DRAW_CRICLE);

  tf2::Quaternion q;
  q.setRPY(0, 3.14/2, 0);
  geometry_msgs::msg::Quaternion const Q_MSG = tf2::toMsg(q);

  // Define circle parameters
  const double RADIUS = 0.2; // Radius of the circle
  const double CENTER_X = 0.75; // X-coordinate of the circle's center
  const double CENTER_Y = 0.0;  // Y-coordinate of the circle's center
  const double CENTER_Z = 0.75; // Z-coordinate of the circle's center
  const int NUM_POINTS = 4;    // Number of points in the circle

  // We spin up a SingleThreadedExecutor so MoveItVisualTools interact with ROS
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, PLANNING_GROUP);
  
  // Construct and initialize MoveItVisualTools
  auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
    node, BASE_FRAME, rviz_visual_tools::RVIZ_MARKER_TOPIC,
    move_group_interface.getRobotModel()};
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();

  // Show the title of the momentary action
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

  // Wait for the user to press 'Next' in the RvizVisualToolsGui window
  auto const prompt = [&moveit_visual_tools]() {
    moveit_visual_tools.prompt("Press 'Next' in the RvizVisualToolsGui window to execute");
  };

  // Draw the trajectory
  auto const draw_trajectory_tool_path =
      [&moveit_visual_tools,
      jmg = move_group_interface.getRobotModel()->getJointModelGroup(
          PLANNING_GROUP)](auto const trajectory) {
        moveit_visual_tools.publishTrajectoryLine(trajectory, jmg);
      };
      
  // Create a plan to draw the circle
  auto const create_plan = [&move_group_interface](){
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = move_group_interface.plan(msg);
    return std::make_pair(ok, msg);
  };

  // Create a function to calculate the points along the circle
  auto const calculate_pose = [NUM_POINTS, Q_MSG, CENTER_X, CENTER_Y, CENTER_Z, RADIUS](int i) {
      double angle = 2 * M_PI * i / NUM_POINTS; // Angle for this point
      geometry_msgs::msg::Pose pose;
      pose.position.x = CENTER_X;
      pose.position.y = CENTER_Y + RADIUS * cos(angle);
      pose.position.z = CENTER_Z + RADIUS * sin(angle);
      pose.orientation = Q_MSG; // Keep orientation constant#
      return pose;
    };

  // We can print the name of the reference frame for this robot.
  RCLCPP_INFO(logger, "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  RCLCPP_INFO(logger, "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  RCLCPP_INFO(logger, "Available Planning Groups:");
  std::copy(move_group_interface.getJointModelGroupNames().begin(), move_group_interface.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));

  moveit_visual_tools.trigger();
  //prompt();

  move_group_interface.setPlanningTime(10.0);

  /*moveit_msgs::msg::OrientationConstraint ocm;
  ocm.link_name = END_EFFECTOR_LINK;
  ocm.header.frame_id = BASE_FRAME;
  ocm.orientation = Q_MSG;
  ocm.absolute_x_axis_tolerance = 0.5;
  ocm.absolute_y_axis_tolerance = 0.5;
  ocm.absolute_z_axis_tolerance = 0.5;
  ocm.weight = 1.0;

  // set the path constraints
  moveit_msgs::msg::Constraints constraints;
  constraints.orientation_constraints.push_back(ocm);
  move_group_interface.setPathConstraints(constraints);*/

  
  // Generate poses along the circle
  for (int i = 0; i <= NUM_POINTS; ++i) {
    geometry_msgs::msg::Pose pose = calculate_pose(i % NUM_POINTS);
    move_group_interface.setPoseTarget(pose);
    RCLCPP_INFO(logger, "Moving to position x: %f y: %f z: %f", pose.position.x, pose.position.y, pose.position.z);

    draw_title(std::string("Planning_position_") + std::to_string(i));
    moveit_visual_tools.trigger();
    auto const [success, plan] = create_plan();

    // Execute the plan
    if (success == moveit::core::MoveItErrorCode::SUCCESS) {
      draw_trajectory_tool_path(plan.trajectory_);
      moveit_visual_tools.publishAxisLabeled(pose, std::string("position_") + std::to_string(i));
      moveit_visual_tools.trigger();
      //prompt();
      draw_title(std::string("Moving_to_position_") + std::to_string(i));
      moveit_visual_tools.trigger();
      move_group_interface.execute(plan);
    } else {
      draw_title("Planning_Failed!");
      moveit_visual_tools.trigger();
      RCLCPP_ERROR(logger, "Planing failed!");
    }
  }

  // Shutdown ROS
  rclcpp::shutdown();  // <--- This will cause the spin function in the thread to return
  spinner.join();  // <--- Join the thread before exiting
  return 0;
}