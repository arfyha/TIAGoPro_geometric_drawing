#include <memory>
#include <vector>
#include <iterator>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/kinematics_metrics/kinematics_metrics.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/convert.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/transform_datatypes.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <draw/nullspace_exploration.hpp>
#include <geometry_msgs/msg/twist.hpp>

static const std::string NODE_NAME = "draw_function_whiteboard_node";
static const std::string PLANNING_GROUP = "arm_right";
static const std::string BASE_FRAME = "base_footprint";
static const std::string END_EFFECTOR_LINK = "arm_right_tool_link";
//set head_link to -0.64

class DrawFunctionWhiteboardNode : public rclcpp::Node {
public:
  DrawFunctionWhiteboardNode()
    : Node(NODE_NAME, rclcpp::NodeOptions()
      .automatically_declare_parameters_from_overrides(true)) 
    {
    // Set use_sim_time = true
    //this->declare_parameter("use_sim_time", true);
    tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    br = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  }

  void initialize() {
    move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), PLANNING_GROUP);
    visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
      shared_from_this(), BASE_FRAME, rviz_visual_tools::RVIZ_MARKER_TOPIC,
      move_group_interface_->getRobotModel()
    );
    nullspace_explorer_ = std::make_shared<NullspaceExplorationNode>();
    robot_model_ = move_group_interface_->getRobotModel();
    jmg_ = robot_model_->getJointModelGroup(PLANNING_GROUP);

    // Remove all collision objects from the scene
    planning_scene_interface.removeCollisionObjects(planning_scene_interface.getKnownObjectNames());
    visual_tools_->deleteAllMarkers();
    visual_tools_->loadRemoteControl();
    visual_tools_->trigger();

    driveForwardCallback();
    rclcpp::sleep_for(std::chrono::seconds(1)); // Wait for the robot to stop moving and point cloud setteling

    move_group_interface_->setPlannerId("RRTConnectkConfigDefault");
    planning_scene_interface.applyCollisionObject(createCollisionObject());
    
    move_group_interface_->setPlanningTime(30.0);
    move_group_interface_->setNumPlanningAttempts(200);
    move_group_interface_->setMaxVelocityScalingFactor(1.0);
    move_group_interface_->setMaxAccelerationScalingFactor(1.0);

    drawFunction();
  }

private:
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
  std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  
  std::shared_ptr<NullspaceExplorationNode> nullspace_explorer_;
  moveit::core::RobotModelConstPtr robot_model_;
  const moveit::core::JointModelGroup* jmg_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> br;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  void drawTitle(const std::string &text) {
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().x() = 1.0;
    text_pose.translation().z() = 1.0;
    visual_tools_->publishText(text_pose, text, rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  }

  void drawFunction() {
    std::vector<geometry_msgs::msg::Pose> waypoints;
    for (int i = 0; i < 360; ++i) {
      geometry_msgs::msg::TransformStamped transformStamped;
      try {
        transformStamped = tf_buffer_->lookupTransform(BASE_FRAME, "function_point_" + std::to_string(i), tf2::TimePointZero, tf2::durationFromSec(1.0));
      } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "Could not transform: %s", ex.what());
        return;
      }
      geometry_msgs::msg::Pose pose;
      pose.position.x = transformStamped.transform.translation.x;
      pose.position.y = transformStamped.transform.translation.y;
      pose.position.z = transformStamped.transform.translation.z;
      pose.orientation = transformStamped.transform.rotation;

      //auto pose_ = calculatePose(i);
      waypoints.push_back(pose);
      visual_tools_->publishSphere(pose, rviz_visual_tools::RED, rviz_visual_tools::SMALL);
    }
    visual_tools_->trigger();
    //return;

    move_group_interface_->setPoseTarget(waypoints.front());
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto success = move_group_interface_->plan(plan);
    if (success == moveit::core::MoveItErrorCode::SUCCESS) {
      move_group_interface_->execute(plan);
      //nullspaceExploration(plan);
      RCLCPP_INFO(this->get_logger(), "Initial pose set successfully.");
      //return;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to set initial pose.");
      return;
    }

    //waypoints.push_back(waypoints.front());

    for(auto &waypoint : waypoints) {
      move_group_interface_->setPoseTarget(waypoint);
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      auto success = move_group_interface_->plan(plan);
      if (success == moveit::core::MoveItErrorCode::SUCCESS) {
        move_group_interface_->execute(plan);
        //nullspaceExploration(plan);
        RCLCPP_INFO(this->get_logger(), "pose set successfully.");
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to set pose.");
        return;
      }
    }
    return;

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 1000.0;
    const double eef_step = 1e-4;
    double fraction = 0.0;
    int attempt = 0;
    int max_attempts = 10;
    const double path_coverage = 0.85;

    while(fraction < path_coverage && attempt < max_attempts) {
      move_group_interface_->setStartStateToCurrentState();
      fraction = move_group_interface_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, true);
      RCLCPP_INFO(this->get_logger(), "Computing plan %i (%.2f%% achieved)", attempt, fraction * 100.0);
      attempt++;
    }

    if (fraction >= path_coverage) {
        RCLCPP_INFO(this->get_logger(), "Successfully planned Cartesian path with %.2f%% coverage.", fraction * 100.0);

        //drawTitle("Execute_Cartesian_Path");
        visual_tools_->publishTrajectoryLine(trajectory, jmg_);
        visual_tools_->trigger();
        //return;
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;
        move_group_interface_->execute(plan);

        planning_scene_interface.removeCollisionObjects(planning_scene_interface.getKnownObjectNames());

        RCLCPP_INFO(this->get_logger(), "Function drawn successfully.");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to plan Cartesian path with sufficient coverage after %d attempts.", max_attempts);
    }

  }

  // Create collision object for the robot to avoid
  moveit_msgs::msg::CollisionObject createCollisionObject(){
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = "whiteboard_bb_center";
    collision_object.header.stamp = this->get_clock()->now();
    collision_object.id = "whiteboard";
    shape_msgs::msg::SolidPrimitive primitive;
    geometry_msgs::msg::TransformStamped whiteboard_min_pt;
    geometry_msgs::msg::TransformStamped whiteboard_max_pt;
    try {
        whiteboard_min_pt = tf_buffer_->lookupTransform("whiteboard_bb_center", "whiteboard_min_pt", tf2::TimePointZero, tf2::durationFromSec(5.0));
        whiteboard_max_pt = tf_buffer_->lookupTransform("whiteboard_bb_center", "whiteboard_max_pt", tf2::TimePointZero, tf2::durationFromSec(5.0));
      } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "Could not transform: %s", ex.what());
        return collision_object;
      }

    // Define the size of the box in meters
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    const auto& t1 = whiteboard_max_pt.transform.translation;
    const auto& t2 = whiteboard_min_pt.transform.translation;
    primitive.dimensions[primitive.BOX_X] = std::abs(t1.x - t2.x) + 0.02; 
    primitive.dimensions[primitive.BOX_Y] = std::abs(t1.y - t2.y) + 0.02;
    primitive.dimensions[primitive.BOX_Z] = 0.07; // Width of the whiteboard plus extra space for the robot to avoid it

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose whiteboard_pose;
    whiteboard_pose.position.z = -0.03;
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(whiteboard_pose);
    collision_object.operation = collision_object.ADD;

    return collision_object;
  }

  void driveForwardCallback(){
    geometry_msgs::msg::TransformStamped transformStamped;
    try {
      transformStamped = tf_buffer_->lookupTransform(BASE_FRAME, "whiteboard_bb_center", tf2::TimePointZero, tf2::durationFromSec(1.0));
    } catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(), "Could not transform: %s", ex.what());
      return;
    }
    double distance_threshold = 0.7;
    while (transformStamped.transform.translation.x > distance_threshold) {
      RCLCPP_INFO(this->get_logger(), "Driving forward to ensure whiteboard is in range, distance: %.2f", transformStamped.transform.translation.x);
      geometry_msgs::msg::Twist cmd_vel_msg;
      if (transformStamped.transform.translation.x < distance_threshold + 0.1)
        cmd_vel_msg.linear.x = 0.05;
      else
        cmd_vel_msg.linear.x = 0.1;
      cmd_vel_msg.angular.z = 0.0;
      cmd_vel_pub_->publish(cmd_vel_msg);

      try {
        transformStamped = tf_buffer_->lookupTransform(BASE_FRAME, "whiteboard_bb_center", tf2::TimePointZero, tf2::durationFromSec(1.0));
      } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "Could not transform: %s", ex.what());
        return;
      }
    }
    RCLCPP_INFO(this->get_logger(), "Reached the whiteboard, distance: %.2f", transformStamped.transform.translation.x);
    geometry_msgs::msg::Twist cmd_vel_msg;
    cmd_vel_msg.linear.x = 0.0;
    cmd_vel_msg.angular.z = 0.0;
    cmd_vel_pub_->publish(cmd_vel_msg);
  }

  void nullspaceExploration(moveit::planning_interface::MoveGroupInterface::Plan plan){
      //Nullspace exploration
      move_group_interface_->clearPoseTargets();
      std::vector<double> planned_joint_values = plan.trajectory_.joint_trajectory.points.back().positions;
      moveit::core::RobotState end_state(robot_model_);
      end_state.setJointGroupPositions(jmg_, planned_joint_values);
      end_state.update();
      std::vector<double> best_joint_values = nullspace_explorer_->explore(end_state);
      move_group_interface_->setJointValueTarget(best_joint_values);
      
      // Plan the trajectory
      moveit::planning_interface::MoveGroupInterface::Plan plan_joints;
      bool success = (move_group_interface_->plan(plan_joints) == moveit::core::MoveItErrorCode::SUCCESS);
      
      if (!success)
      {
          RCLCPP_ERROR(this->get_logger(), "Failed to plan trajectory!");
          return;
      }

      // Execute the planned trajectory
      moveit::core::MoveItErrorCode result = move_group_interface_->execute(plan);
      if (result == moveit::core::MoveItErrorCode::SUCCESS)
      {
          RCLCPP_INFO(this->get_logger(), "Nullspace motion executed successfully!");
      }
      else
      {
          RCLCPP_ERROR(this->get_logger(), "Nullspace motion execution failed!");
          return;
      }
  }
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<DrawFunctionWhiteboardNode>();
  node->initialize();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
