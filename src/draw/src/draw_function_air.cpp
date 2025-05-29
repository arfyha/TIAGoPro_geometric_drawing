#include <memory>
#include <vector>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.hpp>


static const std::string NODE_NAME = "draw_function_whiteboard_node";
static const std::string PLANNING_GROUP = "arm_right_torso";
static const std::string BASE_FRAME = "base_footprint";
static const std::string END_EFFECTOR_LINK = "arm_right_tool_link";

class DrawFunctionAirNode : public rclcpp::Node {
public:
  DrawFunctionAirNode()
    : Node(NODE_NAME, rclcpp::NodeOptions()
      .automatically_declare_parameters_from_overrides(true)) 
    {
    // Set use_sim_time = true
    //this->declare_parameter("use_sim_time", true);
  }

  void initialize() {
    move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), PLANNING_GROUP);
    visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
      shared_from_this(), BASE_FRAME, rviz_visual_tools::RVIZ_MARKER_TOPIC,
      move_group_interface_->getRobotModel()
    );

    move_group_interface_->setPlanningTime(30.0);
    move_group_interface_->setNumPlanningAttempts(100);
    move_group_interface_->setMaxVelocityScalingFactor(1.0);
    move_group_interface_->setMaxAccelerationScalingFactor(1.0);

    visual_tools_->deleteAllMarkers();
    visual_tools_->loadRemoteControl();

    initializeOrientation();
    //addOrientationConstraint();
    drawFunction();
  }

private:
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
  std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;

  moveit_msgs::msg::Constraints orientation_constraints_;
  geometry_msgs::msg::Quaternion q_msg_;

  const double radius_ = 0.2;
  const double center_x_ = 0.6;
  const double center_y_ = -0.17;
  const double center_z_ = 0.75;
  const int num_points_ = 360;

  void initializeOrientation() {
    tf2::Quaternion q;
    q.setRPY(M_PI / 2, 0, M_PI / 2);
    q.normalize();
    q_msg_ = tf2::toMsg(q);
  }

  void addOrientationConstraint() {
    moveit_msgs::msg::OrientationConstraint ocm;
    ocm.header.frame_id = move_group_interface_->getPoseReferenceFrame();
    ocm.link_name = move_group_interface_->getEndEffectorLink();
    ocm.orientation = q_msg_;
    ocm.absolute_x_axis_tolerance = M_PI;
    ocm.absolute_y_axis_tolerance = 0.2;
    ocm.absolute_z_axis_tolerance = 0.2;
    ocm.weight = 1.0;

    orientation_constraints_.orientation_constraints.push_back(ocm);
  }

  geometry_msgs::msg::Pose calculatePose(int i) {
    double angle = 2 * M_PI * i / num_points_;

    geometry_msgs::msg::Pose pose;
    pose.position.x = center_x_;
    pose.position.y = center_y_ + radius_ * std::cos(angle);
    pose.position.z = center_z_ + radius_ * std::sin(angle);
    pose.orientation = q_msg_;

    return pose;
  }

  void drawTitle(const std::string &text) {
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().x() = 1.0;
    text_pose.translation().z() = 1.0;
    visual_tools_->publishText(text_pose, text, rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  }

  std::pair<bool, moveit::planning_interface::MoveGroupInterface::Plan> createPlan() {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_interface_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    return std::make_pair(success, plan);
  }

  void drawFunction() {
    std::vector<geometry_msgs::msg::Pose> waypoints;
    for (int i = 0; i < num_points_; ++i) {
      auto pose = calculatePose(i);
      
      waypoints.push_back(pose);
    }
    waypoints.push_back(waypoints.front());

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.001;
    double fraction = 0.0;
    int attempt = 0;
    int max_attempts = 100;

    while(fraction < 0.9 && attempt < max_attempts) {
      move_group_interface_->setStartStateToCurrentState();
      fraction = move_group_interface_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
      RCLCPP_INFO(this->get_logger(), "Computing plan %i (%.2f%% achieved)", attempt, fraction * 100.0);
      attempt++;
    }

    if (fraction >= 0.9) {
        RCLCPP_INFO(this->get_logger(), "Successfully planned Cartesian path with %.2f%% coverage.", fraction * 100.0);

        drawTitle("Plan_Cartesian_Path");
        visual_tools_->publishPath(waypoints, rviz_visual_tools::LIME_GREEN, rviz_visual_tools::SMALL);
        for (std::size_t i = 0; i < waypoints.size(); ++i)
            //visual_tools_->publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rviz_visual_tools::SMALL);
            visual_tools_->trigger();
            //visual_tools_->prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

        drawTitle("Execute_Cartesian_Path");
        visual_tools_->trigger();

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;
        move_group_interface_->execute(plan);

        RCLCPP_INFO(this->get_logger(), "Function drawn successfully.");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to plan Cartesian path with sufficient coverage after %d attempts.", max_attempts);
    }

  }
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<DrawFunctionAirNode>();
  node->initialize();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
