#include <memory>
#include <vector>
#include <iterator>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>

static const std::string NODE_NAME = "draw_circle";
static const std::string PLANNING_GROUP = "arm_right";
static const std::string BASE_FRAME = "base_footprint";
static const std::string END_EFFECTOR_LINK = "arm_right_tool_link";

class DrawCircleNode : public rclcpp::Node {
public:
  DrawCircleNode()
    : Node(NODE_NAME, rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) {
    // Set use_sim_time = true
    //this->declare_parameter("use_sim_time", true);
  }

  void initialize() {
    // Use node shared_ptr now that it's safe
    move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), PLANNING_GROUP);
    visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
      shared_from_this(), BASE_FRAME, rviz_visual_tools::RVIZ_MARKER_TOPIC,
      move_group_interface_->getRobotModel()
    );

    visual_tools_->deleteAllMarkers();
    visual_tools_->loadRemoteControl();

    initializeOrientation();
    addOrientationConstraint();
    logBasicInfo();
    drawCircle();
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
  const int num_points_ = 100;

  void initializeOrientation() {
    tf2::Quaternion q;
    q.setRPY(0, M_PI / 2, 0);
    q.normalize();
    q_msg_ = tf2::toMsg(q);
  }

  void logBasicInfo() {
    RCLCPP_INFO(this->get_logger(), "Planning frame: %s", move_group_interface_->getPlanningFrame().c_str());
    RCLCPP_INFO(this->get_logger(), "End effector link: %s", move_group_interface_->getEndEffectorLink().c_str());
    RCLCPP_INFO(this->get_logger(), "Available Planning Groups:");
    for (const auto &group : move_group_interface_->getJointModelGroupNames()) {
      std::cout << "  - " << group << std::endl;
    }
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

  void drawCircle() {
    visual_tools_->trigger();
    for (int i = 0; i <= num_points_; ++i) {
      auto pose = calculatePose(i % num_points_);
      move_group_interface_->setPoseTarget(pose);

      if (i == 0) {
        move_group_interface_->clearPathConstraints();
      } else {
        move_group_interface_->setPathConstraints(orientation_constraints_);
      }

      RCLCPP_INFO(this->get_logger(), "Planning to pose %d: x=%f, y=%f, z=%f", i, pose.position.x, pose.position.y, pose.position.z);
      drawTitle("Planning_position_" + std::to_string(i));
      visual_tools_->trigger();

      auto [success, plan] = createPlan();
      if (success) {
        visual_tools_->publishTrajectoryLine(plan.trajectory_, move_group_interface_->getRobotModel()->getJointModelGroup(PLANNING_GROUP));
        visual_tools_->publishAxisLabeled(pose, "position_" + std::to_string(i));
        drawTitle("Executing_plan_" + std::to_string(i));
        visual_tools_->trigger();
        move_group_interface_->execute(plan);
      } else {
        drawTitle("Planning_Failed");
        visual_tools_->trigger();
        RCLCPP_ERROR(this->get_logger(), "Planning failed for point %d", i);
      }
    }
  }
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<DrawCircleNode>();
  node->initialize();  // <-- shared_from_this() is safe here
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
