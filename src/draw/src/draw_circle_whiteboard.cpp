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
//head_2_joint = -0.23
//torso_joint = 0.25

class DrawCircleNode : public rclcpp::Node {
public:
  DrawCircleNode(): Node(NODE_NAME, rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) {
    // Set use_sim_time = true
    //this->declare_parameter("use_sim_time", true);
    drawing_positions_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "/drawing_positions", 10, std::bind(&DrawCircleNode::drawCircle, this, std::placeholders::_1));
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
  }

private:
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr drawing_positions_sub_;
  std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;

  void drawTitle(const std::string &text) {
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().x() = 1.0;
    text_pose.translation().z() = 1.0;
    visual_tools_->publishText(text_pose, text, rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  }

  void drawCircle(const geometry_msgs::msg::PoseArray &posearray) {
    std::vector<geometry_msgs::msg::Pose> waypoints;
    for (size_t i = 0; i < posearray.poses.size(); ++i) {
        auto pose = posearray.poses.at(i);
        waypoints.push_back(pose);
    }

    waypoints.push_back(waypoints.front());

    drawTitle("Plan_Cartesian_Path");
    visual_tools_->publishPath(waypoints, rviz_visual_tools::LIME_GREEN, rviz_visual_tools::SMALL);
    for (std::size_t i = 0; i < waypoints.size(); ++i)
        //visual_tools_->publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rviz_visual_tools::SMALL);
        visual_tools_->trigger();
        //visual_tools_->prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    move_group_interface_->setPlanningTime(30.0);
    move_group_interface_->setNumPlanningAttempts(100);

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = 0.0;
    int attempt = 0;
    int max_attempts = 100;
    double percentage = 0.9;

    while(fraction < percentage && attempt < max_attempts) {
      fraction = move_group_interface_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
      RCLCPP_INFO(this->get_logger(), "Visualizing plan %i (%.2f%% achieved)", attempt, fraction * 100.0);
      attempt++;
    }

    if (fraction >= percentage) {
        RCLCPP_INFO(this->get_logger(), "Successfully planned Cartesian path with %.2f%% coverage.", fraction * 100.0);

        drawTitle("Execute_Cartesian_Path");
        visual_tools_->trigger();

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;
        move_group_interface_->execute(plan);

        RCLCPP_INFO(this->get_logger(), "Circle drawn successfully.");

        rclcpp::shutdown();
        return;
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to plan Cartesian path with sufficient coverage after %i attempts.", attempt);
    }
  }
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<DrawCircleNode>();
  node->initialize();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
