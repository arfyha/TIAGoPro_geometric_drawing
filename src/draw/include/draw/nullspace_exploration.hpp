#ifndef NULLSPACE_EXPLORATION_HPP
#define NULLSPACE_EXPLORATION_HPP


#include "rclcpp/rclcpp.hpp"
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/kinematics_metrics/kinematics_metrics.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_state.hpp>
#include <moveit/robot_state/conversions.h>
#include <Eigen/SVD>
#include <std_msgs/msg/float64_multi_array.hpp>


Eigen::MatrixXd computeNullspace(const moveit::core::RobotState& robot_state, 
                                 const moveit::core::JointModelGroup* jmg);

Eigen::VectorXd recursiveNullspaceExploration(moveit::core::RobotState& robot_state, 
                                              const moveit::core::JointModelGroup* jmg, 
                                              std::shared_ptr<kinematics_metrics::KinematicsMetrics> kinematics_metrics,
                                              double prev_manipulability, Eigen::VectorXd prev_best_config);

Eigen::VectorXd gridSearch(moveit::core::RobotState& robot_state, 
                           const moveit::core::JointModelGroup* jmg, 
                           std::shared_ptr<kinematics_metrics::KinematicsMetrics> kinematics_metrics,
                           double best_manipulability, Eigen::VectorXd best_config, Eigen::MatrixXd nullspace);


class NullspaceExplorationNode : public rclcpp::Node
{
public:
    NullspaceExplorationNode();
    void initialize_move_group();
    void robot_state_callback(const moveit_msgs::msg::RobotState::SharedPtr msg);
    std::vector<double> explore(moveit::core::RobotState& current_state);

private:
    moveit::core::RobotModelPtr robot_model_;
    moveit::core::RobotStatePtr robot_state_;
    const moveit::core::JointModelGroup* jmg_;
    std::shared_ptr<kinematics_metrics::KinematicsMetrics> kinematics_metrics_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    rclcpp::Subscription<moveit_msgs::msg::RobotState>::SharedPtr subscription_;
};

#endif // NULLSPACE_EXPLORATION_HPP
