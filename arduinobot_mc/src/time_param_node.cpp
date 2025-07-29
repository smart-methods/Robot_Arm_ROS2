#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>

class TimeParamNode : public rclcpp::Node
{
public:
  TimeParamNode() : Node("time_param_node")
  {
    // Delay initialization to avoid shared_from_this() in constructor
  }

  void initialize()
  {
    // Disambiguate shared_from_this by static_cast to rclcpp::Node
    auto node_shared = std::static_pointer_cast<rclcpp::Node>(shared_from_this());
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_shared, "arm");
    planAndExecute();
  }

private:
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

  void planAndExecute()
  {
    move_group_->setNamedTarget("home");

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!success || plan.trajectory_.joint_trajectory.points.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "Planning failed");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Planning successful, points: %zu", plan.trajectory_.joint_trajectory.points.size());

    robot_trajectory::RobotTrajectory rt(move_group_->getRobotModel(), move_group_->getName());
    rt.setRobotTrajectoryMsg(*move_group_->getCurrentState(), plan.trajectory_);

    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    bool time_param_success = iptp.computeTimeStamps(rt);
    if (!time_param_success)
    {
      RCLCPP_ERROR(this->get_logger(), "Time parameterization failed");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Time parameterization succeeded");

    moveit_msgs::msg::RobotTrajectory robot_trajectory_msg;
    rt.getRobotTrajectoryMsg(robot_trajectory_msg);

    plan.trajectory_ = robot_trajectory_msg;

    auto exec_status = move_group_->execute(plan);
    if (exec_status != moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "Execution failed");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Execution succeeded");
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<TimeParamNode>();
  node->initialize();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
