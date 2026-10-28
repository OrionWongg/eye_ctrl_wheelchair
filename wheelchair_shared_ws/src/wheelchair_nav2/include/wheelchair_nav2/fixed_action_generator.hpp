#pragma once
#include <string>
#include <vector>
#include "dwb_core/trajectory_generator.hpp"
namespace wheelchair_nav2 {
class FixedActionGenerator : public dwb_core::TrajectoryGenerator {
 public:
  void initialize(const nav2_util::LifecycleNode::SharedPtr &,const std::string &) override;
  void startNewIteration(const nav_2d_msgs::msg::Twist2D &) override;
  bool hasMoreTwists() override;
  nav_2d_msgs::msg::Twist2D nextTwist() override;
  dwb_msgs::msg::Trajectory2D generateTrajectory(const geometry_msgs::msg::Pose2D &,const nav_2d_msgs::msg::Twist2D &,const nav_2d_msgs::msg::Twist2D &) override;
  void setSpeedLimit(const double &,const bool &) override;
 private:
  double forward_,left_,right_,linear_accel_,angular_accel_,linear_brake_,angular_brake_,step_,forward_hold_,turn_hold_,startup_,dwell_;
  std::vector<nav_2d_msgs::msg::Twist2D> commands_;
  size_t next_=0;
  bool limited_=false;
};
}
