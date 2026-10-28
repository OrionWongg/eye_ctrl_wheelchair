#include "wheelchair_nav2/fixed_action_generator.hpp"
#include <cmath>
#include <stdexcept>
#include "pluginlib/class_list_macros.hpp"
namespace wheelchair_nav2 {
void FixedActionGenerator::initialize(const nav2_util::LifecycleNode::SharedPtr &node,const std::string &name) {
  auto param=[&](const std::string &key,double value) {
    const auto full=name+"."+key;
    if (!node->has_parameter(full)) node->declare_parameter(full,value);
    const double result=node->get_parameter(full).as_double();
    if (!std::isfinite(result) || result<=0) throw std::invalid_argument(full);
    return result;
  };
  forward_=param("forward_speed",.372);left_=param("left_speed",.389);right_=param("right_speed",.403);
  linear_accel_=param("linear_accel",.6);angular_accel_=param("angular_accel",.8);
  linear_brake_=param("linear_brake",.5);angular_brake_=param("angular_brake",.7);
  forward_hold_=param("forward_prediction_time",.60);turn_hold_=param("turn_prediction_time",.30);
  step_=param("integration_step",.02);startup_=param("startup_delay",.12);dwell_=param("stop_dwell",.15);
  nav_2d_msgs::msg::Twist2D c;commands_.push_back(c);
  c.x=forward_;commands_.push_back(c);c.x=0;c.theta=left_;commands_.push_back(c);c.theta=-right_;commands_.push_back(c);
}
void FixedActionGenerator::startNewIteration(const nav_2d_msgs::msg::Twist2D &) {next_=0;}
bool FixedActionGenerator::hasMoreTwists() {return next_<(limited_?1:commands_.size());}
nav_2d_msgs::msg::Twist2D FixedActionGenerator::nextTwist() {return commands_.at(next_++);}
void FixedActionGenerator::setSpeedLimit(const double &limit,const bool &percentage) {
  // Fixed gears cannot honor arbitrary slowdown requests by changing velocity.
  limited_=limit>0 && (percentage?limit<100:limit<forward_);
}
dwb_msgs::msg::Trajectory2D FixedActionGenerator::generateTrajectory(
    const geometry_msgs::msg::Pose2D &start,const nav_2d_msgs::msg::Twist2D &initial,const nav_2d_msgs::msg::Twist2D &command) {
  bool legal=false;
  for(const auto &c:commands_) if(std::abs(command.x-c.x)<1e-9 && std::abs(command.theta-c.theta)<1e-9 && std::abs(command.y)<1e-9) legal=true;
  if(!legal) throw std::invalid_argument("Unsupported fixed action");
  dwb_msgs::msg::Trajectory2D out;out.velocity=command;
  auto pose=start;double v=initial.x,w=initial.theta,t=0;
  auto add=[&]() {out.poses.push_back(pose);out.time_offsets.push_back(rclcpp::Duration::from_seconds(t));};
  add();
  auto toward=[](double value,double target,double amount) {return value<target?std::min(target,value+amount):std::max(target,value-amount);};
  auto phase=[&](double seconds,double tv,double tw,bool braking) {
    const int n=std::max(1,int(std::ceil(seconds/step_)));const double dt=seconds/n;
    for(int i=0;i<n;++i) {
      const double nv=toward(v,tv,(braking?linear_brake_:linear_accel_)*dt);
      const double nw=toward(w,tw,(braking?angular_brake_:angular_accel_)*dt);
      const double mid=pose.theta+(w+nw)*dt/4;
      pose.x+=(v+nv)*dt/2*std::cos(mid);pose.y+=(v+nv)*dt/2*std::sin(mid);pose.theta+=(w+nw)*dt/2;
      v=nv;w=nw;t+=dt;add();
    }
  };
  // Search nodes are stopped poses; this also handles a measured nonzero start.
  if(std::abs(v)>1e-6 || std::abs(w)>1e-6) phase(std::max(std::abs(v)/linear_brake_,std::abs(w)/angular_brake_),0,0,true);
  if(command.x==0 && command.theta==0) return out;
  phase(dwell_+startup_,0,0,true);
  const bool drive=command.x!=0;
  // These times bound the planning rollout only; they never stop the actuator.
  // Predict a held direction, rather than solve for an ideal distance/angle.
  phase(drive?forward_hold_:turn_hold_,command.x,command.theta,false);
  phase(std::max(std::abs(v)/linear_brake_,std::abs(w)/angular_brake_),0,0,true);
  return out;
}
}
PLUGINLIB_EXPORT_CLASS(wheelchair_nav2::FixedActionGenerator,dwb_core::TrajectoryGenerator)
