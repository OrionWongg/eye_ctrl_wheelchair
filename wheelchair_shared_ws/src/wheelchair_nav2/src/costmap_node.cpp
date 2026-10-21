// SPDX-License-Identifier: BSD-3-Clause
// Configuration adapted from Nav2 1.1.20 costmap_2d_ros.cpp.
// Copyright (c) 2008, 2013, Willow Garage, Inc. All rights reserved.
// See LICENSE.costmap for the retained upstream license.
#include "rclcpp/rclcpp.hpp"
#include <csignal>
#include <thread>
#include <chrono>
#include <set>
#include "lifecycle_msgs/msg/state.hpp"
static volatile std::sig_atomic_t stopping=0;
static void stop_signal(int) {stopping=1;}
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/create_timer_ros.h"
class StandaloneCostmap : public nav2_costmap_2d::Costmap2DROS {
 public:
  explicit StandaloneCostmap(pluginlib::ClassLoader<nav2_costmap_2d::Layer> &libraries):
      Costmap2DROS("local_costmap","/wheelchair","local_costmap"),libraries_(libraries) {
    // This constructor normally creates a lifecycle follower owned by a
    // controller server. Here the costmap has its own lifecycle manager.
    is_lifecycle_follower_=false;
  }
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
    // Configuration follows Nav2 1.1.20 Costmap2DROS::on_configure, except TF
    // subscriptions share the sensor/timer executor. Humble's separate TF
    // thread can deadlock Buffer::waitForTransform against setTransform.
    RCLCPP_INFO(get_logger(),"Configuring serialized TF and sensor callbacks");
    getParameters();
    callback_group_=create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive,false);
    layered_costmap_=std::make_unique<nav2_costmap_2d::LayeredCostmap>(
        global_frame_,rolling_window_,track_unknown_space_);
    if(!layered_costmap_->isSizeLocked()) {
      layered_costmap_->resizeMap(static_cast<unsigned int>(map_width_meters_/resolution_),
          static_cast<unsigned int>(map_height_meters_/resolution_),resolution_,origin_x_,origin_y_);
    }
    tf_buffer_=std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_buffer_->setCreateTimerInterface(std::make_shared<tf2_ros::CreateTimerROS>(
        get_node_base_interface(),get_node_timers_interface(),callback_group_));
    rclcpp::SubscriptionOptions options;options.callback_group=callback_group_;
    tf_listener_=std::make_shared<tf2_ros::TransformListener>(*tf_buffer_,shared_from_this(),false,
        tf2_ros::DynamicListenerQoS(),tf2_ros::StaticListenerQoS(),options,options);
    // A dedicated executor services TF while the map-update thread waits.
    tf_buffer_->setUsingDedicatedThread(true);
    auto add_layers=[&](const auto &names,const auto &types,bool filter) {
      for(size_t i=0;i<names.size();++i) {
        pin(types[i]);
        auto plugin=plugin_loader_.createSharedInstance(types[i]);
        std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*layered_costmap_->getCostmap()->getMutex());
        if(filter)layered_costmap_->addFilter(plugin);else layered_costmap_->addPlugin(plugin);
        plugin->initialize(layered_costmap_.get(),names[i],tf_buffer_.get(),shared_from_this(),callback_group_);
      }
    };
    add_layers(plugin_names_,plugin_types_,false);
    add_layers(filter_names_,filter_types_,true);
    footprint_sub_=create_subscription<geometry_msgs::msg::Polygon>("footprint",rclcpp::SystemDefaultsQoS(),
        [this](geometry_msgs::msg::Polygon::SharedPtr polygon){setRobotFootprintPolygon(polygon);});
    footprint_pub_=create_publisher<geometry_msgs::msg::PolygonStamped>("published_footprint",rclcpp::SystemDefaultsQoS());
    costmap_publisher_=std::make_unique<nav2_costmap_2d::Costmap2DPublisher>(shared_from_this(),
        layered_costmap_->getCostmap(),global_frame_,"costmap",always_send_full_costmap_);
    if(use_radius_)setRobotFootprint(nav2_costmap_2d::makeFootprintFromRadius(robot_radius_));
    else {
      std::vector<geometry_msgs::msg::Point> footprint;
      nav2_costmap_2d::makeFootprintFromString(footprint_,footprint);setRobotFootprint(footprint);
    }
    clear_costmap_service_=std::make_unique<nav2_costmap_2d::ClearCostmapService>(shared_from_this(),*this);
    executor_=std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_callback_group(callback_group_,get_node_base_interface());
    executor_thread_=std::make_unique<nav2_util::NodeThread>(executor_);
    return nav2_util::CallbackReturn::SUCCESS;
  }
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override {
    if(!executor_thread_)executor_thread_=std::make_unique<nav2_util::NodeThread>(executor_);
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(10);
    while(!tf_buffer_->canTransform(global_frame_,robot_base_frame_,tf2::TimePointZero)) {
      if(stopping || !rclcpp::ok() || std::chrono::steady_clock::now()>=deadline) {
        RCLCPP_WARN(get_logger(),"Activation cancelled: robot transform unavailable");
        return nav2_util::CallbackReturn::FAILURE;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    auto result=Costmap2DROS::on_activate(state);
    if(result==nav2_util::CallbackReturn::SUCCESS)createBond();
    return result;
  }
  void prepare_exit() {
      // Gazebo may stop /clock before this process. A TF timeout in the map
      // thread must still expire before on_deactivate joins that thread.
      // Remove the map parameter callback before changing clock mode: it locks
      // the same mutex held by an in-flight map update waiting for TF.
      dyn_params_handler.reset();
      set_parameter(rclcpp::Parameter("use_sim_time",false));
  }
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override {
    destroyBond();
    // Stop sensor callbacks before unsubscribing the layers. This also avoids
    // tearing down subscriptions while the private executor dispatches them.
    executor_thread_.reset();
    return Costmap2DROS::on_deactivate(state);
  }
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override {
    // Humble destroys layers and their TF buffer before joining this executor.
    // Drain callbacks first so queued scan/TF callbacks cannot use freed layers.
    executor_thread_.reset();
    auto result=Costmap2DROS::on_cleanup(state);
    // Callback groups retain weak subscription references whose control-block
    // destructors live in the layer libraries. Release them while plugin_loader_
    // still owns those libraries, rather than during base member destruction.
    executor_.reset();
    callback_group_.reset();
    return result;
  }
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override {
    const auto id=state.id();
    if(id==lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)on_deactivate(state);
    if(id==lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE ||
       id==lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)on_cleanup(state);
    return Costmap2DROS::on_shutdown(state);
  }
 private:
  void pin(const std::string &type) {
    if(pinned_.insert(type).second)libraries_.loadLibraryForClass(type);
  }
  pluginlib::ClassLoader<nav2_costmap_2d::Layer> &libraries_;
  std::set<std::string> pinned_;
};
int main(int argc,char **argv) {
  rclcpp::init(argc,argv,rclcpp::InitOptions(),rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT,stop_signal);std::signal(SIGTERM,stop_signal);
  // Layer subscriptions also leave weak references in NodeBase's default
  // callback group. Their libraries must outlive the ENTIRE node, including
  // base destructors, not just Costmap2DROS::on_cleanup.
  pluginlib::ClassLoader<nav2_costmap_2d::Layer> libraries("nav2_costmap_2d","nav2_costmap_2d::Layer");
  auto node=std::make_shared<StandaloneCostmap>(libraries);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  while(rclcpp::ok() && !stopping) {executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(5));}
  node->prepare_exit();
  // Humble applies clock-mode changes through a parameter EVENT subscription.
  // Keep dispatching until that event has actually disabled simulated time.
  const auto clock_deadline=std::chrono::steady_clock::now()+std::chrono::seconds(1);
  while(rclcpp::ok() && node->get_clock()->ros_time_is_active() && std::chrono::steady_clock::now()<clock_deadline) {
    executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  if(node->get_clock()->ros_time_is_active())rclcpp::shutdown();
  if(node->get_current_state().id()==lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)node->deactivate();
  if(node->get_current_state().id()==lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)node->cleanup();
  executor.remove_node(node->get_node_base_interface());node.reset();rclcpp::shutdown();return 0;
}
