#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <vector>
#include <unistd.h>
#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "dwb_core/trajectory_generator.hpp"
#include "pluginlib/class_loader.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "wheelchair_interfaces/msg/input_health.hpp"
#include "wheelchair_interfaces/srv/compute_discrete_path.hpp"
using Pose=geometry_msgs::msg::Pose2D;
using Clock=std::chrono::steady_clock;
static double wrap(double a) {return std::atan2(std::sin(a),std::cos(a));}
static double distance(const Pose &a,const Pose &b) {return std::hypot(a.x-b.x,a.y-b.y);}
static nav2_costmap_2d::Footprint rounded_padding(const nav2_costmap_2d::Footprint &body,double padding) {
  std::vector<geometry_msgs::msg::Point> samples;
  const double radius=padding/std::cos(M_PI/32);
  for(const auto &vertex:body) for(int i=0;i<32;++i) {auto p=vertex;p.x+=radius*std::cos(i*M_PI/16);p.y+=radius*std::sin(i*M_PI/16);samples.push_back(p);}
  std::sort(samples.begin(),samples.end(),[](const auto &a,const auto &b){return a.x<b.x || (a.x==b.x && a.y<b.y);});
  auto cross=[](const auto &a,const auto &b,const auto &c){return (b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x);};
  std::vector<geometry_msgs::msg::Point> hull;
  for(const auto &p:samples) {while(hull.size()>1 && cross(hull[hull.size()-2],hull.back(),p)<=0)hull.pop_back();hull.push_back(p);}
  const auto lower=hull.size();
  for(int i=int(samples.size())-2;i>=0;--i) {while(hull.size()>lower && cross(hull[hull.size()-2],hull.back(),samples[i])<=0)hull.pop_back();hull.push_back(samples[i]);}
  hull.pop_back();return hull;
}
class DiscretePlanner : public rclcpp::Node {
 public:
  explicit DiscretePlanner(const nav2_util::LifecycleNode::SharedPtr &context):Node("discrete_path_planner"),
      loader_("dwb_core","dwb_core::TrajectoryGenerator"),context_(context) {
    auto parameter=[&](const std::string &key,double value) {return declare_parameter<double>(key,value);};
    forward_=parameter("forward_speed",.372);left_=parameter("left_speed",.389);right_=parameter("right_speed",.403);
    const std::vector<std::pair<std::string,double>> model={{"forward_speed",forward_},{"left_speed",left_},{"right_speed",right_},
      {"linear_accel",parameter("linear_accel",.6)},{"angular_accel",parameter("angular_accel",.8)},
      {"linear_brake",parameter("linear_brake",.5)},{"angular_brake",parameter("angular_brake",.7)},
      {"startup_delay",parameter("startup_delay",.12)},{"stop_dwell",parameter("stop_dwell",.15)}};
    for(auto &p:model) context_->declare_parameter("fixed."+p.first,p.second);
    generator_=loader_.createSharedInstance("wheelchair_nav2::FixedActionGenerator");generator_->initialize(context_,"fixed");
    // The action model is SE(2)-invariant. Reuse its stopped-state rollouts,
    // transforming them at each lattice node instead of reintegrating them.
    nav_2d_msgs::msg::Twist2D zero;Pose origin;generator_->startNewIteration(zero);
    while(generator_->hasMoreTwists()) {
      auto command=generator_->nextTwist();auto trajectory=generator_->generateTrajectory(origin,zero,command);
      std::vector<Pose> sampled;std::vector<builtin_interfaces::msg::Duration> offsets;
      for(size_t i=0;i<trajectory.poses.size();++i) {
        const auto &p=trajectory.poses[i];
        if(sampled.empty() || distance(sampled.back(),p)>=.012 || std::abs(p.theta-sampled.back().theta)>=.015 || i+1==trajectory.poses.size()) {sampled.push_back(p);offsets.push_back(trajectory.time_offsets[i]);}
      }
      trajectory.poses=sampled;trajectory.time_offsets=offsets;primitives_.push_back(trajectory);
    }
    auto points=declare_parameter<std::vector<double>>("footprint",{-.45,-.30,.55,-.30,.55,.30,-.45,.30});
    if(points.size()<6 || points.size()%2) throw std::invalid_argument("Invalid footprint");
    double radius=0;
    for(size_t i=0;i<points.size();i+=2) {geometry_msgs::msg::Point p;p.x=points[i];p.y=points[i+1];footprint_.push_back(p);radius=std::max(radius,std::hypot(p.x,p.y));}
    double padding=parameter("margin",.06)+parameter("model_error",.025)+(forward_+radius*std::max(left_,right_))*parameter("step",.025)/2;
    comfort_footprint_=rounded_padding(footprint_,padding+parameter("free_space_clearance",.08));
    footprint_=rounded_padding(footprint_,padding);
    timeout_=parameter("search_time_limit",.25);map_timeout_=parameter("costmap_timeout",.5);
    obstacle_weight_=parameter("obstacle_cost_weight",1.0);
    heuristic_weight_=parameter("heuristic_weight",4.0);
    max_expansions_=declare_parameter<int>("max_search_expansions",12000);
    session_=std::to_string(getpid())+"-"+std::to_string(Clock::now().time_since_epoch().count());
    grid_sub_=create_subscription<nav_msgs::msg::OccupancyGrid>("/wheelchair/local_costmap/costmap",rclcpp::QoS(1).transient_local(),
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr m) {
        if(m->header.frame_id!="odom" || m->info.width==0 || m->info.height==0 ||
           size_t(m->info.width)*m->info.height>2000000 || m->data.size()!=size_t(m->info.width)*m->info.height ||
           !std::isfinite(m->info.resolution) || m->info.resolution<=0 || m->info.resolution>.2 ||
           !std::isfinite(m->info.origin.position.x) || !std::isfinite(m->info.origin.position.y) ||
           std::abs(m->info.origin.orientation.z)>1e-6 || std::abs(m->info.origin.orientation.w-1)>1e-6)return;
        map_=std::make_unique<nav2_costmap_2d::Costmap2D>(m->info.width,m->info.height,m->info.resolution,
          m->info.origin.position.x,m->info.origin.position.y,nav2_costmap_2d::NO_INFORMATION);
        for(size_t i=0;i<m->data.size();++i) {
          int v=m->data[i];map_->getCharMap()[i]=v<0?255:v>=100?254:v==99?253:v==0?0:std::min(252,1+(v-1)*251/97);
        }
        map_stamp_=rclcpp::Time(m->header.stamp);map_received_=Clock::now();
      });
    health_=create_publisher<wheelchair_interfaces::msg::InputHealth>("/wheelchair/path_planner_health",1);
    timer_=create_wall_timer(std::chrono::milliseconds(100),[this]() {
      wheelchair_interfaces::msg::InputHealth h;h.header.stamp=now();h.protocol_version=1;h.source_session_id=session_;
      h.sequence=h.sample_sequence=++sequence_;h.source_valid=fresh();h.ttl_ms=500;
      h.source_age_ms=map_?std::max(0.,(now()-map_stamp_).seconds())*1000:10000.;health_->publish(h);
    });
    service_=create_service<wheelchair_interfaces::srv::ComputeDiscretePath>("/wheelchair/compute_discrete_path",
      [this](const std::shared_ptr<wheelchair_interfaces::srv::ComputeDiscretePath::Request> request,
             std::shared_ptr<wheelchair_interfaces::srv::ComputeDiscretePath::Response> response) {compute(*request,*response);});
    RCLCPP_INFO(get_logger(),"Nav2 costmap + loaded DWB FixedActionGenerator; no chassis output");
  }
 private:
  bool fresh() const {return map_ && std::chrono::duration<double>(Clock::now()-map_received_).count()<map_timeout_ &&
      (now()-map_stamp_).seconds()>=-.05 && (now()-map_stamp_).seconds()<map_timeout_;}
  bool valid(const Pose &p,const Pose &origin,double band) {
    if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.theta))return false;
    if(band>0 && std::abs(-(p.x-origin.x)*std::sin(origin.theta)+(p.y-origin.y)*std::cos(origin.theta))>band)return false;
    unsigned int x,y;if(!map_->worldToMap(p.x,p.y,x,y)||map_->getCost(x,y)>=254)return false;
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(map_.get());
    double cost=checker.footprintCostAtPose(p.x,p.y,p.theta,footprint_);
    return cost>=0 && cost<254;
  }
  bool connect(const Pose &start,const Pose &goal,const Pose &origin,double band,std::vector<Pose> &out,int half_turn_sign=1) {
    out={start};Pose cur=start;
    auto rotate=[&](double target) {
      double delta=wrap(target-cur.theta);
      if(std::abs(std::abs(delta)-M_PI)<1e-6)delta=half_turn_sign*M_PI;
      const int n=std::max(1,int(std::ceil(std::abs(delta)/.025)));const double original=cur.theta;
      for(int i=1;i<=n;++i) {cur.theta=original+delta*i/n;if(!valid(cur,origin,band))return false;out.push_back(cur);}
      return true;
    };
    if(distance(start,goal)>1e-5) {
      double heading=std::atan2(goal.y-start.y,goal.x-start.x);
      if(!rotate(heading))return false;
      const int n=std::max(1,int(std::ceil(distance(start,goal)/.025)));
      for(int i=1;i<=n;++i) {cur.x=start.x+(goal.x-start.x)*i/n;cur.y=start.y+(goal.y-start.y)*i/n;
        if(!valid(cur,origin,band))return false;out.push_back(cur);}
    }
    return rotate(goal.theta);
  }
  std::string key(const Pose &p) {
    return std::to_string(std::lround(p.x/.10))+":"+std::to_string(std::lround(p.y/.10))+":"+
      std::to_string((int(std::lround(wrap(p.theta)/(M_PI/8)))+16)%16);
  }
  bool search(const Pose &start,const Pose &goal,const Pose &origin,double band,double max_length,
              std::vector<Pose> &result,uint32_t &expansions,Clock::time_point deadline,bool allow_detour) {
    if(!valid(start,origin,band)||!valid(goal,origin,band))return false;
    if(connect(start,goal,origin,band,result) || connect(start,goal,origin,band,result,-1))return distance(start,goal)<=max_length;
    // A locked doorway is a route constraint, not merely an endpoint. Searching
    // around another doorway would silently change the user's selected entry.
    if(!allow_detour)return false;
    struct Restore {nav2_costmap_2d::Footprint &value;nav2_costmap_2d::Footprint old;~Restore(){value=old;}} restore{footprint_,footprint_};
    // A free-space detour reserves tracking error around bends. Straight
    // corridors/doorways still use the exact hard footprint check above.
    if(comfort_search_) {
      footprint_=comfort_footprint_;
      if(!valid(start,origin,band)||!valid(goal,origin,band))footprint_=restore.old;
    }
    struct Vertex {Pose pose;double cost,length;int parent,action;std::vector<Pose> edge;};
    struct Candidate {double cost;int index;bool operator<(const Candidate &other)const{return cost>other.cost;}};
    std::vector<Vertex> nodes={{start,0,0,-1,0,{start}}};std::priority_queue<Candidate> queue;
    std::unordered_map<std::string,double> best;queue.push({heuristic_weight_*distance(start,goal)/forward_,0});best[key(start)]=0;
    while(!queue.empty() && expansions<unsigned(max_expansions_) && Clock::now()<deadline) {
      const int index=queue.top().index;queue.pop();const auto current=nodes[index];++expansions;
      if(best[key(current.pose)]+1e-6<current.cost)continue;
      std::vector<Pose> tail;
      if(distance(current.pose,goal)<.55 && current.length+distance(current.pose,goal)<=max_length &&
         (connect(current.pose,goal,origin,band,tail)||connect(current.pose,goal,origin,band,tail,-1))) {
        std::vector<int> lineage;for(int i=index;i>=0;i=nodes[i].parent)lineage.push_back(i);
        std::reverse(lineage.begin(),lineage.end());result.clear();
        for(auto i:lineage)result.insert(result.end(),nodes[i].edge.begin(),nodes[i].edge.end());
        result.insert(result.end(),tail.begin(),tail.end());return true;
      }
      for(size_t id=1;id<primitives_.size();++id) {
        const auto &primitive=primitives_[id];std::vector<Pose> trajectory;
        const double cosine=std::cos(current.pose.theta),sine=std::sin(current.pose.theta);
        for(const auto &p:primitive.poses) {Pose q;q.x=current.pose.x+cosine*p.x-sine*p.y;q.y=current.pose.y+sine*p.x+cosine*p.y;q.theta=current.pose.theta+p.theta;trajectory.push_back(q);}
        const Pose next=trajectory.back();double length=current.length+distance(current.pose,next);
        if(length+distance(next,goal)>max_length)continue;
        bool free=true;double cost=0;
        for(const auto &pose:trajectory) {if(!valid(pose,origin,band)){free=false;break;}
          unsigned int x,y;map_->worldToMap(pose.x,pose.y,x,y);cost+=map_->getCost(x,y)/252.;}
        if(!free)continue;
        const double duration=rclcpp::Duration(primitive.time_offsets.back()).seconds();
        double total=current.cost+duration+(current.action && current.action!=id?.25:0)+
          obstacle_weight_*duration*cost/trajectory.size();
        const auto k=key(next);if(best.count(k)&&best[k]<=total)continue;
        best[k]=total;nodes.push_back({next,total,length,index,int(id),trajectory});
        queue.push({total+heuristic_weight_*distance(next,goal)/forward_,int(nodes.size()-1)});
      }
    }
    return false;
  }
  void compute(const wheelchair_interfaces::srv::ComputeDiscretePath::Request &r,
               wheelchair_interfaces::srv::ComputeDiscretePath::Response &out) {
    const auto begin=Clock::now();out.header.stamp=now();out.header.frame_id="odom";out.task_id=r.task_id;out.request_id=r.request_id;
    out.reason="INVALID_REQUEST";
    if(r.header.frame_id!="odom"||r.task_id.empty()||r.goals.empty()||r.goals.size()>8||!std::isfinite(r.max_path_length)||r.max_path_length<=0||r.max_path_length>12||!std::isfinite(r.corridor_half_width)||r.corridor_half_width<0)return;
    auto finite=[](const Pose &p){return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.theta);};
    if(!finite(r.start)||!finite(r.corridor_origin)||!std::all_of(r.goals.begin(),r.goals.end(),finite))return;
    if(!fresh()){out.reason="COSTMAP_STALE";return;}
    out.costmap_stamp=map_stamp_;
    const double age=(now()-rclcpp::Time(r.header.stamp)).seconds();if(age<-.05||age>1.){out.reason="STALE_REQUEST";return;}
    auto deadline=begin+std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(timeout_));
    comfort_search_=r.goals.size()==1 && distance(r.start,r.goals.back())>.2 && r.corridor_half_width==0;
    Pose start=r.start;std::vector<Pose> poses;double remaining=r.max_path_length;
    for(const auto &goal:r.goals) {
      std::vector<Pose> part;
      auto outside=[&](const Pose &p){const auto &o=r.corridor_origin;return r.corridor_half_width>0 &&
        std::abs(-(p.x-o.x)*std::sin(o.theta)+(p.y-o.y)*std::cos(o.theta))>r.corridor_half_width;};
      if(outside(start))out.reason="START_OUTSIDE_CORRIDOR";
      else if(outside(goal))out.reason="GOAL_OUTSIDE_CORRIDOR";
      else if(!valid(start,r.corridor_origin,0))out.reason="START_COLLISION";
      else if(!valid(goal,r.corridor_origin,0))out.reason="GOAL_COLLISION";
      else out.reason="";
      if(!out.reason.empty()) {out.compute_ms=std::chrono::duration<double,std::milli>(Clock::now()-begin).count();return;}
      if(!search(start,goal,r.corridor_origin,r.corridor_half_width,remaining,part,out.expansions,deadline,r.allow_detour)) {
        out.reason=Clock::now()>=deadline?"SEARCH_TIMEOUT":out.expansions>=unsigned(max_expansions_)?"SEARCH_EXPANSION_LIMIT":
          !r.allow_detour?"LOCKED_ROUTE_BLOCKED":"NO_EXECUTABLE_PATH";out.compute_ms=std::chrono::duration<double,std::milli>(Clock::now()-begin).count();return;
      }
      for(size_t i=1;i<part.size();++i)remaining-=distance(part[i-1],part[i]);
      if(remaining<-.001){out.reason="PATH_DISTANCE_LIMIT";return;}
      poses.insert(poses.end(),part.begin(),part.end());start=goal;
    }
    out.path.header=out.header;
    for(const auto &p:poses) {geometry_msgs::msg::PoseStamped pose;pose.header=out.header;pose.pose.position.x=p.x;pose.pose.position.y=p.y;
      pose.pose.orientation.z=std::sin(p.theta/2);pose.pose.orientation.w=std::cos(p.theta/2);out.path.poses.push_back(pose);}
    out.success=true;out.reason="PLANNED";out.compute_ms=std::chrono::duration<double,std::milli>(Clock::now()-begin).count();
  }
  pluginlib::ClassLoader<dwb_core::TrajectoryGenerator> loader_;
  nav2_util::LifecycleNode::SharedPtr context_;
  std::shared_ptr<dwb_core::TrajectoryGenerator> generator_;
  std::vector<dwb_msgs::msg::Trajectory2D> primitives_;
  std::unique_ptr<nav2_costmap_2d::Costmap2D> map_;
  nav2_costmap_2d::Footprint footprint_;
  nav2_costmap_2d::Footprint comfort_footprint_;
  bool comfort_search_=false;
  rclcpp::Time map_stamp_{0,0,RCL_ROS_TIME};Clock::time_point map_received_;
  double forward_,left_,right_,timeout_,map_timeout_,obstacle_weight_,heuristic_weight_;int max_expansions_;
  std::string session_;uint64_t sequence_=0;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;
  rclcpp::Publisher<wheelchair_interfaces::msg::InputHealth>::SharedPtr health_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<wheelchair_interfaces::srv::ComputeDiscretePath>::SharedPtr service_;
};
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  auto context=std::make_shared<nav2_util::LifecycleNode>("fixed_action_model");
  auto node=std::make_shared<DiscretePlanner>(context);
  rclcpp::spin(node);rclcpp::shutdown();return 0;
}
