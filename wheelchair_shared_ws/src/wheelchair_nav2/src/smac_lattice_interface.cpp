// Workspace adapter for the unmodified Nav2 Humble SmacPlannerLattice plugin.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <unistd.h>
#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "pluginlib/class_loader.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "wheelchair_interfaces/msg/input_health.hpp"
#include "wheelchair_interfaces/srv/compute_discrete_path.hpp"
using Pose=geometry_msgs::msg::Pose2D;
using Clock=std::chrono::steady_clock;
static double wrap(double a){return std::atan2(std::sin(a),std::cos(a));}
static double distance(const Pose&a,const Pose&b){return std::hypot(a.x-b.x,a.y-b.y);}
static Pose pose(double x,double y,double h){Pose p;p.x=x;p.y=y;p.theta=h;return p;}
static double heading(const geometry_msgs::msg::Quaternion&q){return std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));}
static nav2_costmap_2d::Footprint rounded_padding(const nav2_costmap_2d::Footprint &body,double padding,int sides=32) {
  std::vector<geometry_msgs::msg::Point> samples;
  const double radius=padding/std::cos(M_PI/sides);
  for(const auto &vertex:body) for(int i=0;i<sides;++i) {auto p=vertex;p.x+=radius*std::cos(i*2*M_PI/sides);p.y+=radius*std::sin(i*2*M_PI/sides);samples.push_back(p);}
  std::sort(samples.begin(),samples.end(),[](const auto &a,const auto &b){return a.x<b.x || (a.x==b.x && a.y<b.y);});
  auto cross=[](const auto &a,const auto &b,const auto &c){return (b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x);};
  std::vector<geometry_msgs::msg::Point> hull;
  for(const auto &p:samples) {while(hull.size()>1 && cross(hull[hull.size()-2],hull.back(),p)<=0)hull.pop_back();hull.push_back(p);}
  const auto lower=hull.size();
  for(int i=int(samples.size())-2;i>=0;--i) {while(hull.size()>lower && cross(hull[hull.size()-2],hull.back(),samples[i])<=0)hull.pop_back();hull.push_back(samples[i]);}
  hull.pop_back();return hull;
}

// Snapshot-only wrapper: there is one real sensor costmap, owned by the existing
// wheelchair_costmap node. This object has no map-update or TF worker threads.
class SnapshotCostmap:public nav2_costmap_2d::Costmap2DROS {
 public:
  SnapshotCostmap():Costmap2DROS(rclcpp::NodeOptions().use_global_arguments(false).arguments(
      {"--ros-args","-r","__node:=smac_snapshot"})) {
    global_frame_="odom";robot_base_frame_="base_link";use_radius_=false;
    layered_costmap_=std::make_unique<nav2_costmap_2d::LayeredCostmap>("odom",false,true);
  }
  void footprint(const nav2_costmap_2d::Footprint &f){
    unpadded_footprint_=f;padded_footprint_=f;layered_costmap_->setFootprint(f);
  }
};

class SmacInterface:public rclcpp::Node {
 public:
  explicit SmacInterface(nav2_util::LifecycleNode::SharedPtr context,pluginlib::ClassLoader<nav2_core::GlobalPlanner>&libraries):Node("smac_lattice_interface"),
      loader_(libraries),context_(context) {
    auto parameter=[&](const std::string &key,double value){return declare_parameter<double>(key,value);};
    auto points=declare_parameter<std::vector<double>>("footprint",{-.45,-.30,.55,-.30,.55,.30,-.45,.30});
    if(points.size()<6||points.size()%2)throw std::invalid_argument("Invalid footprint");
    for(size_t i=0;i<points.size();i+=2){geometry_msgs::msg::Point p;p.x=points[i];p.y=points[i+1];if(!std::isfinite(p.x)||!std::isfinite(p.y))throw std::invalid_argument("Nonfinite footprint");body_.push_back(p);}
    double sign=0.;for(size_t i=0;i<body_.size();++i){const auto&a=body_[i];const auto&b=body_[(i+1)%body_.size()];const auto&c=body_[(i+2)%body_.size()];double z=(b.x-a.x)*(c.y-b.y)-(b.y-a.y)*(c.x-b.x);if(std::abs(z)>1e-10){if(sign*z<0)throw std::invalid_argument("Nonconvex footprint");sign=z;}}if(sign==0)throw std::invalid_argument("Degenerate footprint");
    double speed=parameter("forward_speed",.372),left=parameter("left_speed",.389),right=parameter("right_speed",.403);
    double margin=parameter("margin",.06);
    for(double v:{speed,left,right,margin})if(!std::isfinite(v)||v<=0)throw std::invalid_argument("Invalid safety parameter");
    double padding=margin;
    padding_=padding;footprint_=rounded_padding(body_,padding);snapshot_=std::make_shared<SnapshotCostmap>();double sampling=parameter("smac_search_sampling_margin",.05);sampling_=sampling;
    if(!std::isfinite(sampling)||sampling<0||sampling>.15)throw std::invalid_argument("Invalid search sampling margin");
    // Circumscribed octagonal padding remains conservative but reduces the
    // Smac grid checker from ~36 edges to ~12 per pose. Final distance checks
    // below still use the exact original polygon and Euclidean hard margin.
    snapshot_->footprint(rounded_padding(body_,padding+sampling,8));
    map_=snapshot_->getCostmap();timeout_=parameter("search_time_limit",.35);map_timeout_=parameter("costmap_timeout",.5);
    if(!std::isfinite(timeout_)||timeout_<=0||!std::isfinite(map_timeout_)||map_timeout_<=0)throw std::invalid_argument("Invalid timeout");
    session_=std::to_string(getpid())+"-"+std::to_string(Clock::now().time_since_epoch().count());
    loader_.loadLibraryForClass("nav2_smac_planner/SmacPlannerLattice");
    planner_=loader_.createSharedInstance("nav2_smac_planner/SmacPlannerLattice");
    raw_pub_=create_publisher<nav_msgs::msg::Path>("/wheelchair/smac/raw_plan",1);
    grid_sub_=create_subscription<nav_msgs::msg::OccupancyGrid>("/wheelchair/local_costmap/costmap",rclcpp::QoS(1).transient_local(),
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr m){grid(*m);});
    health_=create_publisher<wheelchair_interfaces::msg::InputHealth>("/wheelchair/smac/path_planner_health",1);
    timer_=create_wall_timer(std::chrono::milliseconds(100),[this](){
      wheelchair_interfaces::msg::InputHealth h;h.header.stamp=now();h.protocol_version=1;h.source_session_id=session_;
      h.sequence=h.sample_sequence=++sequence_;h.source_valid=fresh();h.ttl_ms=500;
      h.source_age_ms=valid_map_?std::max(0.,(now()-map_stamp_).seconds())*1000.:10000.;health_->publish(h);
    });
    service_=create_service<wheelchair_interfaces::srv::ComputeDiscretePath>("/wheelchair/smac/compute_discrete_path",
      [this](std::shared_ptr<wheelchair_interfaces::srv::ComputeDiscretePath::Request> r,
             std::shared_ptr<wheelchair_interfaces::srv::ComputeDiscretePath::Response> out){
        auto begin=Clock::now();out->header.stamp=now();out->header.frame_id="odom";out->task_id=r->task_id;out->request_id=r->request_id;
        try{compute(*r,*out,begin);}catch(const std::exception&e){out->success=false;out->reason="SMAC_EXCEPTION";RCLCPP_ERROR(get_logger(),"%s",e.what());}
        out->compute_ms=std::chrono::duration<double,std::milli>(Clock::now()-begin).count();
      });
    RCLCPP_INFO(get_logger(),"Loaded real Humble nav2_smac_planner/SmacPlannerLattice; no chassis publisher");
  }
  ~SmacInterface(){if(configured_){planner_->deactivate();planner_->cleanup();}planner_.reset();}
 private:
  void grid(const nav_msgs::msg::OccupancyGrid&m){
    valid_map_=false;
    const auto&p=m.info.origin.position;const auto&q=m.info.origin.orientation;
    if(m.header.frame_id!="odom"||!m.info.width||!m.info.height||size_t(m.info.width)*m.info.height>2000000||
       m.data.size()!=size_t(m.info.width)*m.info.height||std::abs(m.info.resolution-.05)>1e-6||
       !std::isfinite(p.x)||!std::isfinite(p.y)||std::abs(q.x)+std::abs(q.y)+std::abs(q.z)>1e-6||std::abs(q.w-1)>1e-6)return;
    if(!std::isfinite(q.x)||!std::isfinite(q.y)||!std::isfinite(q.z)||!std::isfinite(q.w))return;
    map_->resizeMap(m.info.width,m.info.height,m.info.resolution,p.x,p.y);
    for(size_t i=0;i<m.data.size();++i){int v=m.data[i];map_->getCharMap()[i]=v<0?255:v>=100?254:v==99?253:v==0?0:std::min(252,1+(v-1)*251/97);}
    map_stamp_=rclcpp::Time(m.header.stamp);map_received_=Clock::now();valid_map_=true;
    if(!configured_){planner_->configure(context_,"Lattice",nullptr,snapshot_);planner_->activate();configured_=true;}
  }
  bool fresh()const{return configured_&&valid_map_&&std::chrono::duration<double>(Clock::now()-map_received_).count()<map_timeout_&&
    (now()-map_stamp_).seconds()>=-.05&&(now()-map_stamp_).seconds()<map_timeout_;}
  bool expired()const{return Clock::now()>=deadline_;}
  static double segmentDistance2(const geometry_msgs::msg::Point&p,const geometry_msgs::msg::Point&a,const geometry_msgs::msg::Point&b){
    double dx=b.x-a.x,dy=b.y-a.y,l=dx*dx+dy*dy;
    double t=l>0?std::clamp(((p.x-a.x)*dx+(p.y-a.y)*dy)/l,0.,1.):0.;
    return std::pow(p.x-a.x-t*dx,2)+std::pow(p.y-a.y-t*dy,2);
  }
  static bool near(const nav2_costmap_2d::Footprint&a,const nav2_costmap_2d::Footprint&b,double pad){
    bool overlap=true;
    for(const auto*poly:{&a,&b})for(size_t i=0;i<poly->size();++i){
      const auto&u=(*poly)[i];const auto&v=(*poly)[(i+1)%poly->size()];double nx=u.y-v.y,ny=v.x-u.x;
      double amin=1e30,amax=-1e30,bmin=1e30,bmax=-1e30;
      for(const auto&p:a){double d=p.x*nx+p.y*ny;amin=std::min(amin,d);amax=std::max(amax,d);}
      for(const auto&p:b){double d=p.x*nx+p.y*ny;bmin=std::min(bmin,d);bmax=std::max(bmax,d);}
      if(amax<bmin||bmax<amin)overlap=false;
    }
    if(overlap)return true;
    for(const auto&p:a)for(size_t i=0;i<b.size();++i)if(segmentDistance2(p,b[i],b[(i+1)%b.size()])<=pad*pad)return true;
    for(const auto&p:b)for(size_t i=0;i<a.size();++i)if(segmentDistance2(p,a[i],a[(i+1)%a.size()])<=pad*pad)return true;
    return false;
  }
  bool valid(const Pose&p){
    if(expired())return false;
    if(band_>0&&std::abs(-(p.x-origin_.x)*std::sin(origin_.theta)+(p.y-origin_.y)*std::cos(origin_.theta))>band_)return false;
    nav2_costmap_2d::Footprint body;nav2_costmap_2d::transformFootprint(p.x,p.y,p.theta,body_,body);
    double x0=1e30,y0=1e30,x1=-1e30,y1=-1e30;
    for(const auto&v:body){x0=std::min(x0,v.x);y0=std::min(y0,v.y);x1=std::max(x1,v.x);y1=std::max(y1,v.y);}
    double ox=map_->getOriginX(),oy=map_->getOriginY(),r=map_->getResolution();
    if(x0-validation_padding_<=ox||y0-validation_padding_<=oy||x1+validation_padding_>=ox+map_->getSizeInCellsX()*r||y1+validation_padding_>=oy+map_->getSizeInCellsY()*r)return false;
    int ax=int(std::floor((x0-validation_padding_-ox)/r)),bx=int(std::floor((x1+validation_padding_-ox)/r));
    int ay=int(std::floor((y0-validation_padding_-oy)/r)),by=int(std::floor((y1+validation_padding_-oy)/r));
    for(int y=ay;y<=by;++y)for(int x=ax;x<=bx;++x){
      if(map_->getCost(x,y)<254)continue;
      nav2_costmap_2d::Footprint cell;
      for(const auto&xy:std::vector<std::pair<double,double>>{{ox+x*r,oy+y*r},{ox+(x+1)*r,oy+y*r},{ox+(x+1)*r,oy+(y+1)*r},{ox+x*r,oy+(y+1)*r}}){geometry_msgs::msg::Point v;v.x=xy.first;v.y=xy.second;cell.push_back(v);}
      if(near(body,cell,validation_padding_))return false;
    }
    return true;
  }
  bool connect(const Pose&start,const Pose&goal,std::vector<Pose>&out,int sign=1){
    out={start};Pose cur=start;if(!valid(cur)||!valid(goal))return false;
    auto rotate=[&](double target){double delta=wrap(target-cur.theta);if(std::abs(std::abs(delta)-M_PI)<1e-6)delta=sign*M_PI;
      int n=std::max(1,int(std::ceil(std::abs(delta)/.025)));double h=cur.theta;
      for(int i=1;i<=n;++i){cur.theta=h+delta*i/n;if(!valid(cur))return false;out.push_back(cur);}return true;};
    double length=distance(start,goal);
    if(length>1e-5){if(!rotate(std::atan2(goal.y-start.y,goal.x-start.x)))return false;
      int n=std::max(1,int(std::ceil(length/.025)));
      for(int i=1;i<=n;++i){cur.x=start.x+(goal.x-start.x)*i/n;cur.y=start.y+(goal.y-start.y)*i/n;if(!valid(cur))return false;out.push_back(cur);}}
    return rotate(goal.theta);
  }
  geometry_msgs::msg::PoseStamped stamped(const Pose&p){geometry_msgs::msg::PoseStamped m;m.header.frame_id="odom";m.header.stamp=now();
    m.pose.position.x=p.x;m.pose.position.y=p.y;m.pose.orientation.z=std::sin(p.theta/2);m.pose.orientation.w=std::cos(p.theta/2);return m;}
  bool lattice(const Pose&start,const Pose&goal,std::vector<Pose>&out){
    // Exact direct routes need no graph search. This also preserves narrow
    // straight passages whose geometry is finer than lattice quantization.
    if(connect(start,goal,out)||connect(start,goal,out,-1))return true;
    validation_padding_=padding_;
    snapshot_->footprint(rounded_padding(body_,validation_padding_+sampling_,8));
    used_smac_=true;
    auto raw=planner_->createPlan(stamped(start),stamped(goal));raw_pub_->publish(raw);
    if(raw.poses.empty()||raw.poses.size()>10000||expired())return false;
    std::vector<Pose> points;for(const auto&m:raw.poses){
      auto p=pose(m.pose.position.x,m.pose.position.y,heading(m.pose.orientation));
      if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.theta))return false;
      // Raw samples are guidance waypoints, not actuator authorization. Humble
      // backtraces may contain overlapping primitive samples. Reconstruct only
      // forward/rotate connectors below and validate every resulting pose.
      points.push_back(p);
    }
    // Collapse dense samples into geometric guide anchors before trying
    // shortcuts. Testing hundreds of equivalent interior samples wastes the
    // bounded service budget. This is guidance-only: connectors are checked.
    std::vector<Pose> anchors;
    for(const auto&p:points){
      if(anchors.size()>=2){
        const auto&a=anchors[anchors.size()-2];const auto&b=anchors.back();
        bool rotation=distance(a,b)<1e-4&&distance(b,p)<1e-4;
        bool line=std::abs(wrap(a.theta-b.theta))<.005&&std::abs(wrap(b.theta-p.theta))<.005&&
          std::abs((b.x-a.x)*(p.y-b.y)-(b.y-a.y)*(p.x-b.x))<1e-6;
        if(rotation||line){anchors.back()=p;continue;}
      }
      anchors.push_back(p);
    }
    points=std::move(anchors);
    // Simplify to exactly executable rotate/forward/rotate pieces. Every piece
    // is rechecked against the whole inflated footprint, including interiors.
    // Exact start/goal connectors handle lattice cell/heading quantization;
    // never send an unvalidated snapped or curved path to the executor.
    out={start};Pose cur=start;size_t cursor=0;
    while(!expired()){
      std::vector<Pose> edge;
      if(connect(cur,goal,edge)||connect(cur,goal,edge,-1)){out.insert(out.end(),edge.begin()+1,edge.end());return true;}
      if(cursor>=points.size())return false;
      bool found=false;
      for(size_t j=points.size();j>cursor;--j){
        const auto&native=points[j-1];if(distance(cur,native)<.10)continue;
        // Lattice cell centers and 5-degree footprint bins can leave a guide
        // anchor just inside our exact contour margin. Try at most one cell of
        // adjustment, never reduce that margin; validate the complete connector.
        const double cell=map_->getResolution();
        const std::vector<std::pair<double,double>> shifts={{0,0},{0,-cell},{0,cell},{-cell,0},{cell,0}};
        for(const auto&shift:shifts){
          Pose next=native;next.x+=shift.first;next.y+=shift.second;
          if(connect(cur,next,edge)||connect(cur,next,edge,-1)){
            out.insert(out.end(),edge.begin()+1,edge.end());cur=next;cursor=j;found=true;break;
          }
        }
        if(found)break;
      }
      if(!found)return false;
    }
    return false;
  }
  void compute(const wheelchair_interfaces::srv::ComputeDiscretePath::Request&r,
               wheelchair_interfaces::srv::ComputeDiscretePath::Response&out,Clock::time_point begin){
    out.reason="INVALID_REQUEST";auto finite=[](const Pose&p){return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.theta);};
    if(r.header.frame_id!="odom"||r.task_id.empty()||r.goals.empty()||r.goals.size()>8||!finite(r.start)||!finite(r.corridor_origin)||
       !std::all_of(r.goals.begin(),r.goals.end(),finite)||!std::isfinite(r.max_path_length)||r.max_path_length<=0||r.max_path_length>12||
       !std::isfinite(r.corridor_half_width)||r.corridor_half_width<0)return;
    if(!fresh()){out.reason="COSTMAP_STALE";return;}
    double age=(now()-rclcpp::Time(r.header.stamp)).seconds();if(age<-.05||age>1.){out.reason="STALE_REQUEST";return;}
    out.costmap_stamp=map_stamp_;deadline_=begin+std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(timeout_));
    origin_=r.corridor_origin;band_=r.corridor_half_width;used_smac_=false;Pose start=r.start;std::vector<Pose> path;
    double remaining=r.max_path_length;
    for(const auto&goal:r.goals){
      validation_padding_=padding_;
      if(!valid(start)){out.reason="START_COLLISION";return;}if(!valid(goal)){out.reason="GOAL_COLLISION";return;}
      std::vector<Pose> part;
      bool ok=r.allow_detour?lattice(start,goal,part):(connect(start,goal,part)||connect(start,goal,part,-1));
      if(!ok){out.reason=expired()?"SEARCH_TIMEOUT":r.allow_detour?"NO_EXECUTABLE_PATH":"LOCKED_ROUTE_BLOCKED";return;}
      for(size_t i=1;i<part.size();++i)remaining-=distance(part[i-1],part[i]);
      if(remaining<-.001){out.reason="PATH_DISTANCE_LIMIT";return;}
      path.insert(path.end(),part.begin(),part.end());start=goal;
    }
    if(!fresh()||expired()){out.reason="SEARCH_TIMEOUT";return;}
    out.path.header=out.header;for(const auto&p:path)out.path.poses.push_back(stamped(p));
    out.success=true;out.reason=!r.allow_detour?"SMAC_LOCKED_ROUTE_VALIDATED":used_smac_?"SMAC_LATTICE_PLANNED":"SMAC_DIRECT_ROUTE_VALIDATED";
  }
  pluginlib::ClassLoader<nav2_core::GlobalPlanner>&loader_;
  nav2_util::LifecycleNode::SharedPtr context_;
  std::shared_ptr<SnapshotCostmap> snapshot_;nav2_core::GlobalPlanner::Ptr planner_;
  nav2_costmap_2d::Costmap2D*map_=nullptr;nav2_costmap_2d::Footprint body_,footprint_;
  bool configured_=false,valid_map_=false,used_smac_=false;double timeout_,map_timeout_,padding_=0.,validation_padding_=0.,sampling_=0.,band_=0.;Pose origin_;
  rclcpp::Time map_stamp_{0,0,RCL_ROS_TIME};Clock::time_point map_received_,deadline_;
  std::string session_;uint64_t sequence_=0;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr raw_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;
  rclcpp::Publisher<wheelchair_interfaces::msg::InputHealth>::SharedPtr health_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<wheelchair_interfaces::srv::ComputeDiscretePath>::SharedPtr service_;
};
int main(int argc,char**argv){
  rclcpp::init(argc,argv);
  // Plugin-created callbacks leave weak control blocks in the lifecycle
  // node. Keep the library loaded until the entire context/executor is gone.
  pluginlib::ClassLoader<nav2_core::GlobalPlanner> libraries("nav2_core","nav2_core::GlobalPlanner");
  auto context=std::make_shared<nav2_util::LifecycleNode>("smac_lattice_plugin");
  auto node=std::make_shared<SmacInterface>(context,libraries);
  {
    rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(context->get_node_base_interface());executor.add_node(node);
    executor.spin();executor.remove_node(node);executor.remove_node(context->get_node_base_interface());
  }
  node.reset();context.reset();rclcpp::shutdown();return 0;
}
