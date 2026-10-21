// SPDX-License-Identifier: Apache-2.0
// Real Gazebo LaserScan data. Rendering mutations are confined to Render events.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <vector>
#include <QEvent>
#include <ignition/gui/Plugin.hh>
#include <ignition/gazebo/gui/GuiSystem.hh>
#include <ignition/gazebo/Util.hh>
#include <gz/sim/components/Sensor.hh>
#include <ignition/gui/Application.hh>
#include <ignition/gui/MainWindow.hh>
#include <ignition/gui/GuiEvents.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/rendering/RenderingIface.hh>
#include <ignition/rendering/RenderEngine.hh>
#include <ignition/rendering/Scene.hh>
#include <ignition/rendering/LidarVisual.hh>
#include <ignition/transport/Node.hh>
#include <ignition/msgs/laserscan.pb.h>

class ScanPoints : public ignition::gazebo::GuiSystem
{
  Q_OBJECT
public:
  void LoadConfig(const tinyxml2::XMLElement *xml) override
  {
    this->title = "Lidar scan points";
    if (xml && xml->FirstChildElement("topic") && xml->FirstChildElement("topic")->GetText())
      topic = xml->FirstChildElement("topic")->GetText();
    if (xml && xml->FirstChildElement("point_size"))
      xml->FirstChildElement("point_size")->QueryIntText(&size);
    size = std::clamp(size, 1, 30);
    auto window = ignition::gui::App()->findChild<ignition::gui::MainWindow *>();
    if (window) window->installEventFilter(this);
    node.Subscribe(topic, &ScanPoints::OnScan, this);
  }
  ~ScanPoints() override {node.Unsubscribe(topic);}
  void Update(const ignition::gazebo::UpdateInfo &,
              ignition::gazebo::EntityComponentManager &ecm) override
  {
    auto entity = ecm.EntityByComponents(ignition::gazebo::components::SensorTopic(topic));
    if (entity == ignition::gazebo::kNullEntity && !topic.empty() && topic[0] == '/')
      entity = ecm.EntityByComponents(ignition::gazebo::components::SensorTopic(topic.substr(1)));
    if (entity == ignition::gazebo::kNullEntity) return;
    std::lock_guard<std::mutex> lock(mutex);
    sensorPose = ignition::gazebo::worldPose(entity, ecm);
    poseReady = true;
  }

protected:
  bool eventFilter(QObject *obj, QEvent *event) override
  {
    if (event->type() != ignition::gui::events::Render::kType)
      return QObject::eventFilter(obj, event);
    if (!visual)
    {
      for (const auto &name : ignition::rendering::loadedEngines())
      {
        auto engine = ignition::rendering::engine(name);
        if (!engine || engine->SceneCount() == 0) continue;
        scene = engine->SceneByIndex(0);
        if (!scene) continue;
        visual = scene->CreateLidarVisual();
        if (!visual) continue;
        scene->RootVisual()->AddChild(visual);
        visual->SetType(ignition::rendering::LidarVisualType::LVT_POINTS);
        visual->SetSize(size);
        visual->SetDisplayNonHitting(false);
        break;
      }
    }
    if (!visual) return false;
    std::lock_guard<std::mutex> lock(mutex);
    if (!received || !poseReady || std::chrono::steady_clock::now() - receivedAt > std::chrono::milliseconds(500))
    {
      visual->SetVisible(false);
      return false;
    }
    if (!dirty) return false;
    const auto n = static_cast<size_t>(msg.count()) * std::max(1u, msg.vertical_count());
    if (!n || n != static_cast<size_t>(msg.ranges_size()) || !msg.has_world_pose())
    {
      visual->SetVisible(false);
      return false;
    }
    // Fortress GPU lidar leaves LaserScan.world_pose at identity; use Gazebo's
    // actual sensor entity world transform, including mounting height and motion.
    visual->SetWorldPose(sensorPose);
    visual->SetVerticalRayCount(std::max(1u, msg.vertical_count()));
    visual->SetHorizontalRayCount(msg.count());
    visual->SetMinHorizontalAngle(msg.angle_min());
    visual->SetMaxHorizontalAngle(msg.angle_max());
    visual->SetMinVerticalAngle(msg.vertical_angle_min());
    visual->SetMaxVerticalAngle(msg.vertical_angle_max());
    visual->SetMinRange(msg.range_min());
    visual->SetMaxRange(msg.range_max());
    std::vector<double> ranges(msg.ranges().begin(), msg.ranges().end());
    for (auto &range : ranges)
      if (!std::isfinite(range) || range < msg.range_min()) range = msg.range_max();
    visual->SetPoints(ranges);
    visual->SetVisible(true);
    visual->Update();
    dirty = false;
    return false;
  }
private:
  void OnScan(const ignition::msgs::LaserScan &scan)
  {
    std::lock_guard<std::mutex> lock(mutex);
    msg = scan; dirty = received = true;
    receivedAt = std::chrono::steady_clock::now();
  }
  ignition::transport::Node node;
  ignition::rendering::ScenePtr scene;
  ignition::rendering::LidarVisualPtr visual;
  ignition::msgs::LaserScan msg;
  std::mutex mutex;
  bool dirty{false}, received{false}, poseReady{false};
  ignition::math::Pose3d sensorPose;
  int size{7};
  std::string topic{"/wheelchair/scan"};
  std::chrono::steady_clock::time_point receivedAt;
};
IGNITION_ADD_PLUGIN(ScanPoints, ignition::gui::Plugin)
#include "ScanPoints.moc"
