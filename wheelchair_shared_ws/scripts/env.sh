# Source this file. Wheelchair owns its runtime; no TurtleBot overlay required.
_wc_ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
_wc_rt="${WHEELCHAIR_GAZEBO_RUNTIME:-$_wc_ws/vendor/runtime}"
if [ ! -d "$_wc_rt/opt/ros/humble" ]; then
  echo "Missing Gazebo runtime: $_wc_rt (set WHEELCHAIR_GAZEBO_RUNTIME if relocated)" >&2
  unset _wc_ws _wc_rt
  return 1
fi
source /opt/ros/humble/setup.bash
export PATH="$_wc_rt/usr/bin:$PATH"
export LD_LIBRARY_PATH="$_wc_rt/usr/lib/aarch64-linux-gnu:$_wc_rt/usr/lib:$_wc_rt/opt/ros/humble/lib:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="$_wc_rt/opt/ros/humble/local/lib/python3.10/dist-packages:$_wc_rt/opt/ros/humble/lib/python3.10/site-packages:${PYTHONPATH:-}"
export AMENT_PREFIX_PATH="$_wc_rt/opt/ros/humble:${AMENT_PREFIX_PATH:-}"
export RUBYLIB="$_wc_rt/usr/lib/ruby/3.0.0:$_wc_rt/usr/lib/aarch64-linux-gnu/ruby/3.0.0:$_wc_rt/usr/lib/ruby/vendor_ruby:$_wc_rt/usr/lib/ruby"
export IGN_CONFIG_PATH="$_wc_rt/usr/share/ignition"
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH="$_wc_rt/usr/lib/aarch64-linux-gnu/ign-gazebo-6/plugins"
export IGN_GUI_PLUGIN_PATH="$_wc_rt/usr/lib/aarch64-linux-gnu/ign-gui-6/plugins:$_wc_rt/usr/lib/aarch64-linux-gnu/ign-gazebo-6/plugins/gui"
export IGN_GAZEBO_PHYSICS_ENGINE_PATH="$_wc_rt/usr/lib/aarch64-linux-gnu/ign-physics-5/engine-plugins"
export IGN_RENDERING_PLUGIN_PATH="$_wc_rt/usr/lib/aarch64-linux-gnu/ign-rendering-6/engine-plugins"
export IGN_GAZEBO_RESOURCE_PATH="$_wc_rt/usr/share:$_wc_rt/usr/share/ignition/ignition-gazebo6"
export IGN_GUI_RESOURCE_PATH="$_wc_rt/usr/share/ignition/ignition-gui6"
export OGRE2_RESOURCE_PATH="$_wc_rt/usr/lib/aarch64-linux-gnu/OGRE-Next"
export IGN_RENDERING_RESOURCE_PATH="$_wc_rt/usr/share/ignition/ignition-rendering6"
export SDF_PATH="$_wc_rt/usr/share/sdformat12/1.9"
export QML2_IMPORT_PATH="$_wc_rt/usr/lib/aarch64-linux-gnu/qt5/qml:$_wc_rt/usr/lib/aarch64-linux-gnu/ign-gazebo-6/plugins/gui:${QML2_IMPORT_PATH:-}"
export QT_QPA_PLATFORM=xcb
# local_setup avoids an obsolete build-time underlay chain embedded in setup.
if [ -f "$_wc_ws/install/local_setup.bash" ]; then source "$_wc_ws/install/local_setup.bash"; fi
unset _wc_ws _wc_rt
# Verified workspace-local Nav2 subset, installed by install_nav2_runtime.py.
_wc_nav="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/vendor/nav2/opt/ros/humble"
if [ -d "$_wc_nav" ]; then
  export AMENT_PREFIX_PATH="$_wc_nav:${AMENT_PREFIX_PATH:-}"
  export CMAKE_PREFIX_PATH="$_wc_nav:${CMAKE_PREFIX_PATH:-}"
  export LD_LIBRARY_PATH="$_wc_nav/lib:${LD_LIBRARY_PATH:-}"
  export PYTHONPATH="$_wc_nav/local/lib/python3.10/dist-packages:${PYTHONPATH:-}"
fi
unset _wc_nav
