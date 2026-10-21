#!/usr/bin/env bash
set -e
root="$(cd "$(dirname "$0")/.." && pwd)"
source /etc/os-release
if [[ "$ID" != ubuntu || "$VERSION_ID" != 24.04 ]]; then
  echo 'This setup targets Ubuntu 24.04 with ROS 2 Jazzy.' >&2
  exit 1
fi
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo 'Install ROS 2 Jazzy first: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html' >&2
  exit 1
fi
sudo apt-get install -y ros-jazzy-ros-gz python3-colcon-common-extensions \
  python3-venv python3-opencv python3-numpy python3-pytest libopencv-dev \
  build-essential pkg-config
python3 -m venv --system-site-packages "$root/.venv"
"$root/.venv/bin/pip" install -r "$root/requirements.txt"
bash "$root/scripts/build_purest.sh"
python3 "$root/wheelchair_shared_ws/scripts/import_willow_office.py"
source /opt/ros/jazzy/setup.bash
cd "$root/wheelchair_shared_ws"
colcon build --symlink-install --base-paths src/wheelchair_interfaces src/wheelchair_sim \
  --packages-select wheelchair_interfaces wheelchair_sim
echo 'Ready: ./scripts/run_sim.sh or ./scripts/run_sim.sh --eye'
