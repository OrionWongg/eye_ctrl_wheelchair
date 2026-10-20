# Local Gazebo runtime

Gazebo Fortress 6.18.0 runtime is owned by this wheelchair workspace. It was copied from the previously verified local runtime on 2026-10-14 after the old TurtleBot directory was moved to Trash. The old project was not restored. Source package versions/hashes are recorded in runtime-packages.json; original shared package notices remain under runtime/usr/share/doc. CLI YAML library paths were relocated to this workspace.

Source scripts/env.sh from the workspace root. It initializes ROS Humble, this runtime and install/local_setup.bash without the obsolete TurtleBot underlay. Runtime files are excluded from colcon source discovery by runtime/COLCON_IGNORE. The native ScanPoints plugin was rebuilt against the relocated libraries.

WHEELCHAIR_GAZEBO_RUNTIME can select a separately configured compatible runtime. Changing installation prefixes also requires matching Ignition CLI metadata and rebuilding native plugins.
