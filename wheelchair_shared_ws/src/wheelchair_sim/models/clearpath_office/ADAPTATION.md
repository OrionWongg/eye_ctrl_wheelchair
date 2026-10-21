# Clearpath Office / Fortress wheelchair adapter

Source: https://github.com/clearpathrobotics/cpr_gazebo/tree/509afbeec942c6aad376a5a76a24330f1c56dc17/cpr_office_gazebo
License: BSD-3-Clause; see LICENSE.txt (Clearpath Robotics, 2020).

Original office.dae and its eight referenced textures are copied without modification.
The original planar extent is 44.01 × 25.45 metres. This is a bounding extent, not usable floor area.

For this flat-floor wheelchair test the derived collision mesh includes original surfaces
clipped to z=0.04..1.30 m. A separate level ground plane supplies floor contacts. This avoids
reporting ordinary wheel-floor contacts as obstacle collisions. The current wheelchair's
highest physical part is below 1.30 m. This adapter is not for taller robots or drop detection.

The obstacle oracle is derived from exactly that collision mesh, projected to 5 cm cells,
conservatively dilated by one cell for rasterization error, then merged into rectangles.
Narrow openings may be conservatively blocked. No building hull bounding box is substituted
for interior walls. It is a simulation oracle, not a sensor-derived perception map.

Regenerate with scripts/import_clearpath_office.py. obstacle_map.json records source hash,
commit, dimensions, collision band, grid resolution, and all merged rectangles.
