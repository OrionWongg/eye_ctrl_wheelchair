# Bounded planar collision representation. Simulation provides full-height geometry.
# Hardware scan maps conservatively encode BOTH occupied and unknown cells as
# obstacle polygons; complete means no unrepresented cells INSIDE the boundary.
# Scan maps describe only the configured height slice, not full-height coverage.
std_msgs/Header header
string session_id
uint64 sequence
geometry_msgs/Polygon[] obstacles
geometry_msgs/Polygon known_free_boundary
bool complete
