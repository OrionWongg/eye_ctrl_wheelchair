# Simulation-only complete planar collision map, not a lidar reconstruction.
# Polygons include every collision obstacle across wheelchair height.
std_msgs/Header header
string session_id
uint64 sequence
geometry_msgs/Polygon[] obstacles
geometry_msgs/Polygon known_free_boundary
bool complete
