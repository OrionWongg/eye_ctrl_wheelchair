uint8 STOP=0
uint8 FORWARD=1
uint8 LEFT=2
uint8 RIGHT=3
std_msgs/Header header
string session_id
uint64 sequence
uint8 command
bool enabled
bool valid
uint32 ttl_ms
string reason

# Protocol-v1 finite task adapter; empty task_id preserves legacy behavior.
string task_id
uint8 task_kind
float64 reference_x
float64 reference_y
float64 reference_yaw
float64 target_yaw
# Directional navigation adapter only: elapsed since the original input event.
float64 task_elapsed
