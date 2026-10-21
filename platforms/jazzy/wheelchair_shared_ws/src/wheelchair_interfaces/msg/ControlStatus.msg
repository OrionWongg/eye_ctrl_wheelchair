uint8 PASS=0
uint8 ASSISTING=1
uint8 BLOCKED=2
uint8 FAULT=3
std_msgs/Header header
string session_id
uint64 sequence
string intent_session_id
string action_session_id
uint8 requested_command
uint8 executed_action
uint8 state
string reason
bool requires_rearm
bool ready_to_rearm
# Empty for legacy joystick commands.
string task_id
