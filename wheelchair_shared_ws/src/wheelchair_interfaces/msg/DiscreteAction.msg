uint8 STOP=0
uint8 FORWARD=1
uint8 LEFT=2
uint8 RIGHT=3
std_msgs/Header header
string session_id
uint64 sequence
string intent_session_id
uint64 intent_sequence
uint8 action
uint32 ttl_ms
# Nonzero bounds one uninterrupted RUNNING episode; renewals cannot extend it.
# STOP must be received before a new bounded episode can start.
uint32 max_run_ms
string reason
