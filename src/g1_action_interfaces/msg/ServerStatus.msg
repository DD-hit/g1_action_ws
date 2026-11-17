uint8 STARTING=0
uint8 IDLE=1
uint8 RUNNING=2
uint8 CANCELLING=3
uint8 RECOVERING=4
uint8 FAULT=5
std_msgs/Header header
uint16 api_version 1
uint8 state
bool accepting_goals
string active_request_id
string active_action_name
string stage
uint8 authority_state
string message
bool last_result_valid false
g1_action_interfaces/ExecutionResult last_result
