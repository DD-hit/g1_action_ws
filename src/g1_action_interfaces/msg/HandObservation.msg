uint8 HANDEDNESS_UNKNOWN=0
uint8 HANDEDNESS_LEFT=1
uint8 HANDEDNESS_RIGHT=2
std_msgs/Header header
string tracking_id
uint8 handedness
float32 confidence
geometry_msgs/Point32[21] normalized_landmarks
