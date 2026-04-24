#world_state.msg

std_msgs/Header header

builtin_interfaces/Time stamp
bool face_present
bool face_stable
bool conversation_active
bool tracking_enabled
bool head_follow_enabled
bool base_follow_enabled
string focus_person_id
string focus_person_name
float32 engagement_level
string mode
string health_state

# Temporal context
string time_of_day      # MORNING, AFTERNOON, EVENING, NIGHT
string day_type         # WEEKDAY, WEEKEND

# Network context
bool network_connected       # Internet connectivity status
string wifi_ssid            # Current WiFi SSID (empty if not connected or not WiFi)
string wifi_status          # KNOWN, UNKNOWN (empty if not WiFi)
string ip_address           # Local IP address (empty if disconnected)

# Timestamps
builtin_interfaces/Time last_face_seen_time      # Last time a face was seen (FACE_APPEARED or FACE_STABLE)
builtin_interfaces/Time last_recognized_time     # Last time a person was recognized
builtin_interfaces/Time last_network_change_time # Last time network status changed