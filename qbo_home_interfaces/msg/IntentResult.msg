uint8 STATUS_SUCCESS=0
uint8 STATUS_FAILED=1
uint8 STATUS_CANCELLED=2
uint8 STATUS_NEEDS_CONFIRMATION=3
uint8 STATUS_NEEDS_CLARIFICATION=4

string request_id
builtin_interfaces/Time stamp

uint8 status
bool success

string spoken_response
string technical_message

string[] affected_entities
string error_code

qbo_home_interfaces/HomeIntent intent
