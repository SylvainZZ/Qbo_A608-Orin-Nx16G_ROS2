uint8 INTENT_UNKNOWN=0
uint8 INTENT_CONTROL=1
uint8 INTENT_QUERY=2
uint8 INTENT_STATUS=3
uint8 INTENT_CANCEL=4

uint8 STATUS_PARSED=0
uint8 STATUS_RESOLVED=1
uint8 STATUS_NEEDS_CLARIFICATION=2
uint8 STATUS_REJECTED=3

string request_id
builtin_interfaces/Time stamp

string raw_text
uint8 intent_type
string action

qbo_home_interfaces/HomeTarget target
qbo_home_interfaces/HomeParameter[] parameters

float32 confidence
uint8 status

string missing_slot
string[] candidates

uint8 risk_level
bool confirmation_required
