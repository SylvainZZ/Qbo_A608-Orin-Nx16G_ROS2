


cd ~/qbo_ws
colcon build --packages-select qbo_social
source install/setup.bash


ros2 launch qbo_social social.launch.py

ros2 topic list | grep qbo_social
/qbo_social/events
/qbo_social/world_state

ros2 topic echo /qbo_social/events


ros2 topic echo /qbo_social/world_state

coté aiml :
ros2 service call /aiml/generate_text qbo_msgs/srv/GenerateText "{type: 'conversation', text: 'bonjour', context_json: ''}"

ros2 service call /aiml/generate_text qbo_msgs/srv/GenerateText "$(cat <<EOF
type: 'diagnostic'
text: ''
context_json: '{"key": "qbo_dynamixel|head_pan_joint", "severity": "warning", "message": "High power consumption"}'
EOF
)"


┌──────────────────────────────────────────────────────────┐
│                    STATE MACHINE                          │
├──────────────────────────────────────────────────────────┤
│                                                           │
│  [IDLE] ──────FACE_APPEARED──────> [TRACKING_FULL]      │
│                                      │                    │
│                                      │ face_stable       │
│                                      │ + 5s delay        │
│                                      ▼                    │
│  [IDLE] <───SEARCH_TIMEOUT───  [HEAD_ONLY_TRACKING]     │
│    ▲              30s               │                    │
│    │                                │ FACE_LOST          │
│    │                                ▼                    │
│    └─────────────────────────  [SEARCHING]              │
│                                  (rotation active)       │
│                                      │                    │
│                                      │ FACE_APPEARED     │
│                                      └──────────────────>│
│                                    [TRACKING_FULL]       │
└──────────────────────────────────────────────────────────┘

====================================================================================================
                              🤖 QBO SOCIAL BEHAVIOR ENGINE - DEBUG MONITOR 🤖
====================================================================================================
Légende:             📥 Event  |  🌍 World  |  💭 Decision  |  🎯 Intent
Option:              --verbose pour afficher les mises à jour WorldState
====================================================================================================

📥 EVENT #1   │ FACE_APPEARED             │ from face_tracking       │ dist=1.37m
🎯 INTENT #1  │ TRACK_FACE_FULL           │ reason=face_appeared

📥 EVENT #2   │ FACE_STABLE               │ from face_tracking       │ dist=1.35m

────────────────────────────────────────────────────────────────────────────────────────────────────
💭 DECISION TRACE #1
   Trigger   : FACE_STABLE
   Intent    : TRACK_FACE_FULL
   Reason    : face_stable_during_idle
   Relevance : [██████████] 1.00
   Status    : ✓  🔊
────────────────────────────────────────────────────────────────────────────────────────────────────

