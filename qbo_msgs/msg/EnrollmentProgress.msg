# EnrollmentProgress.msg
# Real-time feedback during face enrollment process

# Person being enrolled
string person_name

# Progress counters
int32 current_sample        # Current sample number (1-based)
int32 target_samples        # Total number of samples needed

# Quality metrics
float32 current_quality     # Quality of current sample (0.0-1.0)
float32 average_quality     # Average quality of all captured samples (0.0-1.0)
float32 diversity_score     # Diversity/variation score (0.0-1.0, higher is better)

# Status tracking
string status               # "started", "capturing", "rejected", "completed", "error"

# User feedback
string feedback_message     # Human-readable message for the user
string guidance             # Next action suggestion (e.g., "Turn head LEFT")

# For UI integration (qbo_social)
string quality_level        # "EXCELLENT", "GOOD", "ACCEPTABLE", "LOW"
string emoji                # Emoji for visual feedback: "✅", "👍", "⚠️", "❌"
