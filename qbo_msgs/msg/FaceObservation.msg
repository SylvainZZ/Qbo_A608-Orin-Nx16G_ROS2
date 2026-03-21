# face_observation.msg

std_msgs/Header header

# tracking
uint32 face_id
uint32 track_id
uint8 tracking_state

# bounding box
float32 x
float32 y
float32 width
float32 height

# face center
float32 center_x
float32 center_y

# landmarks (YuNet 5 points)
float32[10] landmarks

# detection confidence
float32 detector_score

# quality score (tracker heuristic)
float32 quality

# face size (pixels)
float32 face_size

# estimated distance
float32 distance

# number of faces detected in frame
uint32 faces_detected

# image size
uint32 image_width
uint32 image_height