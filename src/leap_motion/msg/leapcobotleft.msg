######################################################
## Custom Message to communicate with app developed ##
######################################################

Header header

# Left hand information
bool is_left_hand                      # Left hand detected
geometry_msgs/Point left_hand_palmpos  # Palm's position
bool left_hand_fist                    # Fist gesture recognize
bool left_hand_thumb_up                # Thumb up gesture recognize
bool left_hand_pinch                   # Pinch gesture recognize
float32 left_hand_pinch_value		   # Pinch gesture value
bool left_hand_origin_frame            # Reference frame set
bool left_hand_set_origin_frame_detected # Detect gesture
float32 left_hand_rotate_value         # Values between [-1..0..1]
float32 left_hand_turn_value           # Values between [-1..0..1]
float32 left_hand_swing_value          # Values between [-1..0..1]