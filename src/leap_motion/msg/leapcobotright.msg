######################################################
## Custom Message to communicate with app developed ##
######################################################

Header header

# Right hand information
bool is_right_hand                      # Right hand detected
geometry_msgs/Point right_hand_palmpos  # Palm's position
bool right_hand_fist                    # Fist gesture recognize
bool right_hand_thumb_up                # Thumb up gesture recognize
bool right_hand_pinch                   # Pinch gesture recognize
float32 right_hand_pinch_value			# Pinch gesture value
bool right_hand_origin_frame            # Reference frame set
bool right_hand_set_origin_frame_detected # Detect gesture
float32 right_hand_rotate_value         # Values between [-1..0..1] rads
float32 right_hand_turn_value           # Values between [-1..0..1] rads
float32 right_hand_swing_value          # Values between [-1..0..1] rads