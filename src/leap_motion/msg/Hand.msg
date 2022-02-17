std_msgs/Header header

# A unique ID assigned to this Hand object
int32 lmc_hand_id

# This changed to between true/false depending if there's a hand detected in the LMC frame
bool is_present

# This changed to between true/false depending if there's a hand detected in the LMC frame
bool valid_gestures

# The duration of time this Hand has been visible to the Leap Motion Controller. 
float32 time_visible

# How confident we are with a given hand pose. 
# The confidence level ranges between 0.0 and 1.0 inclusive.
float32 confidence

# The following three angles are in radians!
float32 roll
float32 pitch
float32 yaw

# The direction vector
geometry_msgs/Vector3 direction

# The normal vector
geometry_msgs/Vector3 normal

# The angle between the fingers and the hand of a grab hand pose. 
# In radians
float32 grab_strength

# The distance between the thumb and index finger of a pinch hand pose. 
float32 pinch_strength

# The rate of change of the palm position in meters/second. 
float32[] palm_velocity

# The center position of the palm in meters from the Leap Motion Controller origin. 
geometry_msgs/Point palm_center

# The estimated width of the palm when the hand is in a flat position. 
float32 palm_width

# The radius (m) of a sphere fit to the curvature of this hand. 
float32 sphere_radius

# The center of a sphere fit to the curvature of this hand. 
float32[] sphere_center

# A string containing a brief, human readable description of the Hand object. 
string to_string 

# A list of fingers and gestures associated with this hand
Finger[] finger_list
Gesture[] gesture_list

# The arm associated with this hand
Arm arm
