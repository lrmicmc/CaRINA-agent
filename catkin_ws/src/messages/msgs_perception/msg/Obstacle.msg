# header for time/frame information
Header header

#object position 
int32   id                                # object ID useful in conjunction with the namespace for manipulating and deleting the object later
string  ns                    		  # Namespace to place this object in... used in conjunction with id to create a unique name for the object
geometry_msgs/Pose    pose    		  # object pose and orientation
geometry_msgs/Twist   twist  		  # object velocity
geometry_msgs/Vector3 linear_acceleration # object acceleration
geometry_msgs/Vector3 scale       	  # Scale of the object 1,1,1 means (usually 1 meter square)

#Fusion extras
sensor_msgs/PointCloud2 point_cloud
int32 class_id
string[] classes
msgs_perception/BoundingBox bbox

#Radar extras
bool    oncoming
float64 lat_rate
int32   track_status         # 0 = no target, 1 = new target, 2 = new updated target, 3 = updated target, 4 = coasted target, 5 = merged target, 6 = invalid coasted target, 7 = new coasted target
bool    bridge_object        # connects two or more objects that are associated with the same obstacle

#visualization extras
std_msgs/ColorRGBA color          # Color [0.0-1.0]
duration lifetime                 # How long the object should last before being automatically deleted.  0 means forever
bool     frame_locked             # If this marker should be frame-locked, i.e. retransformed into its frame every timestep
float32  animation_speed          # Speed of animation,
int32    action 	          # 0 add/modify an object, 1 (deprecated), 2 deletes an object, 3 deletes all objects
uint8    ADD=0
uint8    MODIFY=0
uint8    DELETE=2
int32 type                 		      # object type : -1 unknown
string vehicle_model

# based on Marker, See http://www.ros.org/wiki/rviz/DisplayTypes/Marker
