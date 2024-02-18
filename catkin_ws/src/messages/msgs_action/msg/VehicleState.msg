Header header
ackermann_msgs/AckermannDrive drive	# Look for ackermann_msgs at ros wiki
int32 engine_speed					# Engine revolution-counter [range 0 to 10240 rpm]
float64 throttle					# Throttle pedal position [range: 0 to 99.96%]
int32 car_gear 						# Gear selection [range: -1 (reverse), 0 (neutral) to 5 (fifth)]
int32 brake							# Brake active (1) or inactive (0)
int32 handbrake						# Handbrake active (1) or inactive (0) 
