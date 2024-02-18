# ART Navigator behaviors (lower numbers have higher priority)
# $Id: Behavior.msg 996 2011-02-27 16:07:34Z jack.oquin $

#Emergency state
int32 operational = 0
int32 emergency = 1

#Acceleration state
int32 manual_acceleration = 0
int32 auto_acceleration = 2

#Steering state
int32 manual_steering = 0
int32 auto_steering = 4

int32 value
