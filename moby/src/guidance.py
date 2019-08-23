#!/usr/bin/python
import roslib
roslib.load_manifest("moby")
import rospy, math, sys
from moby.msg import Yaw, GPS, PWMCommand

STRAIGHT_PWMS = (2, 2, 2.33, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1)
LEFT_PWMS = (2, 1, 2.33, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1)
RIGHT_PWMS = (1, 2, 2.33, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1)
STOP_PWMS = (-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1)

lat = 0.0
lon = 0.0
fix = False

yaw = 0.0

min_lon = 0
max_lon = 0
min_lat = 0 
max_lat = 0 
    
secondary_min_lon = 0
secondary_max_lon = 0
secondary_min_lat = 0
secondary_max_lat = 0

def update_gps(data):
#     print data.fix
    global lat
    global lon
    global fix
    global heading
    lat = data.lat
    lon = data.long
    fix = data.fix
    heading = math.radians(data.hdg)
    
    
def update_yaw(data):
    global yaw
    yaw = data.yaw


def in_primary_geofence():
    return fix and lat > min_lat and lat < max_lat and lon > min_lon and lon < max_lon


def in_secondary_geofence():
    return fix and lat > secondary_min_lat and lat < secondary_max_lat and lon > secondary_min_lon and lon < secondary_max_lon


def angle_error(target, current):
    return math.atan2(math.sin(target - current), math.cos(target - current))

def stupid_brain():
    rospy.Subscriber("gps", GPS, update_gps)
    rospy.Subscriber("yaw", Yaw, update_yaw)
    pub = rospy.Publisher("navpwm", PWMCommand)
    rate = rospy.Rate(100)
    
    # get values for parameters that have defaults
    after_turn_drive = rospy.get_param("~after_turn_drive", 3) # seconds
    secondary_geofence_width = rospy.get_param("secondary_geofence/width", 0.00002) # about 8 feet

    # get parameters with no defaults
    try:
        global min_lon
        global max_lon
        global min_lat
        global max_lat
        
        min_lon = rospy.get_param("geofence/lon/min")
        max_lon = rospy.get_param("geofence/lon/max")
        min_lat = rospy.get_param("geofence/lat/max")
        max_lat = rospy.get_param("geofence/lat/min")
    except KeyError:
        rospy.logfatal("Geofence was not configured on the parameter server! Shutting down")
        sys.exit(1)

    # get parameters with defaults that require the aforementioned non-default parameters
    global secondary_min_lon
    global secondary_max_lon
    global secondary_min_lat
    global secondary_max_lat
    secondary_min_lat = rospy.get_param("secondary_geofence/lat/min", min_lat - secondary_geofence_width)
    secondary_max_lat = rospy.get_param("secondary_geofence/lat/max", max_lat + secondary_geofence_width)
    secondary_min_lon = rospy.get_param("secondary_geofence/lon/min", min_lon - secondary_geofence_width)
    secondary_max_lon = rospy.get_param("secondary_geofence/lon/max", max_lon + secondary_geofence_width)


    def publish_and_sleep(duty_cycles):
        pub.publish(PWMCommand(duty_cycles=duty_cycles))
        rate.sleep()
    while not fix:
        rospy.sleep(0.5)
    if fix:
        rospy.loginfo("GPS acquired")
    while not rospy.is_shutdown():
        if fix:
            if in_primary_geofence():
                rospy.loginfo("within geofence; driving straight")
            
            while in_primary_geofence():
                publish_and_sleep(STRAIGHT_PWMS)
        
            # we exited the loop, so we're outside the geofence or we lost fix
            
            # if we lost our GPS fix then stop (this will go to the top of the while loop and then into no-fix mode)
            if not fix:
                rospy.logerr("GPS fix lost")
                continue
            
            if not in_secondary_geofence():
                rospy.logfatal("outside secondary geofence; stopping")
                while not rospy.is_shutdown():
                    # loop infinitely
                    publish_and_sleep(STOP_PWMS)
                break
            
            # otherwise figure out where we should be pointing
            target = 0
        
            if lat < min_lat: 
                rospy.loginfo("too far south!")
                target = 0 # 0 degrees (north)
            elif lon < min_lon: 
                rospy.loginfo("too far west!")
                target = math.pi / 2 # 90 degrees (east)
            elif lat > max_lat: 
                rospy.loginfo("too far north!")
                target = math.pi # 180 degrees (south)
            elif lon > max_lon:
                rospy.loginfo("too far east!")
                target = 3 * math.pi / 2 # 270 degrees (west)
            else:
                rospy.logwarn("something weird happened: exited straight loop while inside geofence. resetting")
                continue # restart the logic
            
            rospy.loginfo("current heading " + str(math.degrees(heading)) + "deg")
            turn_amount = angle_error(target, heading)
            
            yaw_offset = yaw
            
            if turn_amount < 0:
                rospy.loginfo("turning left by " + str(math.degrees(turn_amount)) + "deg to " + str(math.degrees(target)) + "deg")
                while yaw - yaw_offset > turn_amount and in_secondary_geofence():
                    publish_and_sleep(LEFT_PWMS)
            else:
                rospy.loginfo("turning right by " + str(math.degrees(turn_amount)) + "deg to " + str(math.degrees(target)) + "deg")
                while yaw - yaw_offset < turn_amount and in_secondary_geofence():
                    publish_and_sleep(RIGHT_PWMS)
            
            # check our fix again, just to be safe
            if not fix:
                rospy.logerr("GPS fix lost")
                continue
            if not in_secondary_geofence():
               rospy.logfatal("outside secondary geofence; stopping")
               while not rospy.is_shutdown():
                   # loop infinitely
                   publish_and_sleep(STOP_PWMS)
               break
            # drive straight
            rospy.loginfo("driving straight to return to geofence")
            start_time = rospy.get_time()
            while rospy.get_time() - start_time < after_turn_drive:
                publish_and_sleep(STRAIGHT_PWMS)

        else:
            rospy.logerror('no GPS fix')
            while not fix:
                publish_and_sleep(STOP_PWMS)

if __name__ == "__main__":
    rospy.init_node('stupid_guidance')
    try:
        stupid_brain()
    except rospy.ROSInterruptException: 
        pass