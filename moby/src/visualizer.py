#!/usr/bin/python
import roslib
roslib.load_manifest("moby")
import rospy, math
from moby.msg import GPS
from geometry_msgs.msg import Vector3, Point
from visualization_msgs.msg import Marker
from sensor_msgs.msg import FluidPressure
from std_msgs.msg import Header, ColorRGBA

lat = 0.0
lon = 0.0
fix = False
pressure_offset = -1
relative_alt = 0

def update_gps(data):
    global lat
    global lon
    global fix
    lat = data.lat
    lon = data.long
    fix = data.fix

def update_pressure(data):
    global pressure_offset
    global relative_alt
    raw_pressure = data.fluid_pressure
    if pressure_offset == -1:
        rospy.loginfo("Setting pressure offset: " + str(raw_pressure) + " Pa")
        pressure_offset = raw_pressure
        
    relative_alt = pressure_offset - raw_pressure

def visualizer():
    rospy.Subscriber("baro/pressure", FluidPressure, update_pressure)
    rospy.Subscriber("gps", GPS, update_gps)
    rate = rospy.Rate(1)
    trace_pub = rospy.Publisher("viz/gps_trace", Marker)
    primary_geofence_pub = rospy.Publisher("viz/primary_geofence", Marker)
    secondary_geofence_pub = rospy.Publisher("viz/secondary_geofence", Marker)
    trace_points = []
    # initialize parameters
    secondary_geofence_width = rospy.get_param("secondary_geofence/width", 0.00002)
    min_lon = 0
    max_lon = 0
    min_lat = 0
    max_lat = 0
    try:
        min_lon = rospy.get_param("geofence/lon/min")
        max_lon = rospy.get_param("geofence/lon/max")
        min_lat = rospy.get_param("geofence/lat/max")
        max_lat = rospy.get_param("geofence/lat/min")
    except KeyError:
        rospy.logfatal("Geofence was not configured on the parameter server! Shutting down")
        sys.exit(1)

    # get parameters with defaults that require the aforementioned non-default parameters
    secondary_min_lat = rospy.get_param("secondary_geofence/lat/min", min_lat + secondary_geofence_width)
    secondary_max_lat = rospy.get_param("secondary_geofence/lat/max", max_lat - secondary_geofence_width)
    secondary_min_lon = rospy.get_param("secondary_geofence/lon/min", min_lon - secondary_geofence_width)
    secondary_max_lon = rospy.get_param("secondary_geofence/lon/max", max_lon + secondary_geofence_width)
    
    while not rospy.is_shutdown():
        if fix and pressure_offset != -1:
            gps_divisor = rospy.get_param("~gps_divisor", 0.00001)
            try: 
                primary_geofence_pts = (
                    Point(0, 0, 0),
                    Point(0, (max_lat - min_lat) / gps_divisor, 0),
                    Point((max_lon - min_lon) / gps_divisor, (max_lat - min_lat) / gps_divisor, 0),
                    Point((max_lon - min_lon) / gps_divisor, 0, 0),
                    Point(0, 0, 0)
                )
                secondary_geofence_pts = (
                    Point((secondary_min_lon - min_lon) / gps_divisor, (secondary_min_lat - min_lat) / gps_divisor, 0),
                    Point((secondary_min_lon - min_lon) / gps_divisor, (secondary_max_lat - min_lat) / gps_divisor, 0),
                    Point((secondary_max_lon - min_lon) / gps_divisor, (secondary_max_lat - min_lat) / gps_divisor, 0),
                    Point((secondary_max_lon - min_lon) / gps_divisor, (secondary_min_lat - min_lat) / gps_divisor, 0),
                    Point((secondary_min_lon - min_lon) / gps_divisor, (secondary_min_lat - min_lat) / gps_divisor, 0)
                )
                trace_points.append(Point((lon - min_lon) / gps_divisor, (lat - min_lat) / gps_divisor, relative_alt / rospy.get_param("~baro_divisor", 10)))
                trace_color = rospy.get_param("~trace_color", {"r": 0, "g": 1, "b": 0, "a": 1})
                viz_trace_color = ColorRGBA(trace_color['r'], trace_color['g'], trace_color['b'], trace_color['a'])
                
                primary_geofence_color = rospy.get_param("~primary_geofence_color", {"r": 1, "g": 1, "b": 0, "a": 1})
                viz_primary_geofence_color = ColorRGBA(primary_geofence_color['r'], primary_geofence_color['g'], primary_geofence_color['b'], primary_geofence_color['a'])
                secondary_geofence_color = rospy.get_param("~secondary_geofence_color", {"r": 1, "g": 0, "b": 0, "a": 1})
                viz_secondary_geofence_color = ColorRGBA(secondary_geofence_color['r'], secondary_geofence_color['g'], secondary_geofence_color['b'], secondary_geofence_color['a'])
                
                scale = rospy.get_param("~scale", 0.25)
                scale_vec = Vector3(scale, 0, 0)
        
                trace_ns = rospy.get_param("~trace_ns", "moby_trace")
                primary_geofence_ns = rospy.get_param("~primary_geofence_ns", "moby_primary_geofence")
                secondary_geofence_ns = rospy.get_param("~secondary_geofence_ns", "moby_secondary_geofence")
                            
                header = Header()
                header.stamp = rospy.Time.now()
                header.frame_id = "map"
                trace_msg = Marker(header=header, ns=trace_ns, id=0, type=4, action=0, scale=scale_vec, color=viz_trace_color, lifetime=rospy.Duration(), frame_locked=False, points=trace_points)
                trace_msg.pose.orientation.w = 1.0
                trace_pub.publish(trace_msg)
                
                primary_geofence_msg = Marker(header=header, ns=primary_geofence_ns, id=0, type=4, action=0, scale=scale_vec, color=viz_primary_geofence_color, lifetime=rospy.Duration(), frame_locked=False, points=primary_geofence_pts)
                primary_geofence_pub.publish(primary_geofence_msg)
                secondary_geofence_msg = Marker(header=header, ns=secondary_geofence_ns, id=0, type=4, action=0, scale=scale_vec, color=viz_secondary_geofence_color, lifetime=rospy.Duration(), frame_locked=False, points=secondary_geofence_pts)
                secondary_geofence_pub.publish(secondary_geofence_msg)
            except KeyError:
                rospy.logerr("Visualization is enabled but zero point(s) are not set!")
                
        rate.sleep()

if __name__ == "__main__":
    rospy.init_node('visualizer')
    try:
        visualizer()
    except rospy.ROSInterruptException: 
        pass