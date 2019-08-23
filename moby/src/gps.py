#!/usr/bin/python
import roslib
roslib.load_manifest("moby")
import re, rospy, navio.ublox
from moby.msg import GPS
from geometry_msgs.msg import Vector3

if __name__ == "__main__":
    try:
        rospy.init_node("gps")
        
        pub = rospy.Publisher("gps", GPS)

        ubl = navio.ublox.UBlox("spi:0.0", baudrate=5000000, timeout=2)

        ubl.configure_poll_port()
        ubl.configure_poll(navio.ublox.CLASS_CFG, navio.ublox.MSG_CFG_USB)
        #ubl.configure_poll(navio.ublox.CLASS_MON, navio.ublox.MSG_MON_HW)

        ubl.configure_port(port=navio.ublox.PORT_SERIAL1, inMask=1, outMask=0)
        ubl.configure_port(port=navio.ublox.PORT_USB, inMask=1, outMask=1)
        ubl.configure_port(port=navio.ublox.PORT_SERIAL2, inMask=1, outMask=0)
        ubl.configure_poll_port()
        ubl.configure_poll_port(navio.ublox.PORT_SERIAL1)
        ubl.configure_poll_port(navio.ublox.PORT_SERIAL2)
        ubl.configure_poll_port(navio.ublox.PORT_USB)
        ubl.configure_solution_rate(rate_ms=1000)

        ubl.set_preferred_dynamic_model(None)
        ubl.set_preferred_usePPP(None)

        ubl.configure_message_rate(navio.ublox.CLASS_NAV, navio.ublox.MSG_NAV_POSLLH, 1)
        ubl.configure_message_rate(navio.ublox.CLASS_NAV, navio.ublox.MSG_NAV_PVT, 1)
        ubl.configure_message_rate(navio.ublox.CLASS_NAV, navio.ublox.MSG_NAV_STATUS, 1)
        ubl.configure_message_rate(navio.ublox.CLASS_NAV, navio.ublox.MSG_NAV_SOL, 1)
        ubl.configure_message_rate(navio.ublox.CLASS_NAV, navio.ublox.MSG_NAV_VELNED, 1)
        ubl.configure_message_rate(navio.ublox.CLASS_NAV, navio.ublox.MSG_NAV_SVINFO, 1)
        ubl.configure_message_rate(navio.ublox.CLASS_NAV, navio.ublox.MSG_NAV_VELECEF, 1)
        ubl.configure_message_rate(navio.ublox.CLASS_NAV, navio.ublox.MSG_NAV_POSECEF, 1)
        ubl.configure_message_rate(navio.ublox.CLASS_RXM, navio.ublox.MSG_RXM_RAW, 1)
        ubl.configure_message_rate(navio.ublox.CLASS_RXM, navio.ublox.MSG_RXM_SFRB, 1)
        ubl.configure_message_rate(navio.ublox.CLASS_RXM, navio.ublox.MSG_RXM_SVSI, 1)
        ubl.configure_message_rate(navio.ublox.CLASS_RXM, navio.ublox.MSG_RXM_ALM, 1)
        ubl.configure_message_rate(navio.ublox.CLASS_RXM, navio.ublox.MSG_RXM_EPH, 1)
        ubl.configure_message_rate(navio.ublox.CLASS_NAV, navio.ublox.MSG_NAV_TIMEGPS, 5)
        ubl.configure_message_rate(navio.ublox.CLASS_NAV, navio.ublox.MSG_NAV_CLOCK, 5)
        #ubl.configure_message_rate(navio.ublox.CLASS_NAV, navio.ublox.MSG_NAV_DGPS, 5)
        fix = False
        lat = 0.0
        lon = 0.0
        velX = 0.0
        velY = 0.0
        velZ = 0.0
        hdg = 0.0
        ground_spd = 0.0
        
        viz_points = []
        rospy.loginfo("Starting GPS")
        
        conn = False
        while not rospy.is_shutdown():
            msg = ubl.receive_message()
            if not conn:
                rospy.loginfo("First GPS message recieved")
                conn = True
            if msg is None:
                if opts.reopen:
                    ubl.close()
                    ubl = navio.ublox.UBlox("spi:0.0", baudrate=5000000, timeout=2)
                    continue
                break
            #print(msg.name())
            if msg.name() == "NAV_POSLLH":
                s = str(msg).split(",")
                s = "".join(s)
                
                lat = float(re.search('Latitude=([\d-]*)', s).group(1)) / 1e7
                lon = float(re.search('Longitude=([\d-]*)', s).group(1)) / 1e7
                
                pub.publish(GPS(lat=lat, long=lon, fix=fix, vel=Vector3(velX, velY, velZ), gndspd=ground_spd, hdg=hdg))
            if msg.name() == "NAV_STATUS":
                s = str(msg).split(",")
                s = "".join(s)
                fix = int(re.search('gpsFix=(\d)', s).group(1)) == 3
           
            if msg.name() == "NAV_VELNED":
                s = str(msg).split(",")
                s = "".join(s)
                velX = float(re.search('velN=([\d-]*)', s).group(1)) / 100
                velY = float(re.search('velE=([\d-]*)', s).group(1)) / 100
                velZ = float(re.search('velD=([\d-]*)', s).group(1)) / 100
                ground_spd = float(re.search('gSpeed=([\d-]*)', s).group(1)) / 100
                hdg = float(re.search('heading=([\d-]*)', s).group(1)) / 1e5
        
            #print(str(msg))
    except rospy.ROSInterruptException: pass
