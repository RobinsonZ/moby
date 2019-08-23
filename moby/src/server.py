#!/usr/bin/python
import roslib
roslib.load_manifest("moby")
import sys, socket, struct, rospy
from moby.msg import PWMCommand

ENABLE_FLAG = 0b10000000

if __name__ == '__main__':
    rospy.init_node("navio_server")
    
    port = rospy.get_param("~port", 5001)
    timeout = rospy.get_param("~recv_timeout", 0.1)

    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('', port))
    sock.settimeout(timeout)

    conn = False
    enable = False
    last_seq = 0

    pub = rospy.Publisher("navpwm", PWMCommand)

    while not rospy.is_shutdown():
        try:
            packet, addr = sock.recvfrom(128) # the packet is actually 121 bytes, but next power of 2
            data = struct.unpack("!qB14d", packet)

            seq = data[0]
            cmd = data[1]
            pwm_throts = data[2:]
            if seq > last_seq:
                last_seq = seq

                if not conn:
                    print >> sys.stderr, "Connected from " + str(addr)
                    conn = True

                if ENABLE_FLAG & cmd != 0:
                    if not enable:
                        print >> sys.stderr, "!!!! ENABLED !!!!"
                        enable = True
                    
                    pub.publish(PWMCommand(duty_cycles=pwm_throts))
                    

                else:
                    if enable:
                        print >> sys.stderr, "Disabling"
                        enable = False
                        pub.publish(PWMCommand(duty_cycles=(-1,)*14))

            else:
                print >> sys.stder, "Invalid sequence number " + str(seq) + ", should be greater than " + str(last_seq)
                conn = False
                enable = False
        except socket.timeout:
            if conn:
                print >> sys.stderr, "Did not receive packet in time, disabling outputs"
                conn = False
                enable = False
