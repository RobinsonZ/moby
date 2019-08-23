#!/usr/bin/python
import roslib
roslib.load_manifest("moby")
import rospy, sys, serial, struct, binascii, base64
from moby.msg import ArduinoSensors

def ardu_sensors():
	ino = serial.Serial(rospy.get_param("~address", "/dev/ttyACM0"), rospy.get_param("~baud", 115200))
	ino.readline()

	pub = rospy.Publisher("ardusensors", ArduinoSensors)

	while not rospy.is_shutdown():
		b64 = ino.readline()
		try:
			data = bytearray(binascii.a2b_base64(b64))

			# checksum
			cksum = sum(data[:-1]) % 256
		
			dios = [False,] * 11 
			byte0mask = (0b00000000, 0b00000000, 0b00000100, 0b00000010, 0b00000001)
			for i in range(2,5):
				dios[i - 2] = (data[0] &  byte0mask[i] == byte0mask[i])
			for i in range(8):
				dios[i + 3] = (data[1] >> (7 - i)  & 1 == 1)
		
			analogs = [0,] * 6
			other_data = struct.unpack(">6hc", data[2:])
			for i in range(6):
				analogs[i] = other_data[i]
		
			real_cksum = ord(other_data[6])
		
			if cksum != real_cksum:
				print >> sys.stderr, "Checksum error!"
			else:
				pub.publish(ArduinoSensors(digital=dios, analog=analogs))
		except (binascii.Error, IndexError) as err:
			print >> sys.stderr, "Read error (" + type(err).__name__ + ": " + str(err) + ") on packet " + str(b64) + ", retrying"

if __name__ == "__main__":
    rospy.init_node("arduino_serial")
    try:
        ardu_sensors()
    except rospy.ROSInterruptException: pass
