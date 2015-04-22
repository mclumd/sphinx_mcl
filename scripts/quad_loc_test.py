from __future__ import print_function
import time
import rospy
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import String

points = [Point(x = 5.0, y = 0.0, z = 0.0),
		  Point(x = 8.0, y = 7.0, z = 2.0),
		  Point(x = 7.0, y = -4.0, z = -0.5)]

def send_rotating_points(nPoints, delay):
	rospy.init_node('quad_pos_test1')
	pub = rospy.Publisher('/quad_position', PointStamped, queue_size=10)
	
	n = 0
	while n < nPoints:
		n += 1
		time.sleep(delay)
		p = points[n % 3]
		print("Sending point command:", p)
		pub.publish(PointStamped(point = p))
	rospy.spin()

def full_pointing_test():
	'''
	Sends Baxter points, tells it to point to them. Also tests sending
	null points and the origin, which should both cancel previous loc
	messages, and maximum time before a location is 'forgotten'"
	'''
	rospy.init_node('quad_pos_test2')
	pub = rospy.Publisher('/quad_position', PointStamped, queue_size=10)
	cmdPub = rospy.Publisher('cmds_received', String, queue_size=10)

	time.sleep(2)
	p = points[0]
	print("Sending location", p)
	pub.publish(PointStamped(point = p))
	time.sleep(0.5)
	print("sending point command")
	cmdPub.publish(String("point to the quad"))
	time.sleep(1.0)
	p = points[1]
	print("Sending location", p)
	pub.publish(PointStamped(point = p))
	time.sleep(1.0)
	print("sending point command")
	cmdPub.publish(String("point to the quad"))
	time.sleep(0.5)
	p = points[2]
	print("Sending location", p)
	pub.publish(PointStamped(point = p))
	time.sleep(3.0)
	print("sending point command - failure expected due to time elapsed")
	cmdPub.publish(String("point to the quad"))
	time.sleep(1.0)
	p = points[2]
	print("Sending location", p)
	pub.publish(PointStamped(point = p))
	time.sleep(0.5)
	p  = Point(x = 0.0, y = 0.0, z = 0.0)
	print("Sending origin to cancel", p)
	pub.publish(PointStamped(point = p))
	time.sleep(0.5)
	print("sending point command - failure expected due to cancel")
	cmdPub.publish(String("point to the quad"))
	time.sleep(1.0)
	rospy.spin()
	
	

if __name__ == "__main__":
	#send_rotating_points(20, 2.0)
	full_pointing_test()
	
	
	
