#!/usr/bin/env python
import rospy
import os
from std_msgs.msg import String
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

#will not use position reports received further in the past.
#To disable this behavior, simply set the value to float('inf')
CURRENT_POSITION_MAX_LAG = 2.0

#convert max delay time to ROS Duration
currentPositionMaxDuration = rospy.Duration.from_sec(CURRENT_POSITION_MAX_LAG)

pub_cmd = None
pub_arm = None
pub_tuck = None
pub_audio = None
quadPos = None
lastPosTime = None
soundhandle = None
voice = 'voice_kal_diphone'

def pos_str(pos):
	if not pos:
		return "()"
	else:
		return "(" + str(pos.x) + ", " + str(pos.y) + ", " + str(pos.z) + ")"

def cmd_callback(data):
	rospy.loginfo("got " + data.data)
        #global voice
        #global soundhandle
	cmd = str(data.data).strip()
	if cmd == "hello baxter":
		rospy.loginfo("hello commander")
		msg = SoundRequest()
                msg.sound = -3
                msg.command = 1
                msg.arg = 'Hello Human.  I heard you say Hello Baxter'
                msg.arg2 = 'voice_kal_diphone'
                pub_audio.publish(msg)
                #os.system("espeak -v en 'hello commander'") #speaker?
	elif cmd == "point to the quad":
		'''
		if the quadrotor's positon is being published, its last known
		position should be stored in quadPos. If quadPos is None, no
		position has been reported.
		'''
		if quadPos:
			t = rospy.get_rostime()
			rospy.loginfo(str(t) + "," + str(lastPosTime))
			if lastPosTime != 0 and t - lastPosTime < currentPositionMaxDuration:
				msg = SoundRequest()
                                msg.sound = -3
                                msg.command = 1
                                msg.arg = 'I see the quad and am pointing to the quad'
                                msg.arg2 = 'voice_kal_diphone'
                                pub_audio.publish(msg)
                                pub_cmd.publish(quadPos)
				rospy.loginfo("sending command to point to " + 
				pos_str(quadPos))
			else:
				msg = SoundRequest()
                                msg.sound = -3
                                msg.command = 1
                                msg.arg = 'I do not see the quad'
                                msg.arg2 = 'voice_kal_diphone'
                                pub_audio.publish(msg)
                                rospy.loginfo("Last quad position is outdated: recorded at " + 
				pos_str(quadPos) + str(round((t - lastPosTime).to_sec(), 2))
				+ " seconds ago")
		else:
			msg = SoundRequest()
                        msg.sound = -3
                        msg.command = 1
                        msg.arg = 'I do not see the quad'
                        msg.arg2 = 'voice_kal_diphone'
                        pub_audio.publish(msg)
                        rospy.loginfo("No known quad location")
	elif cmd == "turn on the quad":
                msg = SoundRequest()
                msg.sound = -3
                msg.command = 1
                msg.arg = 'I do not know how to do that'
                msg.arg2 = 'voice_kal_diphone'
                pub_audio.publish(msg)
		rospy.loginfo("I do not know how to start quad")
	elif cmd == "move your right arm to the side":
                msg = SoundRequest()
                msg.sound = -3
                msg.command = 1
                msg.arg = 'moving right arm to the side'
                msg.arg2 = 'voice_kal_diphone'
                pub_audio.publish(msg)
                amsg = String()
                amsg.data = "right_arm_side"
                pub_arm.publish(amsg)
		rospy.loginfo("move your right arm to the side")
	elif cmd == "move your left arm to the side":
                msg = SoundRequest()
                msg.sound = -3
                msg.command = 1
                msg.arg = 'moving right arm to the side'
                msg.arg2 = 'voice_kal_diphone'
                pub_audio.publish(msg)
                amsg = String()
                amsg.data = "left_arm_side"
                pub_arm.publish(amsg)
		rospy.loginfo("move your left arm to the side")
	elif cmd == "move both of your arms to the side":
                msg = SoundRequest()
                msg.sound = -3
                msg.command = 1
                msg.arg = 'moving right arm to the side'
                msg.arg2 = 'voice_kal_diphone'
                pub_audio.publish(msg)
                amsg = String()
                amsg.data = "both_arms_side"
                pub_arm.publish(amsg)
		rospy.loginfo("move your left arm to the side")
	elif cmd == "tuck your arms":
                msg = SoundRequest()
                msg.sound = -3
                msg.command = 1
                msg.arg = 'tucking my arms'
                msg.arg2 = 'voice_kal_diphone'
                pub_audio.publish(msg)
                amsg = String()
                amsg.data = "True"
                pub_tuck.publish(amsg)
		rospy.loginfo("tucking arms")
	elif cmd == "untuck your arms":
                msg = SoundRequest()
                msg.sound = -3
                msg.command = 1
                msg.arg = 'untucking my arms'
                msg.arg2 = 'voice_kal_diphone'
                pub_audio.publish(msg)
                amsg = String()
                amsg.data = "False"
                pub_tuck.publish(amsg)
		rospy.loginfo("untucking arms")
	elif cmd == "raise the quad":
		msg = SoundRequest()
                msg.sound = -3
                msg.command = 1
                msg.arg = 'I do not know how to do that'
                msg.arg2 = 'voice_kal_diphone'
                rospy.loginfo("I do not know how to raise quad")
	elif cmd == "lower the quad":
                msg = SoundRequest()
                msg.sound = -3
                msg.command = 1
                msg.arg = 'I do not know how to do that'
                msg.arg2 = 'voice_kal_diphone'		
                rospy.loginfo("I do not know how to lower quad")
	elif cmd == "land the quad":
                msg = SoundRequest()
                msg.sound = -3
                msg.command = 1
                msg.arg = 'I do not know how to do that'
                msg.arg2 = 'voice_kal_diphone'		
                rospy.loginfo("I do not know how to land quad")
	elif cmd == "turn off the quad":
		msg = SoundRequest()
                msg.sound = -3
                msg.command = 1
                msg.arg = 'I do not know how to do that'
                msg.arg2 = 'voice_kal_diphone'
                rospy.loginfo("I do not know how to turn off quad")
	else:
		msg = SoundRequest()
                msg.sound = -3
                msg.command = 1
                msg.arg = 'I did not understand that command'
                msg.arg2 = 'voice_kal_diphone'
                rospy.loginfo("command " + cmd + " not understood")

def quad_pos_callback(data):
	global quadPos
        t = rospy.get_rostime()
	p = data.point #convert from PointStamped
	global quadPos, lastPosTime
	if not p or p.x == 0 and p.y == 0 and p.z == 0:
		
                rospy.loginfo("Received a null location or origin. Erasing previous \
		quad location.")
		quadPos = None
		lastPosTime = t
	else:		
		#rospy.loginfo("setting last known quad position to " + 
		#pos_str(p) + ". t =" + str(round(t.to_sec(), 2)))
		quadPos = p
		lastPosTime = t
	
def start_node():
	rospy.init_node('command_node')
        global soundhandle
        soundhandle = SoundClient()
        rospy.sleep(1)
	#subscribe to channels reporting commands and the quadrotor's position
	rospy.Subscriber("cmds_received", String, cmd_callback)
	rospy.Subscriber("quad_pos", PointStamped, quad_pos_callback)
	#start a publisher to the channel where pointing cmds are sent
	global pub_audio
	global pub_cmd
        global pub_arm
        global pub_tuck
        pub_cmd = rospy.Publisher('/point_cmd', Point, queue_size=10)
	pub_arm = rospy.Publisher('/arm_cmd', String, queue_size=10)
        pub_tuck = rospy.Publisher('/tuck_cmd', String, queue_size=10)
        pub_audio = rospy.Publisher('robotsound', SoundRequest, queue_size=10)
	rospy.spin()

if __name__ == "__main__":
	start_node()
