#!/usr/bin/env python
import rospy
import os
import re
from std_msgs.msg import String
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
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
juliaPositionMaxDuration = rospy.Duration.from_sec(10.0)

pub_cmd = None
pub_arm = None
pub_tuck = None
asoa_status_sub = None
commands_sub = None
sense_of_agency = True
pub_audio_asoa = None
pub_audio_nosoa = None
quadPos = None
lastPosTime = None
soundhandle = None
julia_cnd = None
juliaPos = None
lastJuliaTime = None
voice = 'voice_kal_diphone'
havejulia = False
speaking = False

start_re  = re.compile('^Saying.*')
finish_re  = re.compile('^Finished.*')
mute_re = re.compile('^Muted.*')
def pos_str(pos):
	if not pos:
		return "()"
	else:
		return "(" + str(pos.x) + ", " + str(pos.y) + ", " + str(pos.z) + ")"

def cpsay(string_to_say):
        global sense_of_agency
        global pub_audio_asoa
        global pub_audio_nosoa

        if sense_of_agency:
                rospy.loginfo("ASOA_SAY")
                pub_audio_asoa.publish(string_to_say)
        else:
                rospy.loginfo("NOSOA_SAY")
		msg = SoundRequest()
                msg.sound = -3
                msg.command = 1
                msg.arg = string_to_say
                msg.arg2 = 'voice_kal_diphone'
                pub_audio_nosoa.publish(msg)


# Going to try not using this for now!
def asoa_status_callback(data):
        global speaking
        global start_re, finish_re
        global pub_audio_nosoa
        global sense_of_agency
        
        status_cmd = data.data
        match = mute_re.match(status_cmd)
        if match:
                rospy.loginfo("Muted mic!")
                # Cheating for now
                speaking = True
                rospy.sleep(7)
                return
                speaking = False
		msg = SoundRequest()
                msg.sound = -3
                msg.command = 1
                msg.arg = "I can't hear myself.  Is my microphone muted?"
                msg.arg2 = 'voice_kal_diphone'
                pub_audio_nosoa.publish(msg)

        match = start_re.match(status_cmd)
        if match:
                speaking = True
                rospy.loginfo("Setting speaking to true.")

        match = finish_re.match(status_cmd)                
        if match:
                speaking = False
                rospy.loginfo("Setting speaking to false.")
        
def cmd_callback(data):
	global juliaPos
        global speaking
        global commands_sub
        global asoa_status_sub
        global sense_of_agency
        rospy.loginfo("got " + data.data)
        if speaking:
                rospy.loginfo("Still talking!")
                return
        else:
                rospy.loginfo("Not talking!")
	cmd = str(data.data).strip()
	if cmd == "hello baxter":
		rospy.loginfo("hello commander")
                cpsay("Hello Human.  I heard you say Hello Baxter")
                rospy.sleep(10)
        elif "know yourself" in cmd:
                rospy.loginfo("Turning on auditory sense of agency.")
                #asoa_status_sub = rospy.Subscriber("asoa_status", String, asoa_status_callback)
                commands_sub.unregister()
                commands_sub = rospy.Subscriber("cmds_received_asoa", String, cmd_callback)
                cpsay("Turned on sense of agency.")
                sense_of_agency = True
        elif "forget yourself" in cmd:
                speaking = False
                rospy.loginfo("Turning off auditory sense of agency.")
                #asoa_status_sub.unregister()
                commands_sub.unregister()
                commands_sub = rospy.Subscriber("cmds_received_nosoa", String, cmd_callback)
                cpsay("Turned off sense of agency.")
                sense_of_agency = False
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
                                cpsay("I see the quad and am pointing to the quad")
                                pub_cmd.publish(quadPos)
				rospy.loginfo("Speech command: I see the quad and am pointing - sending command to point to " + 
				pos_str(quadPos))
			else:
                                cpsay('I no longer can see the quad')
                                rospy.loginfo("Speech command: I no longer can see the quad - last quad position is outdated: recorded at " + 
				pos_str(quadPos) + str(round((t - lastPosTime).to_sec(), 2))
				+ " seconds ago")
		else:
			cpsay('I do not see the quad')
			rospy.loginfo("No known quad location")
	elif cmd == "turn on the quad":
                cpsay('I do not know how to do that')
		rospy.loginfo("Speech command: I do not know how to start quad")
	elif cmd == "move your right arm to the side":
                cpsay('moving my right arm to the side')
                amsg = String()
                amsg.data = "right_arm_side"
                pub_arm.publish(amsg)
		rospy.loginfo("Speech command: moving your right arm to the side")
	elif cmd == "move your left arm to the side":
                cpsay('moving my left arm to the side')
                amsg = String()
                amsg.data = "left_arm_side"
                pub_arm.publish(amsg)
		rospy.loginfo("Speech command: moving my left arm to the side")
	elif cmd == "move both of your arms to the side":
                cpsay('moving my arms to the side')
                amsg = String()
                amsg.data = "both_arms_side"
                pub_arm.publish(amsg)
		rospy.loginfo("Speech command: moving both of my arms to the side")
	elif cmd == "tuck your arms":
                cpsay('tucking my arms')
                amsg = String()
                amsg.data = "True"
                pub_tuck.publish(amsg)
		rospy.loginfo("Speech command: tucking arms")
	elif cmd == "untuck your arms":
                cpsay( 'untucking my arms')
                amsg = String()
                amsg.data = "False"
                pub_tuck.publish(amsg)
		rospy.loginfo("Speech command:untucking arms")
	elif cmd == "raise the quad":
                cpsay('I do not know how to do that')
                rospy.loginfo("I do not know how to raise quad")
	elif cmd == "lower the quad":
                cpsay('I do not know how to do that')
                rospy.loginfo("Speech command: I do not know how to lower quad")
	elif cmd == "land the quad":
                cpsay('I do not know how to do that')	
                rospy.loginfo("Speech command: I do not know how to land quad")
	elif cmd == "turn off the quad":
                cpsay('I do not know how to do that')	
                rospy.loginfo("Speech command: I do not know how to land quad")
                rospy.loginfo("Speech command: I do not know how to turn off quad")
	elif cmd == "how are you":
                cpsay('I am fine')
                rospy.loginfo("Speech command: I am fine")
	elif cmd == "how are you doing":
                cpsay( 'I am fine')
                rospy.loginfo("Speech command: I am fine")
	elif "julia" in cmd and (not speaking):
		t = rospy.get_rostime()
                if havejulia and lastJuliaTime != 0 and t - lastJuliaTime < juliaPositionMaxDuration:
                    pub_cmd.publish(juliaPos)
                    cpsay('Julia is over there')
                    rospy.sleep(2)
                    cpsay('I am pointing at Julia.')
                    rospy.loginfo("Speech command: Julia is over there, pointing to her")
                else:
                    cpsay('I am having trouble with my vision and am not sure that I see Julia')
                    rospy.sleep(10)
                    rospy.loginfo("Speech command: Not sure I see Julia")
	else:
                cpsay('I did not understand that command')
                rospy.loginfo("command " + cmd + " not understood")
	

def quad_pos_callback(data):
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

def julia_callback(data):
	global juliaPos
        global havejulia
        global lastJuliaTime
        for marker in data.markers:
	    if marker.id==10 or marker.id==12 or marker.id==20:
                juliaPos = Point() 
                juliaPos.x = marker.pose.pose.position.x
                juliaPos.y = marker.pose.pose.position.y
                juliaPos.z = marker.pose.pose.position.z
                havejulia = True
                lastJuliaTime = rospy.get_rostime()
                break

def start_node():
        global asoa_status_sub
        global commands_sub

	rospy.init_node('command_node')
        #global soundhandle
        #soundhandle = SoundClient()
        rospy.sleep(1)
	#subscribe to channels reporting commands and the quadrotor's position
	commands_sub = rospy.Subscriber("cmds_received_asoa", String, cmd_callback)
	rospy.Subscriber("quad_pos", PointStamped, quad_pos_callback)
        rospy.Subscriber("ar_pose_marker", AlvarMarkers, julia_callback)
        # asoa_status_sub = rospy.Subscriber("asoa_status", String, asoa_status_callback)

	#start a publisher to the channel where pointing cmds are sent
	global pub_audio_asoa
	global pub_audio_nosoa
	global pub_cmd
	global pub_tuck
	global pub_arm
	pub_cmd = rospy.Publisher('/point_cmd', Point, queue_size=10)
	pub_arm = rospy.Publisher('/arm_cmd', String, queue_size=10)
        pub_tuck = rospy.Publisher('/tuck_cmd', String, queue_size=10)
        pub_audio_asoa = rospy.Publisher('/asoa_say_cmd', String, queue_size=10)
        pub_audio_nosoa = rospy.Publisher('robotsound', SoundRequest, queue_size=10)
	rospy.spin()

if __name__ == "__main__":
	start_node()
