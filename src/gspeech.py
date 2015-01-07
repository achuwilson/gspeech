#!/usr/bin/env python
# -*- coding: utf-8 -*-
#########################################################################################
#	                                _     						#
#	  __ _ ___ _ __   ___  ___  ___| |__  						#
#	 / _` / __| '_ \ / _ \/ _ \/ __| '_ \ 						#
#	| (_| \__ \ |_) |  __/  __/ (__| | | |						#
#	 \__, |___/ .__/ \___|\___|\___|_| |_|						#
#	 |___/    |_|                         						#
#											#
# ros package for speech recognition using Google Speech API				#
# run using 'rosrun gspeech gspeech.py'							#
# it creats and runs a node named gspeech						#
# the node gspeech publishes two topics- /speech and /confidence			#
# the topic /speech contains the reconized speech string				#
# the topic /confidence contains the confidence level in percentage of the recognization#
#											#
#											#
# written by achuwilson									#
# 30-06-2012 , 3.00pm									#
# achu@achuwilson.in									#
#########################################################################################
import roslib; roslib.load_manifest('gspeech') 
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int8
import shlex,subprocess,os,sys,json
api_key = "" # PASTE HERE YOUR GOOGLE API KEY
cmd1='sox -r 44100 -t alsa default recording.flac silence 1 0.1 1% 1 1.5 1%'
cmd2='wget -q -U "Mozilla/5.0" --post-file recording.flac --header="Content-Type: audio/x-flac; rate=44100" -O - "https://www.google.com/speech-api/v2/recognize?output=json&lang=en-us&key='+api_key+'"'

def speech():
	rospy.init_node('gspeech')
	pubs = rospy.Publisher('speech', String)
	pubc = rospy.Publisher('confidence', Int8)
	
	
	args2 = shlex.split(cmd2)
	
	os.system(cmd1)	
	output,error = subprocess.Popen(args2,stdout = subprocess.PIPE, stderr= subprocess.PIPE).communicate()
		
	if not error and len(output)>16:
		# print(output)
		output = output.split('\n', 1)[1]
		a = json.loads(output)['result'][0]
		confidence= a['alternative'][0]['confidence']
		confidence= confidence*100
		data=a['alternative'][0]['transcript']
		pubs.publish(String(data))
		pubc.publish(confidence)
		print String(data), confidence
	
	speech()	
	






if __name__ == '__main__':
	try:
		speech()
	except rospy.ROSInterruptException: pass
	except KeyboardInterrupt:
		sys.exit(1)
   