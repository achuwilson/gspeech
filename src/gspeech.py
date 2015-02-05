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

import shlex, subprocess, os, sys, json
import roslib; roslib.load_manifest('gspeech')
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int8
from std_srvs.srv import *


class GSpeech(object):
    """Speech Recogniser using Google Speech API"""

    def __init__(self, _api_key):
        # configure system commands
        self.api_key = _api_key
        self.sox_cmd = "sox -r 44100 -t alsa default recording.flac silence 1 0.1 1% 1 1.5 1%"
        self.wget_cmd = ("wget -q -U \"Mozilla/5.0\" ") + \
            ("--post-file recording.flac ") + \
            ("--header=\"Content-Type: audio/x-flac; rate=44100\" -O - ") + \
            ("\"https://www.google.com/speech-api/v2/recognize") + \
            ("?output=json&lang=en-us&key={api_key}\"")
        self.wget_cmd = self.wget_cmd.format(api_key=self.api_key)
        # start ROS node
        rospy.init_node('gspeech')
        # configure ROS settings
        rospy.on_shutdown(self.shutdown)
        self.pub_speech = rospy.Publisher('~speech', String)
        self.pub_confidence = rospy.Publisher('~confidence', Int8)
        self.srv_start = rospy.Service('~start', Empty, self.start)
        self.srv_stop = rospy.Service('~stop', Empty, self.stop)
        # run speech recognition
        self.started = True
        self.do_gspeech()

    def start(self):
        """Start speech recognition"""
        self.started = True
        rospy.loginfo("gspeech recognizer started")
        return EmptyResponse()

    def stop(self):
        """Stop speech recognition"""
        self.started = False
        rospy.loginfo("gspeech recognizer stopped")
        return EmptyResponse()

    def shutdown(self):
        """Stop all system process and before killing node"""
        self.srv_start.shutdown()
        self.srv_stop.shutdown()

    def do_gspeech(self):
        args2 = shlex.split(self.wget_cmd)
        os.system(self.sox_cmd)
        output, error = subprocess.Popen(
            args2, stdout = subprocess.PIPE, stderr = subprocess.PIPE
        ).communicate()
        if not error and len(output) > 16:
            output = output.split('\n', 1)[1]
            a = json.loads(output)['result'][0]
            if 'confidence' in a['alternative'][0]:
                confidence = a['alternative'][0]['confidence']
                confidence = confidence * 100
                self.pub_confidence.publish(confidence)
                rospy.loginfo("confidence: {}".format(confidence))
            if 'transcript' in a['alternative'][0]:
                data = a['alternative'][0]['transcript']
                self.pub_speech.publish(String(data))
                rospy.loginfo(String(data))
        if self.started:
            self.do_speech()


def main():
    api_key = ""
    speech = GSpeech(api_key)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        sys.exit(0)


