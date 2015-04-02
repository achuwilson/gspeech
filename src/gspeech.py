#!/usr/bin/env python
# -*- coding: utf-8 -*-
#########################################################################################
#                                    _                                                  #
#      __ _ ___ _ __   ___  ___  ___| |__                                               #
#     / _` / __| '_ \ / _ \/ _ \/ __| '_ \                                              #
#    | (_| \__ \ |_) |  __/  __/ (__| | | |                                             #
#     \__, |___/ .__/ \___|\___|\___|_| |_|                                             #
#     |___/    |_|                                                                      #
#                                                                                       #
# ros package for speech recognition using Google Speech API                            #
# run using 'rosrun gspeech gspeech.py'                                                 #
# it creats and runs a node named gspeech                                               #
# the node gspeech publishes two topics- /speech and /confidence                        #
# the topic /speech contains the reconized speech string                                #
# the topic /confidence contains the confidence level in percentage of the recognization#
#                                                                                       #
#                                                                                       #
# UPDATE: for key generation look http://www.chromium.org/developers/how-tos/api-keys   #
#         at the revision date, each key allows your node to make up to 50 request      #
#         change in the cmd2 at the end of the string "yourkey" for your key            #
#                                                                                       #
# written by achuwilson                                                                 #
# revision by pexison                                                                   #
#                                                                                       #
# 30-06-2012 , 3.00pm                                                                   #
# achu@achuwilson.in                                                                    #
# 01-04-2015 , 11:00am                                                                  #
# pexison@gmail.com                                                                     #
#########################################################################################

import json, shlex, socket, subprocess, sys, threading
import roslib; roslib.load_manifest('gspeech')
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int8
<<<<<<< HEAD
import shlex,subprocess,os
cmd1='sox -r 16000 -t alsa default recording.flac silence 1 0.1 1% 1 1.5 1%'
cmd2='wget -q -U "Mozilla/5.0" --post-file recording.flac --header="Content-Type: audio/x-flac; rate=16000" -O - "https://www.google.com/speech-api/v2/recognize?output=json&lang=en-us&key=yourkey"'
=======
from std_srvs.srv import *


class GSpeech(object):
    """Speech Recogniser using Google Speech API"""

    def __init__(self, _api_key):
        """Constructor"""
        # configure system commands
        self.api_key = _api_key
        self.sox_cmd = "sox -r 44100 -t alsa default recording.flac silence 1 0.1 1% 1 1.5 1%"
        self.wget_cmd = ("wget -q -U \"Mozilla/5.0\" ") + \
            ("--post-file recording.flac ") + \
            ("--header=\"Content-Type: audio/x-flac; rate=44100\" -O - ") + \
            ("\"https://www.google.com/speech-api/v2/recognize") + \
            ("?output=json&lang=en-us&key={api_key}\"")
        self.wget_cmd = self.wget_cmd.format(api_key=self.api_key)
        self.sox_args = shlex.split(self.sox_cmd)
        self.wget_args = shlex.split(self.wget_cmd)
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
        self.recog_thread = threading.Thread(target=self.do_recognition, args=())
        self.recog_thread.start()

    def start(self, req):
        """Start speech recognition"""
        if not self.started:
            self.started = True
            if not self.recog_thread.is_alive():
                self.recog_thread = threading.Thread(
                    target=self.do_recognition, args=()
                )
                self.recog_thread.start()
            rospy.loginfo("gspeech recognizer started")
        else:
            rospy.loginfo("gspeech is already running")
        return EmptyResponse()
>>>>>>> e6a999073b0a13e9398bd0fd24228e559b1210e8

    def stop(self, req):
        """Stop speech recognition"""
        if self.started:
            self.started = False
            if self.recog_thread.is_alive():
                self.recog_thread.join()
            rospy.loginfo("gspeech recognizer stopped")
        else:
            rospy.loginfo("gspeech is already stopped")
        return EmptyResponse()

<<<<<<< HEAD
def speech():
  rospy.init_node('gspeech')
  pubs = rospy.Publisher('speech', String)
  pubc = rospy.Publisher('confidence', Int8)
  args2 = shlex.split(cmd2)
  os.system('sox -r 16000 -t alsa default recording.flac silence 1 0.1 1% 1 1.5 1%')    
  output,error = subprocess.Popen(args2,stdout = subprocess.PIPE, stderr= subprocess.PIPE).communicate()
      
  if not error and len(output)>16:
    a = eval(output)
    confidence= a['hypotheses'][0]['confidence']
    confidence= confidence*100
    data=a['hypotheses'][0]['utterance']
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
   
=======
    def shutdown(self):
        """Stop all system process before killing node"""
        self.started = False
        if self.recog_thread.is_alive():
            self.recog_thread.join()
        self.srv_start.shutdown()
        self.srv_stop.shutdown()

    def do_recognition(self):
        """Do speech recognition"""
        while self.started:
            sox_p = subprocess.call(self.sox_args)
            wget_out, wget_err = subprocess.Popen(
                self.wget_args, stdout=subprocess.PIPE, stderr=subprocess.PIPE
            ).communicate()
            if not wget_err and len(wget_out) > 16:
                wget_out = wget_out.split('\n', 1)[1]
                a = json.loads(wget_out)['result'][0]
                if 'confidence' in a['alternative'][0]:
                    confidence = a['alternative'][0]['confidence']
                    confidence = confidence * 100
                    self.pub_confidence.publish(confidence)
                    rospy.loginfo("confidence: {}".format(confidence))
                if 'transcript' in a['alternative'][0]:
                    data = a['alternative'][0]['transcript']
                    self.pub_speech.publish(String(data))
                    rospy.loginfo(String(data))


def is_connected():
    """Check if connected to Internet"""
    try:
        # check if DNS can resolve hostname
        remote_host = socket.gethostbyname("www.google.com")
        # check if host is reachable
        s = socket.create_connection(address=(remote_host, 80), timeout=5)
        return True
    except:
        pass
    return False


def usage():
    """Print Usage"""
    print("Usage:")
    print("rosrun gspeech gspeech.py <API_KEY>")


def main():
    if len(sys.argv) < 2:
        usage()
        sys.exit("No API_KEY provided")
    if not is_connected():
        sys.exit("No Internet connection available")
    api_key = str(sys.argv[1])
    speech = GSpeech(api_key)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        sys.exit(0)


>>>>>>> e6a999073b0a13e9398bd0fd24228e559b1210e8
