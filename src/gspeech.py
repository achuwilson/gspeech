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
# run using 'rosrun gspeech gspeech.py <your api key>'                                  #
# it creates and runs a node named gspeech                                              #
# the node gspeech publishes two topics- /speech and /confidence                        #
# the topic /speech contains the reconized speech string                                #
# the topic /confidence contains the confidence level in percentage of the recognization#
# the node gspeech registers services start and stop                                    #
# For dependencies use: pip install --upgrade google-cloud; sudo apt-get install sox    #
# For authentication do: export GOOGLE_APPLICATION_CREDENTIALS=<serviceaccount.json>    #
#                                                                                       #
# written by achuwilson                                                                 #
# revision by pexison,                                                                  #
# revision by slesinger - removing redundant code, updating to latest Google API         #
#                                                                                       #
# 30-06-2012 , 3.00pm                                                                   #
# achu@achuwilson.in                                                                    #
# 01-04-2015 , 11:00am                                                                  #
# pexison@gmail.com                                                                     #
# 17-08-2017                                                                            #
# slesinger@gmail.com                                                                   #
#########################################################################################

import json, shlex, socket, subprocess, sys, threading
import roslib; roslib.load_manifest('gspeech')
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int8
import shlex,subprocess,os,io
from std_srvs.srv import *
from google.cloud import speech
from google.cloud.speech import enums
from google.cloud.speech import types

class GSpeech(object):
  """Speech Recogniser using Google Speech API"""

  def __init__(self):
    """Constructor"""
    # configure system commands
    self.sox_cmd = "sox -r 16000 -c 1 -t alsa default recording.flac silence 1 0.1 1% 1 1.5 1%"
    self.sox_args = shlex.split(self.sox_cmd)
    self.client = speech.SpeechClient()
    # start ROS node
    rospy.init_node('gspeech')
    # configure ROS settings
    rospy.on_shutdown(self.shutdown)
    self.pub_speech = rospy.Publisher('~speech', String, queue_size=10)
    self.pub_confidence = rospy.Publisher('~confidence', Int8, queue_size=10)
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

  def shutdown(self):
    """Stop all system process before killing node"""
    self.started = False
    if self.recog_thread.is_alive():
      self.recog_thread.join()
    self.srv_start.shutdown()
    self.srv_stop.shutdown()
    os.remove("recording.flac")

  def do_recognition(self):
    """Do speech recognition"""
    while self.started:
      sox_p = subprocess.call(self.sox_args)
      with io.open("recording.flac", 'rb') as audio_file:
        content = audio_file.read()
        audio = types.RecognitionAudio(content=content)

      config = types.RecognitionConfig(
        encoding=enums.RecognitionConfig.AudioEncoding.FLAC,
        sample_rate_hertz=16000,
        language_code='en-US')

      response = self.client.recognize(config, audio)
      print type(response)
      alternatives = response.results[0].alternatives
      for alt in alternatives:
        confidence = alt.confidence * 100
        self.pub_confidence.publish(confidence)
        rospy.loginfo("confidence: {}".format(confidence))
        self.pub_speech.publish(String(alt.transcript))
        rospy.loginfo("transcript: {}".format(alt.transcript))

# end of GSpeech class




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
  if not is_connected():
    sys.exit("No Internet connection available")
  speech = GSpeech()
  rospy.spin()


if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass
  except KeyboardInterrupt:
    sys.exit(0)
