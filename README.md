gspeech
=======

ROS package for speech recognition based on Google Speech API



NOTE: This package needs a key to use the google speech to text API, each key can be used
      for 50 times (you can make 50 request per key to the google server), and each
      request is accepted if the audio duration is less than 15 seconds.

      For key generation see: http://www.chromium.org/developers/how-tos/api-keys
      Dont forget to add your key before using the gspeech node. 
NOTE: This package does not work now, seems that Google changed their APIs/policies or authentication  methods

UPD: Fixed, now working with API v2, you need an API Key from  Google Developer Console


Usage
-----
`rosrun gspeech gspeech.py <API_KEY>`
