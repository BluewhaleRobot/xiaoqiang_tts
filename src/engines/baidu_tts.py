#!/usr/bin/env python
# encoding=utf-8
# The MIT License (MIT)
#
# Copyright (c) 2018 Bluewhale Robot
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#
# Author: Randoms
#         

from aip import AipSpeech
import rospy
from audio_common_msgs.msg import AudioData

class BaiduTTS:

    def __init__(self):
        app_id = rospy.get_param("~app_id")
        app_key = rospy.get_param("~api_key")
        secret_key = rospy.get_param("~secret_key")
        timeout = int(rospy.get_param("~timeout"))

        self.client = AipSpeech(app_id, app_key, secret_key)
        self.client.setConnectionTimeoutInMillis(timeout)
        self.client.setSocketTimeoutInMillis(timeout)

    def tts(self, words):
        result  = self.client.synthesis(words, 'zh', 1, {
            'vol': 5,
        })

        if isinstance(result, dict):
            rospy.logerr(result)
            return None
        audio_data = AudioData()
        audio_data.data = result
        return audio_data

    def asr(self, audio_data):
        self.client.asr(audio_data, 'wav', 16000, {
            'dev_pid': 1536,
        })

