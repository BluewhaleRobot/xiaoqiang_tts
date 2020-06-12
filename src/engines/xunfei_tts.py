#!/usr/bin/env python3
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

from xiaoqiang_tts import _xunfei_tts
import rospy
from audio_common_msgs.msg import AudioData
import time


class XunfeiTTS:

    def __init__(self):
        self.app_id = rospy.get_param("~app_id", "5b2efdad")
        self.timeout = rospy.get_param("~timeout", 5000)
        self.user_dict = rospy.get_param("~dict", "")

    def tts(self, words):
        audio_data = AudioData()
        audio_data.data = _xunfei_tts.tts(words, self.app_id)
        return audio_data

    def asr(self, audio_data):
        res = _xunfei_tts.asr(
            [ord(x) for x in audio_data.data], self.app_id, self.user_dict, self.timeout)
        return res
