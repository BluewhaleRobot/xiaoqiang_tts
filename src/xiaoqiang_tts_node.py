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

import rospy
import time
from engines.baidu_tts import BaiduTTS as bd_client
from engines.xunfei_tts import XunfeiTTS as xf_client
from audio_common_msgs.msg import AudioData
from std_msgs.msg import String

if __name__ == "__main__":
    rospy.init_node("xiaoqiang_tts", anonymous=True)
    audio_pub = rospy.Publisher("/audio", AudioData, queue_size=10)
    engine = rospy.get_param("~engine", "xunfei")

    # init client
    client = None
    if engine == "baidu":
        client = bd_client()
    if engine == "xunfei":
        client = xf_client()
    if client is None:
        rospy.logerr("Unknown engine {engine}".format(engine=engine))

    # set text sub
    processing_flag = False
    def text_cb(text):
        global processing_flag
        if processing_flag:
            return
        processing_flag = True
        audio_data = client.tts(text.data)
        if audio_data is not None:
            audio_pub.publish(audio_data)
        processing_flag = False

    text_sub = rospy.Subscriber("~text", String, text_cb)
    while not rospy.is_shutdown():
        time.sleep(1)