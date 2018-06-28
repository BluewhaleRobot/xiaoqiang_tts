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
from audio_common_msgs.msg import AudioData
from engines.baidu_tts import BaiduTTS as bd_client
from engines.xunfei_tts import XunfeiTTS as xf_client
from std_msgs.msg import String
import time
import struct
import librosa
import numpy
import matplotlib.pyplot as plt
from scipy import signal

AUDIO_CACHE = AudioData()
CURRENT_AUDIO = AudioData()
NEW_AUDIO_FLAG = False
MIN_VOLUM = 2000

b, a = signal.butter(3, 0.10, 'high')


def is_end(audio_data):
    last_two_seconds = audio_data.data[2 * -16000 * 2:]
    if len(last_two_seconds) < 2 * 16000 * 2:
        return False
    last_two_seconds = [float(struct.unpack("<h", "".join(
        last_two_seconds[2*x:2*x + 2]))[0]) for x in range(0, 2 * 16000)]
    sf = signal.filtfilt(b, a, last_two_seconds)
    if numpy.max(sf) > MIN_VOLUM and len(sf[sf > MIN_VOLUM]) > 150:
        return False
    return True


def audio_duration(audio_data):
    return len(audio_data.data) / 32000.0


def is_empty(audio_data):
    raw_data = [float(struct.unpack("<h", "".join(audio_data.data[2*x:2*x + 2]))[0])
                for x in range(0, len(audio_data.data) / 2)]
    raw_data = signal.filtfilt(b, a, raw_data)

    if numpy.max(raw_data) < MIN_VOLUM:
        rospy.loginfo(numpy.max(raw_data))
        return True
    else:
        rospy.logwarn(len(raw_data[raw_data > MIN_VOLUM]))
        if len(raw_data[raw_data > MIN_VOLUM]) < 100:
            return True
        rospy.loginfo(numpy.max(raw_data))
        rospy.loginfo("Voice Found")
        return False


def unify(audio_data):
    raw_data = [float(struct.unpack("<h", "".join(audio_data.data[2*x:2*x + 2]))[0])
                for x in range(0, len(audio_data.data) / 2)]
    raw_data = numpy.array(raw_data)
    rospy.loginfo("Unify Factor: " + str(int(65535.0 / numpy.max(raw_data))))
    raw_data = int(32767.0 / numpy.max(raw_data)) * raw_data
    audio_data.data = ""
    for data in raw_data:
        if data >= 32768:
            data = 32767
        if data <= -32767:
            data = -32767
        audio_data.data += struct.pack("<h", int(data))
    return audio_data


def wav_file(sample_array, sample_rate):
    byte_count = len(sample_array)
    wav_file = ""
    # write the header
    wav_file += struct.pack('<ccccIccccccccIHHIIHH',
                            'R', 'I', 'F', 'F',
                            byte_count + 0x2c - 8,  # header size
                            'W', 'A', 'V', 'E', 'f', 'm', 't', ' ',
                            0x10,  # size of 'fmt ' header
                            1,  # format 3 = floating-point PCM
                            1,  # channels
                            sample_rate,  # samples / second
                            sample_rate * 2,  # bytes / second
                            2,  # block alignment
                            16)  # bits / sample
    wav_file += struct.pack('<ccccI',
                            'd', 'a', 't', 'a', byte_count)
    for sample in sample_array:
        # rospy.loginfo(sample)
        wav_file += struct.pack("<c", sample)
    return wav_file


if __name__ == "__main__":
    rospy.init_node("xiaoqiang_asr_node", anonymous=True)
    words_pub = rospy.Publisher("~text", String, queue_size=10)
    MIN_VOLUM = rospy.get_param("~min_volum", 2000)
    engine = rospy.get_param("~engine", "xunfei")

    # init client
    client = None
    if engine == "baidu":
        client = bd_client()
    if engine == "xunfei":
        client = xf_client()
    if client is None:
        rospy.logerr("Unknown engine {engine}".format(engine=engine))

    def process_audio(audio_data):
        AUDIO_CACHE.data += audio_data.data
        AUDIO_CACHE.data = AUDIO_CACHE.data[-60 * 16000 * 2:]

    rospy.Subscriber("~audio", AudioData, process_audio)

    while not rospy.is_shutdown():
        time.sleep(0.5)
        rospy.loginfo("AUDIO_CACHE Duration: " +
                      str(audio_duration(AUDIO_CACHE)))
        if is_end(AUDIO_CACHE) and audio_duration(AUDIO_CACHE) > 2:
            # valid audio content
            CURRENT_AUDIO.data = AUDIO_CACHE.data
            AUDIO_CACHE.data = []
            if not is_empty(CURRENT_AUDIO):
                rospy.loginfo("New content found")
                rospy.loginfo(
                    "Duration: " + str(audio_duration(CURRENT_AUDIO)))
                audio_file_name = "/home/randoms/audio_" + \
                    str(int(time.time()))
                # with open(audio_file_name, "w+") as audio_file:
                #     audio_file.write(wav_file(
                #         unify(CURRENT_AUDIO).data, 16000))
                words = String()
                words.data = client.asr(CURRENT_AUDIO)
                if len(words.data) == 0:
                    continue
                words_pub.publish(words)
                rospy.loginfo("Heared: " + words.data)
