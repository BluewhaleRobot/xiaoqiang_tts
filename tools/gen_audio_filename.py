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

"""
根据语音内容计算语音文件名，用户可以替换对应文件名的文件实现语音的替换。可以用于多语言支持
"""

import hashlib
import sys

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("用法\n gen_audio_filename.py [语音文字内容]")
        print("比如\n gen_audio_filename.py '这是一段语音' ")
        exit(0)
    text = sys.argv[1]
    m = hashlib.md5()
    m.update(text)
    audio_filename = m.hexdigest()
    print("请将语音文件命名成下面的文件名，并防止到~/.ros/xiaoqiang_tts中对应的位置")
    print(audio_filename)
