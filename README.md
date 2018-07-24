# xiaoqiang_tts

小强语音合成和语音识别程序。处理后端可以方便的设置为科大迅飞语音和百度语音。可以利用audio_capture实时的处理从话筒中收到的数据。

## 安装

```bash
cd [到你的工作空间的src文件夹中]
git clone https://github.com/bluewhalerobot/xiaoqiang_tts
git clone https://github.com/bluewhalerobot/xiaoqiang_audio
rosdep install xiaoqiang_audio
rosdep install xiaoqiang_tts
sudo pip install requests
sudo pip install baidu-aip
sudo apt-get install mplayer
cd ..
catkin_make -DCATKIN_WHITELIST_PACKAGES="xiaoqiang_tts"
```

程序使用的是我自己申请的百度和迅飞参数，最好自己再去百度和迅飞的官网申请一个。迅飞的sdk下载完成后覆盖至src/engines/xunfei/sdk内，注意保留其中的src文件夹。然后在launch文件中设置自己的appid，再catkin_make一下就可以了。

## 使用

### 使用语音合成功能

启动百度语音合成

```bash
roslaunch xiaoqiang_tts tts_baidu.launch
```

|输入Topic|消息类型|
|:--|:--|
|/xiaoqiang_tts/text|std_msgs/String|

|输出Topic|消息类型|
|:--|:--|
|/xiaoqiang_audio/audio|audio_common_msgs/AudioData|

发布消息，测试语音

```bash
rostopic pub /xiaoqiang_tts/text std_msgs/String 测试一下语音合成 -1
```

此时如果正常应该能够听到"测试一下语音合成"的声音

启动科大迅飞语音合成

`注意要先关闭百度tts节点`

```bash
roslaunch xiaoqiang_tts tts_xunfei.launch
```

测试方法和上面一样。正常应该会听到合成的声音。

### 使用语音识别功能

启动百度语音识别

```bash
roslaunch xiaoqiang_tts asr_baidu.launch
```

新开一个终端接收语音识别结果

```bash
rostopic echo /xiaoqiang_tts/text
```

现在可以开始说话了，程序会自动监听环境声音并进行分句。当你停止说话时会开始处理你的这一句的结果。

|输入Topic|消息类型|
|:--|:--|
|/xiaoqiang_audio/audio|audio_common_msgs/AudioData|

|输出Topic|消息类型|
|:--|:--|
|/xiaoqiang_tts/text|std_msgs/String|


启动迅飞语音识别

```bash
roslaunch xiaoqiang_tts asr_xunfei.launch
```

使用方法和上面的一样

### 同时启动语音识别和语音合成

使用科大迅飞语音合成和语音识别

```bash
roslaunch xiaoqiang_tts tts_xunfei.launch
roslaunch xiaoqiang_tts asr_xunfei.launch
```

现在你说一句话机器人就会跟着你说一句话。


### 参数说明

详细的参数说明请参照launch文件内的注释
