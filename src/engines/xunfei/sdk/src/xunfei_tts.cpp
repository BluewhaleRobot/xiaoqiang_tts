#include <Python.h>
#include "python_compat.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <vector>

#include "qtts.h"
#include "msp_cmn.h"
#include "msp_errors.h"
#include "xunfei_asr.h"

/* wav音频头部格式 */
typedef struct _wave_pcm_hdr
{
  char riff[4]; // = "RIFF"
  int size_8;   // = FileSize - 8
  char wave[4]; // = "WAVE"
  char fmt[4];  // = "fmt "
  int fmt_size; // = 下一个结构体的大小 : 16

  short int format_tag;      // = PCM : 1
  short int channels;        // = 通道数 : 1
  int samples_per_sec;       // = 采样率 : 8000 | 6000 | 11025 | 16000
  int avg_bytes_per_sec;     // = 每秒字节数 : samples_per_sec * bits_per_sample / 8
  short int block_align;     // = 每采样点字节数 : wBitsPerSample / 8
  short int bits_per_sample; // = 量化比特数: 8 | 16

  char data[4];  // = "data";
  int data_size; // = 纯数据长度 : FileSize - 44
} wave_pcm_hdr;

/* 默认wav音频头部数据 */
wave_pcm_hdr default_wav_hdr =
    {
        {'R', 'I', 'F', 'F'},
        0,
        {'W', 'A', 'V', 'E'},
        {'f', 'm', 't', ' '},
        16,
        1,
        1,
        16000,
        32000,
        2,
        16,
        {'d', 'a', 't', 'a'},
        0};
/* 文本合成 */
std::vector<uint8_t> text_to_speech(const char *src_text, const char *params)
{
  int ret = -1;
  FILE *fp = NULL;
  const char *sessionID = NULL;
  unsigned int audio_len = 0;
  wave_pcm_hdr wav_hdr = default_wav_hdr;
  int synth_status = MSP_TTS_FLAG_STILL_HAVE_DATA;
  std::vector<uint8_t> audio_data;

  if (NULL == src_text)
  {
    printf("params is error!\n");
    return audio_data;
  }

  /* 开始合成 */
  sessionID = QTTSSessionBegin(params, &ret);
  if (MSP_SUCCESS != ret)
  {
    printf("QTTSSessionBegin failed, error code: %d.\n", ret);
    return audio_data;
  }
  ret = QTTSTextPut(sessionID, src_text, (unsigned int)strlen(src_text), NULL);
  if (MSP_SUCCESS != ret)
  {
    printf("QTTSTextPut failed, error code: %d.\n", ret);
    QTTSSessionEnd(sessionID, "TextPutError");
    return audio_data;
  }
  printf("正在合成 ...\n");
  while (1)
  {
    /* 获取合成音频 */
    const void *data = QTTSAudioGet(sessionID, &audio_len, &synth_status, &ret);
    if (MSP_SUCCESS != ret)
      break;
    if (NULL != data)
    {
      audio_data.insert(audio_data.end(), (uint8_t *)data, (uint8_t *)data + audio_len);
      wav_hdr.data_size += audio_len; //计算data_size大小
    }
    if (MSP_TTS_FLAG_DATA_END == synth_status)
      break;
    usleep(150 * 1000); //防止频繁占用CPU
  }

  if (MSP_SUCCESS != ret)
  {
    printf("QTTSAudioGet failed, error code: %d.\n", ret);
    QTTSSessionEnd(sessionID, "AudioGetError");
    return audio_data;
  }
  /* 修正wav文件头数据的大小 */
  wav_hdr.size_8 += wav_hdr.data_size + (sizeof(wav_hdr) - 8);
  audio_data.insert(audio_data.begin(), (uint8_t *)&wav_hdr, (uint8_t *)&wav_hdr + sizeof(wav_hdr));

  /* 合成完毕 */
  ret = QTTSSessionEnd(sessionID, "Normal");
  if (MSP_SUCCESS != ret)
  {
    printf("QTTSSessionEnd failed, error code: %d.\n", ret);
  }

  return audio_data;
}

/**
 * Python Module Related 
 **/

static PyObject *
tts(PyObject *self, PyObject *args)
{
  const char *words;
  const char *appid;

  if (!PyArg_ParseTuple(args, "ss", &words, &appid))
    Py_RETURN_NONE;

  int ret = MSP_SUCCESS;
  std::string params = "appid = " + std::string(appid) + ", work_dir = .";
  const char *login_params = params.c_str(); //登录参数,appid与msc库绑定,请勿随意改动
  /*
    * rdn:           合成音频数字发音方式
    * volume:        合成音频的音量
    * pitch:         合成音频的音调
    * speed:         合成音频对应的语速
    * voice_name:    合成发音人
    * sample_rate:   合成音频采样率
    * text_encoding: 合成文本编码格式
    *
    */
  const char *session_begin_params = "voice_name = xiaoyan, text_encoding = utf8, sample_rate = 16000, speed = 50, volume = 50, pitch = 50, rdn = 2";

  /* 用户登录 */
  ret = MSPLogin(NULL, NULL, login_params); //第一个参数是用户名，第二个参数是密码，第三个参数是登录参数，用户名和密码可在http://www.xfyun.cn注册获取
  if (MSP_SUCCESS != ret)
  {
    printf("MSPLogin failed, error code: %d.\n", ret);
    MSPLogout(); //退出登录
    Py_RETURN_NONE;
  }
  /* 文本合成 */
  auto audio_data = text_to_speech(words, session_begin_params);
  if (MSP_SUCCESS != ret)
  {
    printf("text_to_speech failed, error code: %d.\n", ret);
  }
  MSPLogout(); //退出登录

  PyObject *pylist = PyList_New(audio_data.size());
  PyObject *item;
  for (int i = 0; i < audio_data.size(); i++)
  {
    item = PyInt_FromLong(audio_data[i]);
    PyList_SET_ITEM(pylist, i, item);
  }
  return pylist;
}

static PyMethodDef module_methods[] =
    {
        {"tts", tts, METH_VARARGS, "tts"},
        {"asr", asr, METH_VARARGS, "asr"},
        {NULL, NULL, 0, NULL},
    };

PyObject *moduleInit(PyObject *m)
{
  return m;
}

#if PY_MAJOR_VERSION < 3
extern "C" void init_xunfei_tts()
{
  moduleInit(Py_InitModule("_xunfei_tts", module_methods));
}

#else
struct PyModuleDef tf_module = {
    PyModuleDef_HEAD_INIT, // base
    "_xunfei_tts",         // name
    NULL,                  // docstring
    -1,                    // state size (but we're using globals)
    module_methods         // methods
};

PyMODINIT_FUNC PyInit_xunfei_tts()
{
  if (!staticInit())
    return NULL;
  return moduleInit(PyModule_Create(&tf_module));
}
#endif