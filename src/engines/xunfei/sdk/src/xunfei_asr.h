#ifndef _XUNFEI_ASR_H
#define _XUNFEI_ASR_H

#include <Python.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <vector>
#include <iostream>
#include "qisr.h"
#include "msp_cmn.h"
#include "msp_errors.h"
#include "samples/iat_record_sample/speech_recognizer.h"

#define FRAME_LEN	640 
#define	BUFFER_SIZE	4096

int upload_userwords();
PyObject* asr(PyObject *self, PyObject *args);

#endif