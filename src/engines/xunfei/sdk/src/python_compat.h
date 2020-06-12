
/******************************************************************************
*
* The MIT License (MIT)
*
* Copyright (c) 2018 Bluewhale Robot
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*
* Author: Randoms
*******************************************************************************/

#ifndef XUNFEI_TTS_PY_PYTHON_COMPAT_H
#define XUNFEI_TTS_PY_PYTHON_COMPAT_H

#include <Python.h>

#include <string>

inline PyObject *stringToPython(const std::string &input)
{
#if PY_MAJOR_VERSION >= 3
  return PyUnicode_FromStringAndSize(input.c_str(), input.size());
#else
  return PyString_FromStringAndSize(input.c_str(), input.size());
#endif
}

inline PyObject *stringToPython(const char *input)
{
#if PY_MAJOR_VERSION >= 3
  return PyUnicode_FromString(input);
#else
  return PyString_FromString(input);
#endif
}

inline std::string stringFromPython(PyObject *input)
{
  Py_ssize_t size;
#if PY_MAJOR_VERSION >= 3
  const char *data = PyUnicode_AsUTF8AndSize(input, &size);
#else
  char *data;
  PyString_AsStringAndSize(input, &data, &size);
#endif
  return std::string(data, size);
}

inline PyObject *pythonImport(const std::string &name)
{
  PyObject *py_name = stringToPython(name);
  PyObject *module = PyImport_Import(py_name);
  Py_XDECREF(py_name);
  return module;
}

inline PyObject *pythonBorrowAttrString(PyObject *o, const char *name)
{
  PyObject *r = PyObject_GetAttrString(o, name);
  Py_XDECREF(r);
  return r;
}

#endif
