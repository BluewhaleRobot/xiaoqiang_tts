#include "xunfei_asr.h"

static std::string reg_result = "";
static bool result_flag = false;

/* Upload User words */
int upload_userwords(const char * user_dict)
{
    char *userwords = NULL;
    size_t len = 0;
    size_t read_len = 0;
    FILE *fp = NULL;
    int ret = -1;

    fp = fopen(user_dict, "rb");
    if (NULL == fp)
    {
        printf("\nopen [userwords.txt] failed!%s \n", user_dict);
        goto upload_exit;
    }

    fseek(fp, 0, SEEK_END);
    len = ftell(fp);
    fseek(fp, 0, SEEK_SET);

    userwords = (char *)malloc(len + 1);
    if (NULL == userwords)
    {
        printf("\nout of memory! \n");
        goto upload_exit;
    }

    read_len = fread((void *)userwords, 1, len, fp);
    if (read_len != len)
    {
        printf("\nread [userwords.txt] failed!\n");
        goto upload_exit;
    }
    userwords[len] = '\0';

    MSPUploadData("userwords", userwords, len, "sub = uup, dtt = userword", &ret); //ÉÏ´«ÓÃ»§´Ê±í
    if (MSP_SUCCESS != ret)
    {
        printf("\nMSPUploadData failed ! errorCode: %d \n", ret);
        goto upload_exit;
    }

upload_exit:
    if (NULL != fp)
    {
        fclose(fp);
        fp = NULL;
    }
    if (NULL != userwords)
    {
        free(userwords);
        userwords = NULL;
    }

    return ret;
}

static void show_result(char *string, char is_over)
{
    reg_result = "";
    reg_result = string;
    result_flag = true;
}

static char *g_result = NULL;
static unsigned int g_buffersize = BUFFER_SIZE;

void on_result(const char *result, char is_last)
{
    if (result)
    {
        size_t left = g_buffersize - 1 - strlen(g_result);
        size_t size = strlen(result);

        if (left < size)
        {
            g_result = (char *)realloc(g_result, g_buffersize + BUFFER_SIZE);
            if (g_result)
                g_buffersize += BUFFER_SIZE;
            else
            {
                printf("mem alloc failed\n");
                result_flag = true;
                return;
            }
        }
        strncat(g_result, result, size);
        show_result(g_result, is_last);
    }
}
void on_speech_begin()
{
    reg_result = "";
    if (g_result)
    {
        free(g_result);
    }
    g_result = (char *)malloc(BUFFER_SIZE);
    g_buffersize = BUFFER_SIZE;
    memset(g_result, 0, g_buffersize);
}
void on_speech_end(int reason)
{
    if (reason != END_REASON_VAD_DETECT){
        reg_result = "";
        result_flag = true;
    }
}

static void process_audio_data(std::vector<char> audio_data, const char *session_begin_params)
{
    int errcode = 0;
    FILE *f_pcm = NULL;
    char *p_pcm = NULL;
    unsigned long pcm_count = 0;
    unsigned long pcm_size = 0;
    unsigned long read_size = 0;
    struct speech_rec iat;
    struct speech_rec_notifier recnotifier = {
        on_result,
        on_speech_begin,
        on_speech_end};

    pcm_size = audio_data.size();

    errcode = sr_init(&iat, session_begin_params, SR_USER, &recnotifier);
    if (errcode)
    {
        printf("speech recognizer init failed : %d\n", errcode);
        result_flag = true;
        goto iat_exit;
    }

    errcode = sr_start_listening(&iat);
    if (errcode)
    {
        printf("\nsr_start_listening failed! error code:%d\n", errcode);
        result_flag = true;
        goto iat_exit;
    }

    while (1)
    {
        unsigned int len = 10 * FRAME_LEN; /* 200ms audio */
        int ret = 0;

        if (pcm_size < 2 * len)
            len = pcm_size;
        if (len <= 0)
            break;

        ret = sr_write_audio_data(&iat, audio_data.data() + pcm_count, len);

        if (0 != ret)
        {
            printf("\nwrite audio data failed! error code:%d\n", ret);
            result_flag = true;
            goto iat_exit;
        }

        pcm_count += (long)len;
        pcm_size -= (long)len;
    }

    errcode = sr_stop_listening(&iat);
    if (errcode)
    {
        printf("\nsr_stop_listening failed! error code:%d \n", errcode);
        result_flag = true;
        goto iat_exit;
    }

iat_exit:

    sr_stop_listening(&iat);
    sr_uninit(&iat);
}

/* main thread: start/stop record ; query the result of recgonization.
 * record thread: record callback(data write)
 * helper thread: ui(keystroke detection)
 */

PyObject *asr(PyObject *self, PyObject *args)
{
    const char *appid;
    const char *user_dict;
    int timeout = 0;
    int ret = MSP_SUCCESS;
    int upload_on = 1; /* whether upload the user word */
    int aud_src = 0; /* from mic or file */

    int num_lines; /* how many lines we passed for parsing */
    int line;      /* pointer to the line as a string */
    char *token;   /* token parsed by strtok */

    PyObject *audio_data_list; /* the list of strings */
    PyObject *str_obj;         /* one string in the list */

    /* the O! parses for a Python object (listObj) checked
    to be of type PyList_Type */
    if (!PyArg_ParseTuple(args, "O!ssi", &PyList_Type, &audio_data_list, &appid, &user_dict, &timeout))
        return NULL;
    /* login params, please do keep the appid correct */
    std::string login_params = "appid = " + std::string(appid) + ", work_dir = .";

    /* get the number of lines passed to us */
    num_lines = PyList_Size(audio_data_list);

    /* should raise an error here. */
    if (num_lines < 0)
        return NULL; /* Not a list */

    std::vector<char> audio_data;

    for (int i = 0; i < num_lines; i++)
    {

        /* grab the string object from the next element of the list */
        str_obj = PyList_GetItem(audio_data_list, i); /* Can't fail */

        /* make it a string */
        line = PyLong_AsLong(str_obj);
        audio_data.push_back((char)line);
    }

    /*
	* See "iFlytek MSC Reference Manual"
	*/
    const char *session_begin_params =
        "sub = iat, domain = iat, language = zh_cn, "
        "accent = mandarin, sample_rate = 16000, "
        "result_type = plain, result_encoding = utf8";

    /* Login first. the 1st arg is username, the 2nd arg is password
	 * just set them as NULL. the 3rd arg is login paramertes 
	 * */
    ret = MSPLogin(NULL, NULL, login_params.c_str());
    if (MSP_SUCCESS != ret)
    {
        printf("MSPLogin failed , Error code %d.\n", ret);
        result_flag = true;
        goto exit; // login fail, exit the program
    }

    if (strlen(user_dict) > 0)
    {
        printf("Uploading the user words ...\n");
        ret = upload_userwords(user_dict);
        if (MSP_SUCCESS != ret){
            result_flag = true;
            goto exit;
        }
        printf("Uploaded successfully\n");
    }
    process_audio_data(audio_data, session_begin_params);
exit:
    MSPLogout(); // Logout...
    int timecount = 0;

    while(!result_flag && (timecount < timeout || timeout == 0)){
        usleep(100*1000);
        timecount += 100;
    }
    return Py_BuildValue("s", reg_result.c_str());
}