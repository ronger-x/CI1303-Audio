//
// Created by ronger on 2025/1/11.
//
#include <stdint.h>

#ifndef VOICE_MODULE_UART_PROTOCOL_H
#define VOICE_MODULE_UART_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#define VMUP_MSG_DATA_MAX_SIZE (20)

#pragma pack(1)
typedef struct
{
    uint16_t header;
    uint16_t data_length;
    uint8_t msg_type;
    uint8_t msg_cmd;
    uint8_t msg_seq;
    uint8_t msg_data[VMUP_MSG_DATA_MAX_SIZE];
    uint16_t chk_sum;
    uint8_t tail;
} sys_msg_com_data_t;


#pragma pack()

/*header*/
#define VMUP_MSG_HEAD_LOW  (0xA5)
#define VMUP_MSG_HEAD_HIGH (0xFC)
#define VMUP_MSG_HEAD   ((VMUP_MSG_HEAD_HIGH<<8)|VMUP_MSG_HEAD_LOW)


/*tail*/
#define VMUP_MSG_TAIL   (0xFB)


/*msg_type*/
#define VMUP_MSG_TYPE_CMD_UP   (0xA0)
#define VMUP_MSG_TYPE_CMD_DOWN (0xA1)
#define VMUP_MSG_TYPE_ACK      (0xA2)
#define VMUP_MSG_TYPE_NOTIFY   (0xA3)


/*msg_cmd*/
#define VMUP_MSG_CMD_ASR_RESULT    (0x91)           //报告语音识别结果
#define VMUP_MSG_CMD_PLAY_VOICE    (0x92)           //播放本地播报音
#define VMUP_MSG_CMD_GET_FLASHUID  (0x93)           //读取FLASH的序列号
#define VMUP_MSG_CMD_GET_VERSION   (0x94)           //读取版本号
#define VMUP_MSG_CMD_RESET_MODULE  (0x95)           //复位语音模块
#define VMUP_MSG_CMD_SET_CONFIG    (0x96)           //设置
#define VMUP_MSG_CMD_ENTER_OTA_MODE (0x97)          //进入升级模式
#define VMUP_MSG_CMD_NOTIFY_STATUS (0x9A)           //事件通知
#define VMUP_MSG_CMD_ACK_COMMON    (0xAA)
/* !!! if user want add please add form VMUP_MSG_CMD_USER_START*/
#define VMUP_MSG_CMD_USER_START    (0xB0)


/*msg_data  msg_cmd:VMUP_MSG_CMD_PLAY_VOICE*/
#define VMUP_MSG_DATA_PLAY_START   (0x80)
#define VMUP_MSG_DATA_PLAY_PAUSE   (0x81)
#define VMUP_MSG_DATA_PLAY_RESUME  (0x82)
#define VMUP_MSG_DATA_PLAY_STOP    (0x83)

#define VMUP_MSG_DATA_PLAY_BY_VOICEID (0x90)
#define VMUP_MSG_DATA_PLAY_BY_SEMANTIC_ID   (0x91)
#define VMUP_MSG_DATA_PLAY_BY_CMD_ID   (0x92)


/*msg_data  msg_cmd:VMUP_MSG_CMD_GET_VERSION*/
#define VMUP_MSG_DATA_VER_PROTOCOL   (0x80)         // 串口协议版本号
#define VMUP_MSG_DATA_VER_SDK        (0x81)         // SDK版本号
#define VMUP_MSG_DATA_VER_ASR        (0x82)         // ASR组件版本号
#define VMUP_MSG_DATA_VER_PREPROCESS (0x83)         // 语音预处理算法版本号
#define VMUP_MSG_DATA_VER_PLAYER     (0x84)         // 播放器版本号
#define VMUP_MSG_DATA_VER_APP        (0x8A)         // 应用程序版本号


/*msg_data  msg_cmd:VMUP_MSG_CMD_NOTIFY_STATUS*/
#define VMUP_MSG_DATA_NOTIFY_POWERON     (0xB0)
#define VMUP_MSG_DATA_NOTIFY_WAKEUPENTER (0xB1)
#define VMUP_MSG_DATA_NOTIFY_WAKEUPEXIT  (0xB2)
#define VMUP_MSG_DATA_NOTIFY_PLAYSTART   (0xB3)
#define VMUP_MSG_DATA_NOTIFY_PLAYEND     (0xB4)


/*msg_data msg_cmd:VMUP_MSG_CMD_SET_CONFIG*/
#define VMUP_MSG_CMD_SET_VOLUME        (0x80)
#define VMUP_MSG_CMD_SET_ENTERWAKEUP   (0x81)
#define VMUP_MSG_CMD_SET_PRT_MID_RST   (0x82)
#define VMUP_MSG_CMD_SET_MUTE          (0x83)
#define VMUP_MSG_CMD_SET_NEEDACK       (0x90)
#define VMUP_MSG_CMD_SET_NEEDSTRING    (0x91)

/************************************
         MSG
*************************************/
typedef enum
{
    SYS_MSG_TYPE_ASR = 0,
    SYS_MSG_TYPE_CMD_INFO,
    SYS_MSG_TYPE_KEY,
    SYS_MSG_TYPE_COM,
    SYS_MSG_TYPE_PLAY,
    SYS_MSG_TYPE_NET,
    SYS_MSG_TYPE_MNG,
    SYS_MSG_TYPE_AUDIO_IN_STARTED,
    SYS_MSG_TYPE_I2C,
    SYS_MSG_TYPE_SED,
    SYS_MSG_TYPE_NLP,
    SYS_MSG_TYPE_TEXT,
    SYS_MSG_TYPE_SET_PALY_PARAMETER, //音量，语速，语调
    SYS_MSG_TYPE_PLAY_OUTSIDE
} sys_msg_type_t;


typedef struct
{
    sys_msg_type_t msg_type; /*here will be modify use union*/
    union
    {
        sys_msg_com_data_t com_data;
    } msg_data;
} sys_msg_t;

int vmup_port_send_packet_rev_msg(sys_msg_com_data_t* msg);


#ifdef __cplusplus
}
#endif

#endif //VOICE_MODULE_UART_PROTOCOL_H
