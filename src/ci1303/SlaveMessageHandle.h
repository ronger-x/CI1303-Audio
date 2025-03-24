//
// Created by ronger on 2025/3/24.
//

#ifndef CI1303_AUDIO_SLAVE_MESSAGE_HANDLE_H
#define CI1303_AUDIO_SLAVE_MESSAGE_HANDLE_H
#include "Arduino.h"

#define CIAS_STANDARD_MAGIC 0x5a5aa5a5
#define CIAS_SEND_MSG_BUF_LEN 1040 // 1024 + 16

#define CIAS_PCM_UPLOAD_QUEUE_SIZE 10

/**
 * @brief 标准消息头部结构
 */
typedef struct __attribute__((packed)) {
    uint32_t magic;       // Start data for frame (0x5a5aa5a5)
    uint16_t checksum;    // Checksum
    uint16_t type;        // Command type
    uint16_t len;         // Data stream length
    uint16_t version;     // Version information
    uint32_t fill_data;   // Fill data
} cias_standard_head_t;

/**
 * @brief 原始语音数据结构
 */
typedef struct __attribute__((packed)) {
    uint8_t data[1024];
    uint32_t length;
    uint16_t type;
} cias_raw_speech_t;

/**
 * @brief 发送消息的数据结构
 */
typedef struct __attribute__((packed)) {
    uint8_t data[CIAS_SEND_MSG_BUF_LEN];
    uint16_t length;
    uint16_t type;
    uint32_t ack_flag;  // 需要ACK
} cias_send_msg_t;

/**
 * @brief 填充类型枚举
 */
typedef enum {
    DEF_FILL = 0x12345678,
    SPEAK_FILL = 0x12345677,
    INVAILD_SPEAK_FILL = 0x12345666,
    MUSIC_START_FILL = 0x12345699,
    MP3_FILL = 0x12345688,
    M4A_FILL = 0x123456aa,
    WAV_FILL = 0x123456bb,
    REPEAT_FILL = 0x123456ab,
} cias_fill_type_t;

/**
 * @brief 从机接收状态枚举
 */
typedef enum {
    MSG_FIND_HEAD = 0,
    MSG_RECV_MSG = 1,
    MSG_VERIFY = 2,
} slave_recv_state_t;

/**
 * @brief SDIO消息结构
 */
typedef struct {
    uint32_t wifi_msg_index;
} sdio_msg_t;

#define AUDIO_MUSIC_BUFF_COUNT (29)
#define AUDIO_MUSIC_BUFF_LEN   (AUDIO_MUSIC_BUFF_COUNT * 1024)

extern volatile bool audio_player_need_data_1L;
extern volatile bool audio_player_need_data_2L;
extern volatile bool audio_player_need_data_3L;

#define TEST_PLAYER 0 // 播报原始数据测试

/**
 * @brief WiFi通信命令枚举
 */
typedef enum {
    // 语音识别相关
    LOCAL_ASR_RESULT_NOTIFY             =  0x0101,  // 本地语音识别通知
    WAKE_UP                             =  0x0102,  // 唤醒
    VAD_END                             =  0x0103,  // 云端VAD END
    SKIP_INVAILD_SPEAK                  =  0x0104,  // 跳过无效语音
    PCM_MIDDLE                          =  0x0105,  // PCM数据中间包
    PCM_FINISH                          =  0x0106,  // PCM数据结束包
    PCM_IDLE                            =  0x0107,  // PCM数据空闲

    // 网络播放相关
    NET_PLAY_START                      =  0x0201,  // 开始播放
    NET_PLAY_PAUSE                      =  0x0202,  // 播放暂停
    NET_PLAY_RESUME                     =  0x0203,  // 恢复播放
    NET_PLAY_STOP                       =  0x0204,  // 停止播放
    NET_PLAY_RESTART                    =  0x0205,  // 重播
    NET_PLAY_NEXT                       =  0x0206,  // 播放下一首
    NET_PLAY_LOCAL_TTS                  =  0x0207,  // 播放本地TTS
    NET_PLAY_END                        =  0x0208,  // 播放结束
    NET_PLAY_RECONECT_URL               =  0x0209,  // 重新获取连接
    PLAY_DATA_GET                       =  0x020a,  // 获取后续播放数据
    PLAY_DATA_RECV                      =  0x020b,  // 接收播放数据
    PLAY_DATA_END                       =  0x020c,  // 播放数据接收完
    PLAY_TTS_END                        =  0x020d,  // 播放tts结束
    PLAY_EMPTY                          =  0x020e,  // 播放空指令
    PLAY_NEXT                           =  0x020f,  // 播放完上一首，主动播放下一首

    // IOT自定义协议
    QCLOUD_IOT_CMD                      =  0x0301,  // 云端IOT指令
    NET_VOLUME                          =  0x0302,  // 云端音量
    LOCAL_VOLUME                        =  0x0303,  // 本地音量
    VOLUME_INC                          =  0x0304,  // 增大音量
    VOLUME_DEC                          =  0x0305,  // 减小音量
    VOLUME_MAXI                         =  0x0306,  // 最大音量
    VOLUME_MINI                         =  0x0307,  // 最小音量

    // 网络相关
    ENTER_NET_CONFIG                    =  0x0401,  // 进入配网模式
    NET_CONFIGING                       =  0x0402,  // 配网中
    EXIT_NET_CONFIG                     =  0x0403,  // 退出配网模式
    INIT_SMARTCONFIG                    =  0x0404,  // 初始密码状态 出厂配置状态
    WIFI_DISCONNECTED                   =  0x0405,  // 网络断开
    WIFI_CONNECTED                      =  0x0406,  // 网络连接成功
    GET_PROFILE                         =  0x0407,  // 已获取鉴权文件
    NEED_PROFILE                        =  0x0408,  // 需要鉴权文件
    CLOUD_CONNECTED                     =  0x0409,  // 云端已连接
    CLOUD_DISCONNECTED                  =  0x040a,  // 云端已断开
    NET_CONFIG_SUCESS                   =  0x040b,  // 配网成功
    NET_CONFIG_FALI                     =  0x040c,  // 配网失败

    // OTA和工厂测试
    CIAS_OTA_START                      =  0x0501,  // 开始OTA
    CIAS_OTA_DATA                       =  0x0502,  // OTA数据
    CIAS_OTA_SUCESS                     =  0x0503,  // OTA升级成功
    CIAS_FACTORY_START                  =  0x0504,  // 生产测试
    CIAS_FACTORY_OK                     =  0x0505,  // 生产测试成功
    CIAS_FACTORY_FAIL                   =  0x0506,  // 生产测试失败
    CIAS_FACTORY_SELF_TEST_START        =  0x0507,  // 自测试
    CIAS_IR_DATA                        =  0x0508,  // 红外数据发送
    CIAS_IR_LOADING_DATA                =  0x0509,  // 红外码库下载中
    CIAS_IR_LOAD_DATA_OVER              =  0x050a,  // 红外码库下载完成
    CIAS_IR_LOAD_DATA_START             =  0x050b,  // 红外下载码库开始
} wifi_communicate_cmd_t;

class SlaveMessageHandle {
public:
    SlaveMessageHandle();
    ~SlaveMessageHandle();
    void voicePortInit(HardwareSerial& serial);
    void sendMessage(uint16_t cmd, cias_fill_type_t type, const uint8_t* data, uint16_t length);
    uint16_t getAudioState();
    uint16_t setAudioState(int state);
private:
    HardwareSerial* _serial;
};


#endif //CI1303_AUDIO_SLAVE_MESSAGE_HANDLE_H
