#ifndef COMMUNICATION_PORT_FOR_AUDIO_H
#define COMMUNICATION_PORT_FOR_AUDIO_H

#include "Arduino.h"
#include "driver/uart.h"

// --- Configuration Constants ---
#define VOICE_UART_TX                     GPIO_NUM_9
#define VOICE_UART_RX                     GPIO_NUM_10
#define VOICE_UART_BAND_RATE              921600
#define VOICE_UART_BUF_SIZE               (1024 * 2)
#define VOICE_RECV_BUFF_LENGTH            1024

// 任务配置
#define VOICE_SEND_DATA_TO_AUDIO_TASK_NAME                "voice-send-data-to-audio-task"
#define VOICE_SEND_SLAVE_DATA_TASK_SIZE                   (1024*2)
#define VOICE_SEND_SLAVE_DATA_TASK_PRIORITY               1
#define VOICE_RECV_DATA_FROM_AUDIO_TASK_NAME              "voice-recv-data-from-audio-task"
#define VOICE_RECV_SLAVE_DATA_TASK_SIZE                   (1024*2)
#define VOICE_RECV_SLAVE_DATA_TASK_PRIORITY               2
#define VOICE_RECV_DATA_QUEUE_ITEAM_COUNT                 10
#define VOICE_RECV_DATA_QUEUE_ITEAM_SIZE                  (1024*4)
#define VOICE_SEND_DATA_QUEUE_ITEAM_COUNT                 2
#define VOICE_SEND_DATA_QUEUE_ITEAM_SIZE                  (1024 + 16 + sizeof(void*) + 4)

#define VOICE_OK 0
#define VOICE_FAIL 1

// 通信层需要的基本消息结构
typedef struct {
    uint8_t *data;
    uint16_t length;
    uint16_t type;
    uint32_t ack_flag;
} comm_message_t;

// --- 接收状态枚举 ---
typedef enum {
    MSG_FIND_HEAD = 0,
    MSG_RECV_MSG = 1,
    MSG_VERIFY = 2,
} comm_recv_state_t;

// --- 队列句柄声明 ---
extern QueueHandle_t messageRecvQueue;
extern QueueHandle_t messageSendQueue;

// --- 通信类定义 ---
class CommunicationPortForAudio {
private:
    uart_port_t uart_num;
    TaskHandle_t recvTaskHandle = nullptr;
    TaskHandle_t sendTaskHandle = nullptr;

    // 状态标志
    bool initialized = false;

    // 通信任务
    static void communicationRecvTask(void *pvParameters);
    static void communicationSendTask(void *pvParameters);

public:
    CommunicationPortForAudio();
    ~CommunicationPortForAudio();

    // 初始化方法
    int communicationPortInit(uart_port_t uart_port);
    int communicationTaskInit();

    // 通信方法
    int32_t communicationRecv(uint8_t *addr, int32_t length);
    int32_t communicationSend(const uint8_t *data, uint16_t length);

    // 获取状态
    bool isInitialized() const { return initialized; }
};

// 全局通信接口实例
extern CommunicationPortForAudio voiceModulePort;

#endif // COMMUNICATION_PORT_FOR_AUDIO_H