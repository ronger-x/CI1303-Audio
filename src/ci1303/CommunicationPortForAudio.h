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
#define VOICE_SEND_SLAVE_DATA_TASK_PRIORITY               (4)                                       //任务优先级
#define CLOUD_HTTP_DOWNLOAD_BUF_SIZE                      (1024*1 + 1)                              // http 取数据 BUF 大小 1KB
#define VOICE_RECV_DATA_FROM_AUDIO_TASK_NAME              "voice-recv-data-from-audio-task"
#define VOICE_RECV_SLAVE_DATA_TASK_SIZE                   (1024*2)
#define VOICE_RECV_SLAVE_DATA_TASK_PRIORITY               (5)
#define VOICE_RECV_DATA_QUEUE_ITEM_COUNT                  10
#define VOICE_RECV_DATA_QUEUE_ITEM_SIZE                   (64)
#define VOICE_SEND_DATA_QUEUE_ITEM_COUNT                  2
#define VOICE_SEND_DATA_QUEUE_ITEM_SIZE                   (CLOUD_HTTP_DOWNLOAD_BUF_SIZE + 50)       // 队列中每个元素的最大长度

/**
 *
 * @brief OS status definetion
 */
typedef enum
{
    VOICE_OK             = 0,    /*success*/
    VOICE_FAIL           = -1,   /*failure*/
    VOICE_ERR_NOMEM      = -2,   /*out of memory*/
    VOICE_ERR_PARAM      = -3,   /*invalid parameter*/
    VOICE_ERR_TIMEOUT    = -4,   /*timeout*/
}voice_status;

// --- 队列句柄声明 ---
extern QueueHandle_t messageRecvQueue;
extern QueueHandle_t messageSendQueue;

// --- 通信类定义 ---
class CommunicationPortForAudio {
private:
    static uart_port_t uart_num;
    static TaskHandle_t recvTaskHandle;
    static TaskHandle_t sendTaskHandle;

    // 状态标志
    static bool initialized;

public:
    // 初始化方法
    static int communicationPortInit(uart_port_t uart_port);
    static int communicationTaskInit();

    // 通信方法
    static int32_t communicationRecv(uint8_t *addr, uint32_t length);
    static int32_t communicationSend(uint8_t *data, uint32_t length);

    // 通信任务
    static void communicationRecvTask(void *pvParameters);
    static void communicationSendTask(void *pvParameters);

    // 清理资源
    static void cleanup();
    // 获取状态
    static bool isInitialized() { return initialized; }
};

#endif // COMMUNICATION_PORT_FOR_AUDIO_H