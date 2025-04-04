#include "CommunicationPortForAudio.h"
#include "SlaveMessageHandle.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "CommPortAudio";

// --- 全局队列句柄 ---
QueueHandle_t messageRecvQueue = nullptr;
QueueHandle_t messageSendQueue = nullptr;

// --- 静态成员变量初始化 ---
uart_port_t CommunicationPortForAudio::uart_num = UART_NUM_1;
TaskHandle_t CommunicationPortForAudio::recvTaskHandle = nullptr;
TaskHandle_t CommunicationPortForAudio::sendTaskHandle = nullptr;
bool CommunicationPortForAudio::initialized = false;

// --- UART初始化 ---
int CommunicationPortForAudio::communicationPortInit(uart_port_t uart_port) {
    if (initialized) {
        ESP_LOGW(TAG, "UART already initialized");
        return VOICE_OK;
    }

    uart_num = uart_port;

    // 配置UART参数
    uart_config_t uart_config = {
        .baud_rate = VOICE_UART_BAND_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    // 安装UART驱动
    esp_err_t err = uart_driver_install(uart_num, VOICE_UART_BUF_SIZE * 2, VOICE_UART_BUF_SIZE * 2, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART driver install failed: %s", esp_err_to_name(err));
        return VOICE_FAIL;
    }

    // 配置UART参数
    err = uart_param_config(uart_num, &uart_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART param config failed: %s", esp_err_to_name(err));
        uart_driver_delete(uart_num);
        return VOICE_FAIL;
    }

    // 设置UART引脚
    err = uart_set_pin(uart_num, VOICE_UART_TX, VOICE_UART_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART set pin failed: %s", esp_err_to_name(err));
        uart_driver_delete(uart_num);
        return VOICE_FAIL;
    }

    initialized = true;
    return VOICE_OK;
}

// --- 通信任务初始化 ---
int CommunicationPortForAudio::communicationTaskInit() {
    if (!initialized) {
        ESP_LOGE(TAG, "UART not initialized. Call communicationPortInit first.");
        return VOICE_FAIL;
    }

    // 创建消息队列
    messageRecvQueue = xQueueCreate(VOICE_RECV_DATA_QUEUE_ITEM_COUNT, VOICE_RECV_DATA_QUEUE_ITEM_SIZE);
    if (messageRecvQueue == nullptr) {
        ESP_LOGE(TAG, "Failed to create messageRecvQueue");
        return VOICE_FAIL;
    }

    messageSendQueue = xQueueCreate(VOICE_SEND_DATA_QUEUE_ITEM_COUNT, VOICE_SEND_DATA_QUEUE_ITEM_SIZE);
    if (messageSendQueue == nullptr) {
        ESP_LOGE(TAG, "Failed to create messageSendQueue");
        vQueueDelete(messageRecvQueue);
        messageRecvQueue = nullptr;
        return VOICE_FAIL;
    }

    // 创建发送任务
    BaseType_t xTaskCreateResult = xTaskCreate(
        communicationSendTask,
        VOICE_SEND_DATA_TO_AUDIO_TASK_NAME,
        VOICE_SEND_SLAVE_DATA_TASK_SIZE / sizeof(StackType_t),
        nullptr,
        VOICE_SEND_SLAVE_DATA_TASK_PRIORITY,
        &sendTaskHandle
    );

    if (xTaskCreateResult != pdPASS) {
        ESP_LOGE(TAG, "Failed to create communicationSendTask");
        vTaskDelete(recvTaskHandle);
        recvTaskHandle = nullptr;
        vQueueDelete(messageRecvQueue);
        vQueueDelete(messageSendQueue);
        messageRecvQueue = nullptr;
        messageSendQueue = nullptr;
        return VOICE_FAIL;
    }

    // 创建接收任务
    xTaskCreateResult = xTaskCreate(
        communicationRecvTask,
        VOICE_RECV_DATA_FROM_AUDIO_TASK_NAME,
        VOICE_RECV_SLAVE_DATA_TASK_SIZE / sizeof(StackType_t),
        nullptr,
        VOICE_RECV_SLAVE_DATA_TASK_PRIORITY,
        &recvTaskHandle
    );

    if (xTaskCreateResult != pdPASS) {
        ESP_LOGE(TAG, "Failed to create communicationRecvTask");
        vQueueDelete(messageRecvQueue);
        vQueueDelete(messageSendQueue);
        messageRecvQueue = nullptr;
        messageSendQueue = nullptr;
        return VOICE_FAIL;
    }

    return VOICE_OK;
}

// --- 接收数据函数 ---
int32_t CommunicationPortForAudio::communicationRecv(uint8_t *addr, uint32_t length) {
    if (!initialized) {
        ESP_LOGE(TAG, "UART not initialized");
        return VOICE_FAIL;
    }

    if (length > VOICE_RECV_BUFF_LENGTH) {
        ESP_LOGE(TAG, "Requested length too large: %d > %d", length, VOICE_RECV_BUFF_LENGTH);
        return VOICE_FAIL;
    }

    size_t buffered_len;
    const esp_err_t err = uart_get_buffered_data_len(uart_num, &buffered_len);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_get_buffered_data_len failed: %s", esp_err_to_name(err));
        return VOICE_FAIL;
    }

    if (buffered_len == 0) {
        return VOICE_OK;
    }

    return uart_read_bytes(uart_num, addr, length, pdMS_TO_TICKS(10));
}

// --- 发送数据函数 ---
int32_t CommunicationPortForAudio::communicationSend(uint8_t *data, uint32_t length) {
    if (!initialized) {
        ESP_LOGE(TAG, "UART not initialized");
        return VOICE_FAIL;
    }

    int bytes_written = uart_write_bytes(uart_num, data, length);
    uart_wait_tx_done(uart_num, pdMS_TO_TICKS(100));
    return bytes_written;
}

// --- 接收任务 ---
void CommunicationPortForAudio::communicationRecvTask(void *pvParameters) {
    int32_t ret = 0;
    uint32_t data_len = 0;
    const cias_standard_head_t *recv_head_part = nullptr;
    static comm_recv_state_t slave_msg_state = MSG_FIND_HEAD;

    uint8_t recv_slave_buf[VOICE_RECV_DATA_QUEUE_ITEM_SIZE] = {};

    vTaskDelay(1000); // 启动延迟

    while (true) {
        // 查找消息头
        if (slave_msg_state == MSG_FIND_HEAD) {
            ret = communicationRecv(recv_slave_buf + data_len, sizeof(cias_standard_head_t) - data_len);

            if (ret >= 0 && ret <= sizeof(cias_standard_head_t)) {
                data_len += ret;
                if (data_len < sizeof(cias_standard_head_t)) {
                    vTaskDelay(5);
                    continue;
                }
                recv_head_part = reinterpret_cast<cias_standard_head_t *>(recv_slave_buf);
                data_len = 0;

                if (recv_head_part->magic == CIAS_STANDARD_MAGIC) { // 预定义的魔数
                    if (recv_head_part->len == 0) {
                        slave_msg_state = MSG_VERIFY;
                    } else {
                        slave_msg_state = MSG_RECV_MSG;
                    }
                } else {
                    ESP_LOGE(TAG, "recv_head_part magic err1\n");
                    slave_msg_state = MSG_FIND_HEAD;
                    memset(recv_slave_buf, 0, VOICE_RECV_DATA_QUEUE_ITEM_SIZE);
                }
            } else {
                ESP_LOGE(TAG, "recv_head_part magic err2\n");
                slave_msg_state = MSG_FIND_HEAD;
                memset(recv_slave_buf,0,VOICE_RECV_DATA_QUEUE_ITEM_SIZE);
            }
        }

        // 接收消息体
        if (slave_msg_state == MSG_RECV_MSG) {
            ret = communicationRecv(recv_slave_buf + sizeof(cias_standard_head_t) + data_len, (recv_head_part->len - data_len));

            if (ret>=0 && ret <= recv_head_part->len) {
                data_len += ret;
                if (data_len < recv_head_part->len) {
                    vTaskDelay(2);
                    continue;
                } else {
                    data_len = 0;
                    slave_msg_state = MSG_VERIFY;
                }
            } else {
                ESP_LOGE(TAG, "recv_head_part magic err2\n");
                data_len = 0;
                memset(recv_slave_buf,0,VOICE_RECV_DATA_QUEUE_ITEM_SIZE);
                slave_msg_state = MSG_FIND_HEAD;
            }
        }

        // 验证并处理消息
        if (slave_msg_state == MSG_VERIFY) {
            slave_msg_state = MSG_FIND_HEAD;

            // 通过队列传递
            if (xQueueSend(messageRecvQueue, recv_slave_buf, pdMS_TO_TICKS(10)) != pdPASS) {
                ESP_LOGE(TAG, "messageRecvQueue send fail...\r\n");
            }

            data_len = 0;
            memset(recv_slave_buf, 0, VOICE_RECV_DATA_QUEUE_ITEM_SIZE);
        }

        ret = 0;
        vTaskDelay(5);
    }
}

// --- 发送任务 ---
void CommunicationPortForAudio::communicationSendTask(void *pvParameters) {
    if (!initialized) {
        ESP_LOGE(TAG, "UART not initialized");
        return;
    }

    cias_send_msg_t send_msg;

    while (true) {
        if (xQueueReceive(messageSendQueue, &send_msg, portMAX_DELAY) == pdTRUE) {
            if (send_msg.length > 0) {
                communicationSend(send_msg.data, send_msg.length);

                // 如果动态分配了内存，需要释放
                if (send_msg.ack_flag == 1) {
//                    free(send_msg.data);
                }
            }
        }
    }
}

// --- 清理资源 ---
void CommunicationPortForAudio::cleanup() {
    // 清理任务
    if (recvTaskHandle) {
        vTaskDelete(recvTaskHandle);
        recvTaskHandle = nullptr;
    }

    if (sendTaskHandle) {
        vTaskDelete(sendTaskHandle);
        sendTaskHandle = nullptr;
    }

    // 清理队列
    if (messageRecvQueue) {
        vQueueDelete(messageRecvQueue);
        messageRecvQueue = nullptr;
    }

    if (messageSendQueue) {
        vQueueDelete(messageSendQueue);
        messageSendQueue = nullptr;
    }

    // 如果UART已初始化，则删除驱动程序
    if (initialized && uart_num < UART_NUM_MAX) {
        uart_driver_delete(uart_num);
    }

    initialized = false;
}
