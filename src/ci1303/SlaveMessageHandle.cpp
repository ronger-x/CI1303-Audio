//
// Created by ronger on 2025/3/24.
//
#include "SlaveMessageHandle.h"
#include "esp_log.h" // For ESP32 logging
#include <string.h> // For memset

static const char *TAG = "SlaveMessageHandle"; // Used for logging

// --- Configuration ---
#define VOICE_UART_TX                                     GPIO_NUM_9
#define VOICE_UART_RX                                     GPIO_NUM_10
#define VOICE_UART_BAND_RATE                              115200 //  Consider 115200 for broader compatibility, unless 1Mbps is *required*
#define VOICE_UART_BUF_SIZE                               (1024 * 2)
#define VOICE_SEND_DATA_TO_AUDIO_TASK_NAME                "voice-send-data-to-audio-task"
#define VOICE_SEND_SLAVE_DATA_TASK_SIZE                   (1024*2)
#define VOICE_SEND_SLAVE_DATA_TASK_PRIORITY               1
#define VOICE_RECV_DATA_FROM_AUDIO_TASK_NAME              "voice-recv-data-from-audio-task"
#define VOICE_RECV_SLAVE_DATA_TASK_SIZE                   (1024*2)
#define VOICE_RECV_SLAVE_DATA_TASK_PRIORITY               2  // Higher priority for receive
#define VOICE_DEAL_RECV_AUDIO_DATA_TASK_NAME              "voice-deal-recv-audio"
#define VOICE_DEAL_RECV_AUDIO_DATA_TASK_SIZE              (1024*4)
#define VOICE_DEAL_RECV_AUDIO_DATA_TASK_PRIORITY          1
#define VOICE_HTTP_DOWNLOAD_BUF_SIZE                      (1024*1 + 1) // Consider removing this if it's not used elsewhere
#define VOICE_RECV_DATA_QUEUE_ITEAM_COUNT                 10
#define VOICE_RECV_DATA_QUEUE_ITEAM_SIZE                  (1024*4)
#define VOICE_SEND_DATA_QUEUE_ITEAM_COUNT                 2
#define VOICE_SEND_DATA_QUEUE_ITEAM_SIZE                  (CIAS_SEND_MSG_BUF_LEN + sizeof(cias_standard_head_t) + 4)   //Queue size adjusted to max possible message size
#define VOICE_RECV_BUFF_LENGTH 1024
#define CIAS_IOT_TVS_ENABLE 1

// --- Status Codes ---
#define VOICE_OK 0
#define VOICE_FAIL 1

// --- Global Variables ---
// HardwareSerial* _serial = nullptr; // Removed global _serial.
QueueHandle_t _messageRecvQueue = nullptr;
QueueHandle_t _messageSendQueue = nullptr;

void wakeup_deal(void) {
//    print_meida_state("uart recv:<1>");
//    is_wakeup = true;
//    if (g_cias_media.audio.state == MEDIA_AUDIO_PLAY)
//    {
//        g_cias_media.audio.state = MEDIA_AUDIO_RESUME;
//    }
//    g_cias_media.play_tts_end = false;
//    if (g_cias_media.speak.state == MEDIA_SPEAK_PLAY || g_cias_media.speak.state == MEDIA_SPEAK_SEND_END)
//    {
//        g_cias_media.speak.state = MEDIA_SPEAK_CANCLE;
//    }
//    cias_media_tts_url_time_stop();
//    g_cias_media.speak.asr_is_valid = false;
//    cias_message_send_queue_clear();
//    g_cias_media.resume_music_offset_err = false;
//    cias_media_set_pcm_state(MEDIA_PCM_START);
//    cias_media_set_skip_speak(false);
//    cias_media_pause_music();
//    cias_media_clear_cmd_queue();
//    cias_media_quit_connect();
//    cias_media_clear_suspension();
//    print_meida_state("uart recv:<2>");
//
//    set_audio_state(WAKE_UP);
}

volatile bool audio_player_need_data_1L = false;
volatile bool audio_player_need_data_2L = false;
volatile bool audio_player_need_data_3L = false;

#if CIAS_IOT_TVS_ENABLE
/**************************全局变量*************************/

static volatile uint16_t g_audio_state = 0;
static bool is_wakeup = false;

/**************************extern变量*************************/
//extern cias_media_info_t g_cias_media;
//extern cias_system_manage_param_t g_cias_system_manage_param;
extern bool is_set_volume;
#if defined(CONFIG_APP_CIAS_CLOUD_TVS) // TVS云端功能使能
extern unsigned char g_tvs_wakeup_msg[];
#endif

void cias_recv_tvs_msg(uint8_t *msg_buf) {
    cias_standard_head_t *phead = (cias_standard_head_t *) msg_buf;
    uint16_t recv_type = phead->type;
    uint32_t fill_data = phead->fill_data;

//	cias_status ret;
//	cias_raw_speech_t send_msg_speech;
    int32_t rent = 0;
    switch (recv_type) {
        case WAKE_UP: {
//			wakeup_deal();
            break;
        }
        case PLAY_DATA_GET: {
//			print_meida_state("uart recv:<3>");
//			//if (cias_get_sta_connect_status())
//			if (cias_get_wifi_sta_connect_state() == CIAS_WIFI_STA_CONNECTED)
//			{
//				if (cias_media_is_http_middle() &&
//					cias_media_is_audio_state(MEDIA_AUDIO_RESUME) &&
//					(g_cias_media.offset != phead->fill_data))
//				{
//					g_cias_media.resume_music_offset_err = true;
//					cias_media_set_audio_offset(phead->fill_data);
//					cias_media_quit_connect();
//					cias_media_clear_suspension();
//					cias_media_set_block_size(0);
//					cias_media_tencent_resume_music();
//					CIAS_PRINT_DEBUG("g_cias_media.offset != phead->fill_data[%d]\n", phead->fill_data);
//				}
//				else
//				{
//					CIAS_PRINT_DEBUG("PLAY_DATA_GET\r\n");
//					cias_media_set_pcm_state(MEDIA_PCM_INIT);
//					cias_media_clear_suspension();
//					cias_media_set_block_size(0);
//					if (g_cias_media.audio.state != MEDIA_AUDIO_PLAY)
//					{
//						cias_message_send_queue_clear();
//					}
//				}
//			}
//			print_meida_state("uart recv:<4>");
            break;
        }
        case PLAY_DATA_RECV: {
//			cias_media_clear_suspension();
//			cias_media_set_block_size(5); //请求5KB audio 数据
            audio_player_need_data_1L = true;
            break;
        }
        case PLAY_NEXT: {
//			CIAS_PRINT_DEBUG("PLAY_NEXT .........\r\n");
//			tvs_api_playcontrol_next();
            break;
        }
        case LOCAL_ASR_RESULT_NOTIFY: {
//			cias_media_set_pcm_state(MEDIA_PCM_FINISH);
//			cias_media_set_audio_offset(g_cias_media.audio.skip_prev_offset);
//			if(NETDEV_LINK_UP != netdev_get_link_state(netdev_get_active()))   //防止出现tvs释放资源出现野指针，系统卡死
//			{
//				return;
//			}
//			is_set_volume = false;
//			print_meida_state("uart recv:<5>");
//			cias_media_set_skip_speak(true); //and by yjd
//			tvs_api_stop_all_activity();
//			print_meida_state("uart recv:<6>");
            break;
        }
        case NET_PLAY_END : {
//			print_meida_state("uart recv:<7>");
//			if (g_cias_media.speak.state == MEDIA_SPEAK_SEND_END || g_cias_media.speak.state == MEDIA_SPEAK_PLAY)
//			{
//				g_cias_media.play_tts_end = false;
//				g_cias_media.speak.state = MEDIA_SPEAK_PLAY_END;
//			}
//			cias_media_set_skip_speak(false);
//			cias_media_clear_suspension();
//			cias_media_set_block_size(0);
//			cias_media_quit_connect();
//			if ((is_wakeup == false) || (g_cias_media.speak.state == MEDIA_SPEAK_PLAY_END))
//			{
//				if (get_http_data_is_complete() && g_cias_media.url_type == SPEAK_URL)
//				{
//					cias_media_recv_cmd(CIAS_RECV_PLAY_END_CMD);
//					CIAS_PRINT_INFO("play end\n");
//				}
//				else if (g_cias_media.audio.state == MEDIA_AUDIO_RESUME)
//				{
//					g_cias_media.audio.state = MEDIA_AUDIO_RESUME;
//					g_cias_media.speak.state = MEDIA_SPEAK_STOP;
//					CIAS_PRINT_INFO("play resume\n");
//				}
//				else if (g_cias_media.audio.state == MEDIA_AUDIO_PAUSE)
//				{
//					CIAS_PRINT_INFO("play pause\n");
//				}
//				else
//				{
//					g_cias_media.audio.state = MEDIA_AUDIO_PLAY;
//					g_cias_media.speak.state = MEDIA_SPEAK_STOP;
//					CIAS_PRINT_INFO("play start\n");
//				}
//			}
//			print_meida_state("uart recv:<8>");
            break;
        }
        case PLAY_TTS_END: //add by roy
        {
//			g_cias_media.stop_notify = false;
            break;
        }
        case SKIP_INVALID_SPEAK: {
//			print_meida_state("uart recv:<9>");
//			cias_message_send_queue_clear();
//			g_cias_media.speak.state = MEDIA_SPEAK_STOP;
//			cias_media_set_skip_speak(true);
//			cias_media_clear_cmd_queue();
//			cias_media_clear_suspension();
//			print_meida_state("uart recv:<10>");
            break;
        }
        case NET_PLAY_RECONNECT_URL: {
//			print_meida_state("uart recv:<11>");
//			if (MEDIA_AUDIO_EXCEPTION_RESUME == g_cias_media.audio.state)
//			{
//				g_cias_media.audio.state = MEDIA_AUDIO_RESUME;
//			}
//			CIAS_PRINT_DEBUG("recv offset = %d\n", phead->fill_data);
//			//if (!cias_get_sta_connect_status())
//			if (cias_get_wifi_sta_connect_state() != CIAS_WIFI_STA_CONNECTED)
//			{
//				g_cias_media.is_disconnect_wifi = false;
//			}
//			cias_media_set_audio_offset(phead->fill_data);
//			print_meida_state("uart recv:<12>");
            break;
        }
        case NET_PLAY_RESTART: //add by roy
        {
//			asr_set_resume_play_status();
//			tvs_media_player_inner_start_play();
            break;
        }
        case PCM_MIDDLE: {
//			rent = cias_send_pcm_middle(send_msg_speech, phead, msg_buf);
            break;
        }
        case PCM_FINISH: {
//			rent = cias_send_pcm_finish(send_msg_speech, phead, msg_buf);
            break;
        }
    }
}

#endif

// --- Class Method Implementations ---

SlaveMessageHandle::SlaveMessageHandle(uart_port_t uart_num) : uart_num(uart_num) {
    // Initialization moved to voicePortInit
}

SlaveMessageHandle::~SlaveMessageHandle() {
    // Clean up tasks and queues
    if (recvTaskHandle) {
        vTaskDelete(recvTaskHandle);
    }
    if (sendTaskHandle) {
        vTaskDelete(sendTaskHandle);
    }
    if (_messageRecvQueue) {
        vQueueDelete(_messageRecvQueue);
    }
    if (_messageSendQueue) {
        vQueueDelete(_messageSendQueue);
    }
    if (recvDealTaskHandle) {
        vTaskDelete(recvDealTaskHandle);
    }

    // Deinitialize UART (Important for proper resource cleanup)
    uart_driver_delete(uart_num);
}

int32_t slaveMsgHandle(uint8_t *msg_buf, int32_t msg_len);

int32_t SlaveMessageHandle::communication_recv(uint8_t *addr, int32_t length) {
    // Use ESP32 UART driver for reading
    if (length > VOICE_RECV_BUFF_LENGTH) {
        ESP_LOGE(TAG, "error length!!!\r\n");
        return -1;
    }

    size_t buffered_len;
    esp_err_t err = uart_get_buffered_data_len(uart_num, &buffered_len); // Check for available data
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_get_buffered_data_len failed: %s", esp_err_to_name(err));
        return -1; // Return an error on failure
    }

    if (buffered_len == 0) {
        return 0;
    }

    return uart_read_bytes(uart_num, addr, length, pdMS_TO_TICKS(10)); // 10ms timeout
}

int32_t SlaveMessageHandle::communication_send(const uint8_t *data, uint16_t length) {
    int bytes_written = uart_write_bytes(uart_num, (const char *) data, length);
    uart_wait_tx_done(uart_num, pdMS_TO_TICKS(100)); // Wait for completion!
    return bytes_written;
}

void SlaveMessageHandle::salveMessageRecvDealTask(void *pvParameters) {
    SlaveMessageHandle *pThis = static_cast<SlaveMessageHandle *>(pvParameters);
    if (!pThis) {
        ESP_LOGE(TAG, "salveMessageRecvDealTask: pThis is NULL!");
        vTaskDelete(NULL); // Delete self if instance pointer is invalid.
        return;
    }

    uint8_t recv_buf[VOICE_RECV_DATA_QUEUE_ITEAM_SIZE];

    while (true) {
        if (xQueueReceive(_messageRecvQueue, recv_buf, portMAX_DELAY) == pdTRUE) {
            cias_standard_head_t *received_header = (cias_standard_head_t *) recv_buf;

            int32_t result = slaveMsgHandle(recv_buf, received_header->len + sizeof(cias_standard_head_t));
            if (result != 0) { // Example of handling a return value. Adjust as needed.
                ESP_LOGW(TAG, "slaveMsgHandle returned an error: %ld", result);
            }
        } else {
            memset(recv_buf, 0, VOICE_RECV_DATA_QUEUE_ITEAM_SIZE);
        }
        vTaskDelay(pdMS_TO_TICKS(5)); // Small delay to prevent tight loops.
    }
}

void SlaveMessageHandle::slaveMessageRecvTask(void *pvParameters) {
    SlaveMessageHandle *pThis = static_cast<SlaveMessageHandle *>(pvParameters); // Get instance pointer.  CRITICAL
    if (!pThis) {
        ESP_LOGE(TAG, "slaveMessageRecvTask: pThis is NULL!");
        vTaskDelete(NULL); // Delete self
        return;
    }

    slave_recv_state_t slave_msg_state = MSG_FIND_HEAD;
    uint8_t *recv_buffer = nullptr;  // 指向分配缓冲区的指针。
    uint16_t received_len = 0;
    cias_standard_head_t *header = nullptr;

    // 在这里分配一个缓冲池，在循环外部。
    uint8_t recv_buffer_pool[VOICE_RECV_DATA_QUEUE_ITEAM_COUNT][VOICE_RECV_DATA_QUEUE_ITEAM_SIZE];

    while (true) {
        // 从池中获取一个缓冲区（或动态分配）。
        // 在真实实现中，应该有一个机制来管理该池。
        recv_buffer = recv_buffer_pool[0];  // 简化示例。

        switch (slave_msg_state) {
            case MSG_FIND_HEAD: {
                int32_t bytes_read = pThis->communication_recv(recv_buffer + received_len,
                                                               sizeof(cias_standard_head_t) - received_len);
                if (bytes_read > 0) {
                    received_len += bytes_read;
                    if (received_len == sizeof(cias_standard_head_t)) {
                        header = reinterpret_cast<cias_standard_head_t *>(recv_buffer);
                        if (header->magic == CIAS_STANDARD_MAGIC) {
                            slave_msg_state = (header->len == 0) ? MSG_VERIFY : MSG_RECV_MSG;
                        } else {
                            ESP_LOGE(TAG, "无效的魔数: %08x", header->magic);
                            received_len = 0; // 重置
                            //TODO 如果持续获取到坏数据，考虑在这里清空 UART。
                        }
                    }

                } else if (bytes_read < 0) {
                    ESP_LOGE(TAG, "UART 读取错误: %s", esp_err_to_name(bytes_read));
                    received_len = 0;
                }
                break;
            }

            case MSG_RECV_MSG: {
                int32_t bytes_read = pThis->communication_recv(
                        recv_buffer + sizeof(cias_standard_head_t) + received_len, header->len - received_len);
                if (bytes_read > 0) {
                    received_len += bytes_read;
                    if (received_len == header->len) {
                        slave_msg_state = MSG_VERIFY;
                    }

                } else if (bytes_read < 0) {
                    ESP_LOGE(TAG, "UART 读取错误: %s", esp_err_to_name(bytes_read));
                    received_len = 0;
                    slave_msg_state = MSG_FIND_HEAD; // 重置

                }

                break;
            }

            case MSG_VERIFY: {

                // 校验和验证
                if (pThis->calculateChecksum(recv_buffer,
                                             sizeof(cias_standard_head_t) + header->len - sizeof(header->checksum)) !=
                    header->checksum) {
                    ESP_LOGE(TAG, "校验和不匹配！");
                    received_len = 0;
                    slave_msg_state = MSG_FIND_HEAD;
                    break;
                }
                // 处理接收到的消息
                if (xQueueSend(_messageRecvQueue, &recv_buffer, pdMS_TO_TICKS(10)) != pdPASS) {
                    ESP_LOGE(TAG, "将接收到的消息发送到队列失败");
                    // 处理队列满的情况。丢弃消息，重试等。
                } else {
                    // 缓冲区返回到池
                    recv_buffer = nullptr; // 标记为可用。
                }
                received_len = 0;
                slave_msg_state = MSG_FIND_HEAD; // 返回到查找头部
                break;
            }

            default: {
                ESP_LOGE(TAG, "无效状态: %d", slave_msg_state);
                slave_msg_state = MSG_FIND_HEAD; // 重置
                received_len = 0;
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5)); // Reduce tight loop delay
    }
}

void SlaveMessageHandle::slaveMessageSendTask(void *pvParameters) {
    SlaveMessageHandle *pThis = static_cast<SlaveMessageHandle *>(pvParameters);
    if (!pThis) {
        ESP_LOGE(TAG, "slaveMessageSendTask: pThis is NULL!");
        vTaskDelete(NULL); // Delete self
        return;
    }

    cias_send_msg_t send_msg;
    while (true) {
        if (xQueueReceive(_messageSendQueue, &send_msg, portMAX_DELAY) == pdTRUE) {
            pThis->communication_send(send_msg.data, send_msg.length);
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // Avoid busy-waiting
    }
}

int SlaveMessageHandle::communicationTaskInit() {
    _messageRecvQueue = xQueueCreate(VOICE_RECV_DATA_QUEUE_ITEAM_COUNT, VOICE_RECV_DATA_QUEUE_ITEAM_SIZE);
    if (_messageRecvQueue == NULL) {
        ESP_LOGE(TAG, "Failed to create _messageRecvQueue");
        return VOICE_FAIL;
    }
    _messageSendQueue = xQueueCreate(VOICE_SEND_DATA_QUEUE_ITEAM_COUNT, VOICE_SEND_DATA_QUEUE_ITEAM_SIZE);
    if (_messageSendQueue == NULL) {
        ESP_LOGE(TAG, "Failed to create _messageSendQueue");
        return VOICE_FAIL;
    }

    // Pass 'this' to the task functions.  *CRITICAL*
    if (xTaskCreate(slaveMessageRecvTask, VOICE_RECV_DATA_FROM_AUDIO_TASK_NAME, VOICE_RECV_SLAVE_DATA_TASK_SIZE, this,
                    VOICE_RECV_SLAVE_DATA_TASK_PRIORITY, &recvTaskHandle) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create slaveMessageRecvTask");
        return VOICE_FAIL;
    }
    if (xTaskCreate(salveMessageRecvDealTask, VOICE_DEAL_RECV_AUDIO_DATA_TASK_NAME,
                    VOICE_DEAL_RECV_AUDIO_DATA_TASK_SIZE, this, VOICE_DEAL_RECV_AUDIO_DATA_TASK_PRIORITY,
                    &recvDealTaskHandle) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create salveMessageRecvDealTask");
        return VOICE_FAIL;
    }
    if (xTaskCreate(slaveMessageSendTask, VOICE_SEND_DATA_TO_AUDIO_TASK_NAME, VOICE_SEND_SLAVE_DATA_TASK_SIZE, this,
                    VOICE_SEND_SLAVE_DATA_TASK_PRIORITY, &sendTaskHandle) != pdPASS) { //Store Task handle
        ESP_LOGE(TAG, "Failed to create slaveMessageSendTask");
        return VOICE_FAIL;
    }
    return VOICE_OK;
}

int SlaveMessageHandle::voicePortInit(uart_port_t uart_num) {
    this->uart_num = uart_num; // Store the UART port number

    // Configure UART parameters using ESP32's uart_config_t
    uart_config_t uart_config = {
            .baud_rate = VOICE_UART_BAND_RATE,
            .data_bits = UART_DATA_8_BITS,
            .parity    = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, // No hardware flow control
            .source_clk = UART_SCLK_APB,  // Corrected clock source
    };

    // Install UART driver (allocate resources)
    esp_err_t err = uart_driver_install(uart_num, VOICE_UART_BUF_SIZE * 2, VOICE_UART_BUF_SIZE * 2, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART driver install failed: %s", esp_err_to_name(err));
        return VOICE_FAIL;
    }

    // Configure UART parameters
    err = uart_param_config(uart_num, &uart_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART param config failed: %s", esp_err_to_name(err));
        return VOICE_FAIL;
    }

    // Set UART pins
    err = uart_set_pin(uart_num, VOICE_UART_TX, VOICE_UART_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART set pin failed: %s", esp_err_to_name(err));
        return VOICE_FAIL;
    }

    return communicationTaskInit();
}

void SlaveMessageHandle::sendMessage(uint16_t cmd, cias_fill_type_t type, const uint8_t *data, uint16_t length) {
    // 1. Check length
    if (length > CIAS_SEND_MSG_BUF_LEN - 16) {
        ESP_LOGE(TAG, "sendMessage: Data length %u exceeds max buffer size %u.", length,
                 CIAS_SEND_MSG_BUF_LEN - sizeof(cias_standard_head_t));
        return;
    }
    // 2. Prepare message structure
    cias_send_msg_t send_msg;
//    memset(&send_msg, 0, sizeof(send_msg));
    unsigned int fill_data = 0;
    fill_data = type;
    /* 	if (cmd == NET_VOLUME)
    {
        fill_data = 0;
        len = 0;
    } */

    // 3. Initialize header
    send_msg.type = cmd;
    initSendMsgHeader(send_msg.data, 16, length, send_msg.type, fill_data, 0x00);

    // 4. Copy data (if present)
    if (data != nullptr && length > 0) {
        memcpy(send_msg.data + 16, data, length);
    }

    // 5. Calculate and set checksum
//    cias_standard_head_t* header = reinterpret_cast<cias_standard_head_t*>(send_msg.data);
//    header->checksum = calculateChecksum(send_msg.data + sizeof(header->magic), length + sizeof(cias_standard_head_t) - sizeof(header->checksum) - sizeof(header->magic));

    // 6. Set total message length
    send_msg.length = 16 + length;
    //send_msg.ack_flag is not set in this function

    // 7. Send the message
    if (cmd == GET_PROFILE || cmd == NEED_PROFILE) {
        // Direct, synchronous send.
        if (communication_send(send_msg.data, send_msg.length) < 0) {
            ESP_LOGE(TAG, "communication_send failed");
        }
    } else {
        // Send via queue (asynchronous).
        if (_messageSendQueue && xQueueSend(_messageSendQueue, &send_msg, pdMS_TO_TICKS(100)) != pdPASS) {
            ESP_LOGE(TAG, "Failed to send to queue");
        }
    }
}

void SlaveMessageHandle::clearSendQueue() {
    if (_messageSendQueue) {
        xQueueReset(_messageSendQueue);
    }
}

// --- Helper:  Initialize Message Header ---
int SlaveMessageHandle::initSendMsgHeader(uint8_t *buffer, uint16_t buffer_size, uint16_t data_len, uint16_t msg_type,
                                          uint32_t fill_data, uint16_t version) {
    memset(buffer, 0x00, buffer_size); // Good practice: Clear the buffer.
    if (buffer == nullptr) {
        return -1; // Error: Buffer too small or NULL.
    }
    cias_standard_head_t *header = (cias_standard_head_t *) buffer;
    header->magic = CIAS_STANDARD_MAGIC;
    header->type = msg_type;
    header->len = data_len;
    header->version = version; // You might want to make this a constant or a parameter.
    header->fill_data = fill_data;
    // Note: Checksum is calculated *after* data is copied.
    return 0; // Success.
}

#if CIAS_IOT_TVS_ENABLE

uint16_t SlaveMessageHandle::getAudioState() {
    return g_audio_state;
}

uint16_t SlaveMessageHandle::setAudioState(int state) {
    g_audio_state = state;
    return g_audio_state;
}

#endif

uint16_t SlaveMessageHandle::calculateChecksum(const uint8_t *data, uint16_t length) {
    uint16_t checksum = 0;
    for (int i = 0; i < length; i++) {
        checksum += data[i];
    }
    return checksum;
}

int32_t slaveMsgHandle(uint8_t *msg_buf, int32_t msg_len) {
    if (!msg_buf || msg_len <= 0) {
        return -1;
    }

#if CIAS_IOT_TVS_ENABLE
    cias_recv_tvs_msg(msg_buf);
#endif

    cias_standard_head_t *phead = (cias_standard_head_t *) msg_buf;
    uint16_t recv_type = phead->type;

    switch (recv_type) {
        case ENTER_NET_CONFIG:
            ESP_LOGI(TAG, "Received ENTER_NET_CONFIG");
            // Add your handling logic here
            break;
        case QCLOUD_IOT_CMD: {
            ESP_LOGI(TAG, "Received QCLOUD_IOT_CMD");
            if (msg_len < 18) break;
            uint16_t recv_cmd = (msg_buf[16]) | (msg_buf[17] << 8);
            ESP_LOGI(TAG, "  Command: 0x%04X", recv_cmd);

            // Add your handling logic here (e.g., call iot_light_recv_audio_cmd_handle)
            break;
        }
        case WAKE_UP:
            ESP_LOGI(TAG, "Received WAKE_UP");
            break;
        case LOCAL_ASR_RESULT_NOTIFY:
            ESP_LOGI(TAG, "Received LOCAL_ASR_RESULT_NOTIFY");
            break;
        case CIAS_OTA_DATA:
            ESP_LOGI(TAG, "Received CIAS_OTA_DATA");
            break;

        default:
            ESP_LOGW(TAG, "Unknown message type: 0x%04X", recv_type);
            break;
    }
    return 0; // Indicate success (adjust as needed)
}