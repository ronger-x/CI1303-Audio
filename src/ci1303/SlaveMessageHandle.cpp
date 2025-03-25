#include "SlaveMessageHandle.h"
#include "CommunicationPortForAudio.h"
#include <string.h>

static const char *TAG = "SlaveMessageHandle";

// --- 全局变量 ---
volatile bool audio_player_need_data_1L = false;
volatile bool audio_player_need_data_2L = false;
volatile bool audio_player_need_data_3L = false;

#define CLOUD_IOT_ENABLE 1

/**************************全局变量*************************/
static volatile uint16_t g_audio_state = 0;
static bool is_wakeup = false;
/**************************extern变量*************************/
extern bool is_set_volume;

// --- 获取音频状态 ---
uint16_t SlaveMessageHandle::getAudioState() {
    return g_audio_state;
}

// --- 设置音频状态 ---
uint16_t SlaveMessageHandle::setAudioState(int state) {
    g_audio_state = state;
    return g_audio_state;
}
//TODO 唤醒事件
void wakeupDeal()
{
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

    g_audio_state = WAKE_UP;
}
#if CLOUD_IOT_ENABLE

// 云端消息处理函数
void cloudRecvMessageHandle(uint8_t *msg_buf)
{
    cias_standard_head_t *phead = (cias_standard_head_t *)msg_buf;
    uint16_t recv_type = phead->type;
    uint32_t fill_data = phead->fill_data;

//    cias_status ret;
//    cias_raw_speech_t send_msg_speech;
    int32_t rent = 0;

    switch (recv_type) {
        case WAKE_UP:
        {
            wakeupDeal();
            break;
        }
        case PLAY_DATA_GET:
        {
//            print_meida_state("uart recv:<3>");
//            //if (cias_get_sta_connect_status())
//            if (cias_get_wifi_sta_connect_state() == CIAS_WIFI_STA_CONNECTED)
//            {
//                if (cias_media_is_http_middle() &&
//                    cias_media_is_audio_state(MEDIA_AUDIO_RESUME) &&
//                    (g_cias_media.offset != phead->fill_data))
//                {
//                    g_cias_media.resume_music_offset_err = true;
//                    cias_media_set_audio_offset(phead->fill_data);
//                    cias_media_quit_connect();
//                    cias_media_clear_suspension();
//                    cias_media_set_block_size(0);
//                    cias_media_tencent_resume_music();
//                    CIAS_PRINT_DEBUG("g_cias_media.offset != phead->fill_data[%d]\n", phead->fill_data);
//                }
//                else
//                {
//                    CIAS_PRINT_DEBUG("PLAY_DATA_GET\r\n");
//                    cias_media_set_pcm_state(MEDIA_PCM_INIT);
//                    cias_media_clear_suspension();
//                    cias_media_set_block_size(0);
//                    if (g_cias_media.audio.state != MEDIA_AUDIO_PLAY)
//                    {
//                        cias_message_send_queue_clear();
//                    }
//                }
//            }
//            print_meida_state("uart recv:<4>");
            break;
        }
        case PLAY_DATA_RECV:
        {
//            cias_media_clear_suspension();
//            cias_media_set_block_size(5); //请求5KB audio 数据
            audio_player_need_data_1L = true;
            break;
        }
        case PLAY_NEXT:
        {
//            CIAS_PRINT_DEBUG("PLAY_NEXT .........\r\n");
//            tvs_api_playcontrol_next();
            break;
        }
        case LOCAL_ASR_RESULT_NOTIFY:
        {
//            cias_media_set_pcm_state(MEDIA_PCM_FINISH);
//            cias_media_set_audio_offset(g_cias_media.audio.skip_prev_offset);
//            if(NETDEV_LINK_UP != netdev_get_link_state(netdev_get_active()))   //防止出现tvs释放资源出现野指针，系统卡死
//            {
//                return;
//            }
//            is_set_volume = false;
//            print_meida_state("uart recv:<5>");
//            cias_media_set_skip_speak(true); //and by yjd
//            tvs_api_stop_all_activity();
//            print_meida_state("uart recv:<6>");
            break;
        }
        case NET_PLAY_END :
        {
//            print_meida_state("uart recv:<7>");
//            if (g_cias_media.speak.state == MEDIA_SPEAK_SEND_END || g_cias_media.speak.state == MEDIA_SPEAK_PLAY)
//            {
//                g_cias_media.play_tts_end = false;
//                g_cias_media.speak.state = MEDIA_SPEAK_PLAY_END;
//            }
//            cias_media_set_skip_speak(false);
//            cias_media_clear_suspension();
//            cias_media_set_block_size(0);
//            cias_media_quit_connect();
//            if ((is_wakeup == false) || (g_cias_media.speak.state == MEDIA_SPEAK_PLAY_END))
//            {
//                if (get_http_data_is_complete() && g_cias_media.url_type == SPEAK_URL)
//                {
//                    cias_media_recv_cmd(CIAS_RECV_PLAY_END_CMD);
//                    CIAS_PRINT_INFO("play end\n");
//                }
//                else if (g_cias_media.audio.state == MEDIA_AUDIO_RESUME)
//                {
//                    g_cias_media.audio.state = MEDIA_AUDIO_RESUME;
//                    g_cias_media.speak.state = MEDIA_SPEAK_STOP;
//                    CIAS_PRINT_INFO("play resume\n");
//                }
//                else if (g_cias_media.audio.state == MEDIA_AUDIO_PAUSE)
//                {
//                    CIAS_PRINT_INFO("play pause\n");
//                }
//                else
//                {
//                    g_cias_media.audio.state = MEDIA_AUDIO_PLAY;
//                    g_cias_media.speak.state = MEDIA_SPEAK_STOP;
//                    CIAS_PRINT_INFO("play start\n");
//                }
//            }
//            print_meida_state("uart recv:<8>");
            break;
        }
        case PLAY_TTS_END: //add by roy
        {
//            g_cias_media.stop_notify = false;
            break;
        }
        case SKIP_INVALID_SPEAK:
        {
//            print_meida_state("uart recv:<9>");
//            cias_message_send_queue_clear();
//            g_cias_media.speak.state = MEDIA_SPEAK_STOP;
//            cias_media_set_skip_speak(true);
//            cias_media_clear_cmd_queue();
//            cias_media_clear_suspension();
//            print_meida_state("uart recv:<10>");
            break;
        }
        case NET_PLAY_RECONNECT_URL:
        {
//            print_meida_state("uart recv:<11>");
//            if (MEDIA_AUDIO_EXCEPTION_RESUME == g_cias_media.audio.state)
//            {
//                g_cias_media.audio.state = MEDIA_AUDIO_RESUME;
//            }
//            CIAS_PRINT_DEBUG("recv offset = %d\n", phead->fill_data);
//            //if (!cias_get_sta_connect_status())
//            if (cias_get_wifi_sta_connect_state() != CIAS_WIFI_STA_CONNECTED)
//            {
//                g_cias_media.is_disconnect_wifi = false;
//            }
//            cias_media_set_audio_offset(phead->fill_data);
//            print_meida_state("uart recv:<12>");
            break;
        }
        case NET_PLAY_RESTART: //add by roy
        {
//            asr_set_resume_play_status();
//            tvs_media_player_inner_start_play();
            break;
        }
        case PCM_MIDDLE:
        {
//            rent = cias_send_pcm_middle(send_msg_speech, phead, msg_buf);
            break;
        }
        case PCM_FINISH:
        {
//            rent = cias_send_pcm_finish(send_msg_speech, phead, msg_buf);
            break;
        }
    }
}
#endif

// --- 构造函数和析构函数 ---
SlaveMessageHandle::SlaveMessageHandle() {
    // 默认构造
}

SlaveMessageHandle::~SlaveMessageHandle() {
    // 清理任务
    if (recvDealTaskHandle) {
        vTaskDelete(recvDealTaskHandle);
        recvDealTaskHandle = nullptr;
    }
}

// --- 消息处理回调函数 ---
int32_t SlaveMessageHandle::messageHandlerCallback(uint8_t *msg_buf, int32_t msg_len) {
    if (!msg_buf || msg_len <= 0) {
        return VOICE_FAIL;
    }

#if CLOUD_IOT_ENABLE
    cloudRecvMessageHandle(msg_buf);
#endif

    cias_standard_head_t *phead = (cias_standard_head_t *)msg_buf;
    uint16_t recv_type = phead->type;

    switch (recv_type) {
        case ENTER_NET_CONFIG:
            ESP_LOGI(TAG, "Received ENTER_NET_CONFIG");
            break;
        case QCLOUD_IOT_CMD: {
            ESP_LOGI(TAG, "Received QCLOUD_IOT_CMD");
            if (msg_len < 18) break;
            uint16_t recv_cmd = (msg_buf[16]) | (msg_buf[17] << 8);
            ESP_LOGI(TAG, "  Command: 0x%04X", recv_cmd);
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
    return 0;
}

// --- 处理接收消息的任务 ---
void SlaveMessageHandle::slaveMessageRecvDealTask(void *pvParameters) {
    SlaveMessageHandle *pThis = static_cast<SlaveMessageHandle *>(pvParameters);
    if (!pThis) {
        ESP_LOGE(TAG, "slaveMessageRecvDealTask: pThis is NULL!");
        vTaskDelete(NULL);
        return;
    }

    uint8_t recv_buf[VOICE_RECV_DATA_QUEUE_ITEAM_SIZE];

    while (true) {
        if (xQueueReceive(messageRecvQueue, recv_buf, portMAX_DELAY) == pdTRUE) {
            cias_standard_head_t *received_header = (cias_standard_head_t *) recv_buf;

            // 处理收到的消息
            messageHandlerCallback(recv_buf, received_header->len + sizeof(cias_standard_head_t));
        } else {
            memset(recv_buf, 0, VOICE_RECV_DATA_QUEUE_ITEAM_SIZE);
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// --- 初始化通信任务 ---
int SlaveMessageHandle::messageHandlerInit() {
    if (g_commPort.communicationTaskInit() != VOICE_OK) {
        return VOICE_FAIL;
    }
    // 创建消息处理任务
    BaseType_t xTaskCreateResult = xTaskCreate(
            slaveMessageRecvDealTask,
            VOICE_DEAL_RECV_AUDIO_DATA_TASK_NAME,
            VOICE_DEAL_RECV_AUDIO_DATA_TASK_SIZE,
            this,
            VOICE_DEAL_RECV_AUDIO_DATA_TASK_PRIORITY,
            &recvDealTaskHandle
    );

    if (xTaskCreateResult != pdPASS) {
        ESP_LOGE(TAG, "Failed to create slaveMessageRecvDealTask");
        return VOICE_FAIL;
    }

    return VOICE_OK;
}

// --- 发送消息 ---
void SlaveMessageHandle::sendMessage(uint16_t cmd, cias_fill_type_t type, const uint8_t *data, uint16_t length) {
    // 检查长度
    if (length > CIAS_SEND_MSG_BUF_LEN - 16) {
        ESP_LOGE(TAG, "sendMessage: Data length %u exceeds max buffer size %u.", length, CIAS_SEND_MSG_BUF_LEN - sizeof(cias_standard_head_t));
        return;
    }

    // 准备消息缓冲区
    uint8_t *buffer = (uint8_t *)malloc(16 + length);
    if (!buffer) {
        ESP_LOGE(TAG, "Failed to allocate memory for message");
        return;
    }

    // 初始化消息头
    initSendMsgHeader(buffer, 16, length, cmd, type, 0x00);

    // 复制数据
    if (data != nullptr && length > 0) {
        memcpy(buffer + 16, data, length);
    }

    // 直接发送或通过队列发送
    if (cmd == GET_PROFILE || cmd == NEED_PROFILE) {
        // 直接同步发送
        int32_t result = g_commPort.communicationSend(buffer, 16 + length);
        if (result < 0) {
            ESP_LOGE(TAG, "communicationSend failed");
        }
        free(buffer);
    } else {
        // 通过队列异步发送
        comm_message_t msg;
        msg.data = buffer;
        msg.length = 16 + length;
        msg.type = cmd;
        msg.ack_flag = 1; // 需要释放内存

        if (xQueueSend(messageSendQueue, &msg, pdMS_TO_TICKS(100)) != pdPASS) {
            ESP_LOGE(TAG, "Failed to send to queue");
            free(buffer);
        }
    }
}

// --- 清空发送队列 ---
void SlaveMessageHandle::clearSendQueue() {
    if (messageSendQueue) {
        xQueueReset(messageSendQueue);
    }
}

// --- 初始化消息头 ---
int SlaveMessageHandle::initSendMsgHeader(uint8_t *buffer, uint16_t buffer_size, uint16_t data_len, uint16_t msg_type, uint32_t fill_data, uint16_t version) {
    memset(buffer, 0x00, buffer_size);
    if (buffer == nullptr) {
        return VOICE_FAIL;
    }
    cias_standard_head_t *header = (cias_standard_head_t *)buffer;
    header->magic = CIAS_STANDARD_MAGIC;
    header->type = msg_type;
    header->len = data_len;
    header->version = version;
    header->fill_data = fill_data;

    // 计算校验和
    header->checksum = calculateChecksum(buffer + 4, data_len + buffer_size - 4);

    return 0;
}

// --- 计算校验和 ---
uint16_t SlaveMessageHandle::calculateChecksum(const uint8_t *data, uint16_t length) {
    uint16_t checksum = 0;
    for (int i = 0; i < length; i++) {
        checksum += data[i];
    }
    return checksum;
}