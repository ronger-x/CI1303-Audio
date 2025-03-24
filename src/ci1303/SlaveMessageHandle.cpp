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
#define VOICE_RECV_BUFF_LEGTH 1024

// --- Status Codes ---
#define VOICE_OK 0
#define VOICE_FAIL 1

// --- Global Variables ---
// HardwareSerial* _serial = nullptr; // Removed global _serial.
QueueHandle_t _messageRecvQueue = nullptr;
QueueHandle_t _messageSendQueue = nullptr;

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

int32_t SlaveMessageHandle::communication_recv(uint8_t *addr, int32_t length)
{
    // Use ESP32 UART driver for reading
    return uart_read_bytes(uart_num, addr, length, pdMS_TO_TICKS(10)); // 10ms timeout
}

void SlaveMessageHandle::salveMessageRecvDealTask(void *pvParameters) {
  SlaveMessageHandle* pThis = static_cast<SlaveMessageHandle*>(pvParameters);
    if (!pThis) {
        ESP_LOGE(TAG, "salveMessageRecvDealTask: pThis is NULL!");
        vTaskDelete(NULL); // Delete self if instance pointer is invalid.
        return;
    }

    uint8_t recv_buf[VOICE_RECV_DATA_QUEUE_ITEAM_SIZE];

    while (true) {
        if (xQueueReceive(_messageRecvQueue, recv_buf, portMAX_DELAY) == pdTRUE) {
            cias_standard_head_t *received_header = (cias_standard_head_t *)recv_buf;

            if (received_header->magic != CIAS_STANDARD_MAGIC) {
                ESP_LOGE(TAG, "Invalid magic number received: %08X", (unsigned int)received_header->magic);
                continue; // Discard the message.
            }

            // Calculate checksum *using the instance pointer*:
            uint16_t calculated_checksum = pThis->calculateChecksum(
                recv_buf + sizeof(received_header->magic),
                received_header->len + sizeof(cias_standard_head_t) - sizeof(received_header->magic) - sizeof(received_header->checksum)
            );
            //Correctly declare and get received checksum
            uint16_t received_checksum = received_header->checksum;
            if (received_checksum != calculated_checksum) {
                ESP_LOGE(TAG, "Checksum mismatch. Received: %04X, Calculated: %04X", received_checksum, calculated_checksum);
                continue;  // Discard the message.
            }

            // Process message based on type:
            switch (received_header->type) {
                case LOCAL_ASR_RESULT_NOTIFY:
                    ESP_LOGI(TAG, "Received LOCAL_ASR_RESULT_NOTIFY");
                    break;
                case WAKE_UP:
                    ESP_LOGI(TAG, "Received WAKE_UP");
                    break;
                case NET_PLAY_START:
                    ESP_LOGI(TAG, "Received NET_PLAY_START");
                    break;
                case QCLOUD_IOT_CMD:
                    ESP_LOGI(TAG, "Received QCLOUD_IOT_CMD");
                    break;
                case CIAS_OTA_DATA:
                    ESP_LOGI(TAG, "Received  CIAS_OTA_DATA");
                    break;
                case CIAS_IR_DATA:
                    ESP_LOGI(TAG, "Received CIAS_IR_DATA");
                    break;
                // ... Add other cases ...
                default:
                    ESP_LOGW(TAG, "Unknown message type received: %04X", received_header->type);
                    break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5)); // Small delay to prevent tight loops.
    }
}

void SlaveMessageHandle::slaveMessageRecvTask(void* pvParameters) {
    SlaveMessageHandle* pThis = static_cast<SlaveMessageHandle*>(pvParameters); // Get instance pointer.  CRITICAL
     if (!pThis) {
        ESP_LOGE(TAG, "slaveMessageRecvTask: pThis is NULL!");
        vTaskDelete(NULL); // Delete self
        return;
    }

    int32_t ret = 0;
    uint16_t temp = 0;
    uint32_t data_len = 0;
    cias_standard_head_t *recv_headpart = nullptr; // Initialize to nullptr
    slave_recv_state_t slave_msg_state = MSG_FIND_HEAD;

    uint8_t recv_slave_buf[VOICE_RECV_DATA_QUEUE_ITEAM_SIZE];

    memset(recv_slave_buf, 0, VOICE_RECV_DATA_QUEUE_ITEAM_SIZE);
    vTaskDelay(pdMS_TO_TICKS(1000)); // Convert to ticks for better accuracy.

    while (1) {
        if (slave_msg_state == MSG_FIND_HEAD) {
            ret = pThis->communication_recv(recv_slave_buf + data_len, sizeof(cias_standard_head_t) - data_len); //Use pThis. CRITICAL

             if (ret > 0) { // Check for > 0, not >= 0, as uart_read_bytes returns the number of bytes *actually* read.
                 data_len += ret;
                 if (data_len < sizeof(cias_standard_head_t)) {
                     vTaskDelay(pdMS_TO_TICKS(5));
                     continue;
                  } else {
                        recv_headpart = (cias_standard_head_t *)recv_slave_buf;
                        data_len = 0;

                        if (recv_headpart->magic == CIAS_STANDARD_MAGIC) {
                            if (recv_headpart->len == 0){
                                slave_msg_state = MSG_VERIFY;
                             } else {
                                slave_msg_state = MSG_RECV_MSG;

                             }

                        } else {
                            ESP_LOGE(TAG, "recv_headpart magic err1: Got 0x%08x, expected 0x%08x", (unsigned int)recv_headpart->magic, (unsigned int)CIAS_STANDARD_MAGIC);
                            slave_msg_state = MSG_FIND_HEAD;
                            memset(recv_slave_buf, 0, VOICE_RECV_DATA_QUEUE_ITEAM_SIZE);
                        }
                  }

            } else if (ret < 0) {
                // Handle UART errors (e.g., UART_FIFO_OVF, UART_FRAME_ERR)
                ESP_LOGE(TAG, "UART receive error: %d", (int)ret);
                 slave_msg_state = MSG_FIND_HEAD; // Reset state on error.
                memset(recv_slave_buf, 0, VOICE_RECV_DATA_QUEUE_ITEAM_SIZE); // Clear the buffer.

            } // else ret == 0:  No data received, just continue the loop.
        }

         if (slave_msg_state == MSG_RECV_MSG) {
            //  ESP_LOGI(TAG, "Entering MSG_RECV_MSG, expected data length: %u", recv_headpart->len);

            ret = pThis->communication_recv(recv_slave_buf + sizeof(cias_standard_head_t) + data_len,
                                            (recv_headpart->len - data_len)); //Use pThis
             if (ret > 0) {  //Check > 0
                data_len += ret;
                if (data_len < recv_headpart->len) {
                    vTaskDelay(pdMS_TO_TICKS(2));  // Shorter delay for data reception.
                      continue;
                } else {
                     data_len = 0;
                     slave_msg_state = MSG_VERIFY;
                 }
            } else if (ret < 0) {
                  ESP_LOGE(TAG, "recv_headpart data err2: %d", (int)ret);
                    data_len = 0;
                    memset(recv_slave_buf, 0, VOICE_RECV_DATA_QUEUE_ITEAM_SIZE);
                   slave_msg_state = MSG_FIND_HEAD;
            }
        }

        if (slave_msg_state == MSG_VERIFY) {
          slave_msg_state = MSG_FIND_HEAD;

          // Checksum Calculation (Before sending to queue)
        //   uint16_t calculated_checksum = pThis->calculateChecksum(recv_slave_buf + 4, recv_headpart->len + sizeof(cias_standard_head_t)-8);
        //     if (calculated_checksum != recv_headpart->checksum) {
        //          ESP_LOGE(TAG, "Checksum mismatch.  Calculated: %04X, Received: %04X", calculated_checksum, recv_headpart->checksum);
        //         memset(recv_slave_buf, 0, VOICE_RECV_DATA_QUEUE_ITEAM_SIZE);
        //         continue; // Don't send the message if checksum is bad
        //      }

            // Send to queue (use a pointer to avoid large copies)
            if (xQueueSend(_messageRecvQueue, recv_slave_buf, pdMS_TO_TICKS(10)) != pdPASS) {
                ESP_LOGE(TAG, "_messageRecvQueue send fail...");
            }
            data_len = 0;
            memset(recv_slave_buf, 0, VOICE_RECV_DATA_QUEUE_ITEAM_SIZE);
        }
        vTaskDelay(pdMS_TO_TICKS(5)); // Reduce tight loop delay
    }
}

void SlaveMessageHandle::slaveMessageSendTask(void* pvParameters) {
  SlaveMessageHandle* pThis = static_cast<SlaveMessageHandle*>(pvParameters);
    if (!pThis) {
        ESP_LOGE(TAG, "slaveMessageSendTask: pThis is NULL!");
        vTaskDelete(NULL); // Delete self
        return;
    }

    cias_send_msg_t send_msg;
    while (true) {
		if (xQueueReceive(_messageSendQueue, &send_msg, portMAX_DELAY) == pdTRUE)
		{
			// 1. Create the complete message (header + data)
			uint8_t complete_message[sizeof(cias_standard_head_t) + send_msg.length];
			cias_standard_head_t *header = (cias_standard_head_t *)complete_message;

			header->magic = CIAS_STANDARD_MAGIC;
			header->type = send_msg.type;
			header->len = send_msg.length;
			header->version = 1; // Set a version (good practice)
			header->fill_data = DEF_FILL;  // Or use send_msg.type if appropriate.

            // Copy the data payload
			memcpy(complete_message + sizeof(cias_standard_head_t), send_msg.data, send_msg.length);

            // Calculate checksum
            header->checksum = pThis->calculateChecksum( complete_message + sizeof(header->magic) , send_msg.length + sizeof(cias_standard_head_t) - sizeof(header->checksum) - sizeof(header->magic) );

			// 2. Send the complete message using ESP32 UART driver
			int bytes_written = uart_write_bytes(pThis->uart_num, (const char *)complete_message, sizeof(cias_standard_head_t) + send_msg.length);

            if (bytes_written < 0)
			{
				ESP_LOGE(TAG, "UART write failed: %d", bytes_written);
				// Handle the error appropriately (retry, reset, etc.)
			} else if (bytes_written != sizeof(cias_standard_head_t) + send_msg.length) {
                 ESP_LOGW(TAG, "UART write incomplete.  Expected %d, wrote %d", sizeof(cias_standard_head_t) + send_msg.length, bytes_written);
            }
            else
			{
				ESP_LOGD(TAG, "Sent message, type: 0x%04X, length: %u, checksum: 0x%04X", header->type, header->len, header->checksum);
			}

             // Wait for the UART transmission to complete (important!)
            uart_wait_tx_done(pThis->uart_num, pdMS_TO_TICKS(100)); // 100ms timeout

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
    if (xTaskCreate(slaveMessageRecvTask, VOICE_RECV_DATA_FROM_AUDIO_TASK_NAME, VOICE_RECV_SLAVE_DATA_TASK_SIZE, this, VOICE_RECV_SLAVE_DATA_TASK_PRIORITY, &recvTaskHandle) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create slaveMessageRecvTask");
        return VOICE_FAIL;
    }
	if (xTaskCreate(salveMessageRecvDealTask, VOICE_DEAL_RECV_AUDIO_DATA_TASK_NAME, VOICE_DEAL_RECV_AUDIO_DATA_TASK_SIZE, this, VOICE_DEAL_RECV_AUDIO_DATA_TASK_PRIORITY, &recvDealTaskHandle) != pdPASS)
	{
		ESP_LOGE(TAG,"Failed to create salveMessageRecvDealTask");
		return VOICE_FAIL;
	}
    if (xTaskCreate(slaveMessageSendTask, VOICE_SEND_DATA_TO_AUDIO_TASK_NAME, VOICE_SEND_SLAVE_DATA_TASK_SIZE, this, VOICE_SEND_SLAVE_DATA_TASK_PRIORITY, &sendTaskHandle) != pdPASS) { //Store Task handle
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

void SlaveMessageHandle::sendMessage(uint16_t cmd, cias_fill_type_t type, const uint8_t *data, uint16_t length)
{
    if (length > CIAS_SEND_MSG_BUF_LEN) {
        ESP_LOGE(TAG, "sendMessage: Data length exceeds buffer size.");
        return; // Or handle the error differently
    }
	cias_send_msg_t message;
    memset(&message, 0, sizeof(message)); //Clear any previous data
	message.type = cmd;
	message.length = length;
	  if (data != nullptr && length > 0) {
        memcpy(message.data, data, length); // Copy the data, if provided.
     }
	message.ack_flag = 0; // Set the ack flag appropriately.

    if (_messageSendQueue != NULL) {
        if (xQueueSend(_messageSendQueue, &message, pdMS_TO_TICKS(100)) != pdPASS)  //Use address of message
        {
            ESP_LOGE(TAG, "Failed to send message to queue");
            // Handle queue send failure (e.g., queue full)
        }
    } else {
        ESP_LOGE(TAG, "_messageSendQueue is NULL. Cannot send message.");
    }
}

uint16_t SlaveMessageHandle::getAudioState() {
    // Placeholder implementation.  Replace with actual logic to retrieve
    // the audio playback state, likely involving a mutex for thread safety.
    //  return audio_state;
    return 0; // Replace with actual implementation.
}

uint16_t SlaveMessageHandle::setAudioState(int state) {
    // Placeholder implementation. Replace with actual state setting logic.
    // audio_state = state;
    // return audio_state;
      return 0; // Replace with actual implementation.
}

 uint16_t SlaveMessageHandle::calculateChecksum(const uint8_t *data, uint16_t length) {
    uint16_t checksum = 0;
    for (int i = 0; i < length; i++) {
        checksum += data[i];
    }
    return checksum;
}