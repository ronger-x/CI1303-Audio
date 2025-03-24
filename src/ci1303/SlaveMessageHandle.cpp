//
// Created by ronger on 2025/3/24.
//

#include "SlaveMessageHandle.h"
#include "Arduino.h"

#define VOICE_UART_TX GPIO_NUM_9 // 串口发送引脚 GPIO_9
#define VOICE_UART_RX GPIO_NUM_10 // 串口接收引脚 GPIO_10
#define VOICE_UART_BAND_RATE 1000000 // 1M
#define DEAL_RECV_AUDIO_DATA_TASK_SIZE              (1024*4)


static void salveMessageRecvDealTask(void* pvParameters) {
}

void slaveMessageRecvDataTask() {
    xTaskCreate(salveMessageRecvDealTask, "", 2048, nullptr, 12, nullptr);
}

SlaveMessageHandle::SlaveMessageHandle() :
    _serial(nullptr)
{

}

SlaveMessageHandle::~SlaveMessageHandle() {}

void SlaveMessageHandle::voicePortInit(HardwareSerial& serial) {
    _serial = &serial;
    _serial->begin(VOICE_UART_BAND_RATE, SERIAL_8N1, VOICE_UART_RX, VOICE_UART_TX);
    // Check if serial is available
    if (_serial->available() >= 0) {
        // 初始化消息处理任务
        slaveMessageRecvDataTask();
    }
}