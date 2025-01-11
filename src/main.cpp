#include <Arduino.h>
#include "uart.h"

#include "voice_module_uart_protocol.h"

#include "mp3_data.h"

/************************* 串口变量声明 ****************************/
#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)
#define EX_UART_NUM 1 // set uart1
#define PATTERN_CHR_NUM (3)

#define UART_PIN_TX GPIO_NUM_9 // 串口发送引脚GPIO_9
#define UART_PIN_RX GPIO_NUM_10 // 串口接收引脚GPIO_10

static QueueHandle_t uart1_queue; // 串口接收队列,当串口完成数据接收后，发送消息给该队列，只需在线程等待该消息队列完成数据处理


// 定义消息队列
QueueHandle_t network_msg_recv_queue;

void uart_init(); // 串口初始化
void Uart_data_process(uint8_t* data, size_t len); // 串口数据处理函数
static void uart_event_task(void* pvParameters); // 接收串口处理函数

void processHexData(const char* hexData, uint8_t* outputArray, size_t& outputSize);
void printHexArray(const uint8_t* hexArray, size_t size);

void setup()
{
    // write your initialization code here
    Serial.begin(115200);
    uart_init();
    xTaskCreate(uart_event_task, "uart_event_task", 2048, nullptr, 12, nullptr); // 串口中断处理线程
}

void loop()
{
    vTaskDelay(5000 / portTICK_PERIOD_MS);
}

uint16_t msg_seq = 1;

void test_uart()
{
    // 输出数组
    uint8_t hexArray[20]; // 每次处理20个字节
    size_t chunkSize = 20; // 每次处理的字节数
    // 分块处理数据
    size_t dataLength = strlen(mp3_data);
    size_t offset = 0;
    while (offset < dataLength) {
        // 提取当前块的40个字符（20个字节）
        const char* chunkData = mp3_data + offset;
        size_t chunkLength = min(chunkSize * 2, dataLength - offset); // 每次最多取40个字符

        // 处理当前块
        processHexData(chunkData, hexArray, chunkSize);

        // 打印当前块的结果
        Serial.print("Processing chunk starting at offset ");
        Serial.print(offset);
        Serial.println(":");
        printHexArray(hexArray, chunkSize);

        sys_msg_com_data_t* msg = new sys_msg_com_data_t();
        msg->header = VMUP_MSG_HEAD;
        msg->data_length = chunkSize;
        memcpy(msg->msg_data, hexArray, chunkSize);
        msg->msg_type = VMUP_MSG_TYPE_CMD_DOWN;
        msg->msg_cmd = VMUP_MSG_CMD_USER_START;
        msg->msg_seq = msg_seq;
        msg->tail = VMUP_MSG_TAIL;
        vmup_port_send_packet_rev_msg(msg);
        msg_seq++;

        // 更新偏移量
        offset += chunkLength;

        // 如果剩余数据不足40个字符，跳出循环
        if (chunkLength < 40) {
            break;
        }
    }
}

// 函数：将字符串形式的16进制数据转换为16进制数组
void processHexData(const char* hexData, uint8_t* outputArray, size_t& outputSize) {
    size_t len = strlen(hexData);
    outputSize = 0;

    for (size_t i = 0; i < len; i += 2) {
        // 提取两个字符
        char byteStr[3];
        byteStr[0] = hexData[i];
        byteStr[1] = hexData[i + 1];
        byteStr[2] = '\0'; // 字符串结束符

        // 将字符串转换为16进制字节
        outputArray[outputSize++] = strtol(byteStr, NULL, 16);
    }
}

// 函数：打印16进制数组
void printHexArray(const uint8_t* hexArray, size_t size) {
    Serial.println("{");
    for (size_t i = 0; i < size; ++i) {
        Serial.print("0x");
        if (hexArray[i] < 0x10) {
            Serial.print("0"); // 补零
        }
        Serial.print(hexArray[i], HEX);
        if (i != size - 1) {
            Serial.print(", ");
        }
        // 每16个字节换行
        if ((i + 1) % 16 == 0) {
            Serial.println();
        }
    }
    Serial.println("\n};");
}

/*************************  串口初始化  ****************************/
void uart_init()
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    Serial2.begin(115200, SERIAL_8N1, UART_PIN_TX, UART_PIN_RX, false, 20000UL, 112);

    uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart1_queue, 0); // 安装串口驱动，并关联队列uart1_queue
    uart_param_config(EX_UART_NUM, &uart_config);

    uart_set_pin(EX_UART_NUM, UART_PIN_TX, UART_PIN_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE); // 设置串口引脚（TX:9,RX:10）
    uart_enable_pattern_det_baud_intr(EX_UART_NUM, '+', PATTERN_CHR_NUM, 9, 0, 0); // Set uart pattern detect function.
    uart_pattern_queue_reset(EX_UART_NUM, 20); // Reset the pattern queue length to record at most 20 pattern positions.
}


/*************************  串口中断事件处理线程  ****************************/
static void uart_event_task(void* pvParameters)
{
    uart_event_t event;
    size_t buffered_size;
    uint8_t* dtmp = (uint8_t*)malloc(RD_BUF_SIZE);
    for (;;)
    {
        if (xQueueReceive(uart1_queue, (void*)&event, (TickType_t)portMAX_DELAY))
        {
            bzero(dtmp, RD_BUF_SIZE);
            switch (event.type)
            {
            case UART_DATA:
                Serial.printf("[UART DATA]: %d", event.size);
                uart_read_bytes(EX_UART_NUM, dtmp, event.size, portMAX_DELAY); // 读出接收到的数据
                uart_write_bytes(EX_UART_NUM, (const char*)dtmp, event.size); // 打印接收到的数据
                Uart_data_process(dtmp, event.size); // 处理接收到的数据
                break;
            default:
                Serial.printf("uart event type: %d", event.type);
                break;
            }
        }
    }
    free(dtmp);
    dtmp = nullptr;
    vTaskDelete(nullptr);
}


/*************************   串口数据处理  ****************************/
void Uart_data_process(uint8_t* data, size_t len)
{
    // 示例数据  A5 FC 07 00 A0 91 07 01 55 E4 01 00 00 A8 1B 03 FB
    // 检查数据长度
    if (len == 0 || len < 10) return;
    // 获取前两个字节
    uint16_t head = (data[0] << 8) | data[1];
    // 获取最后一个字节
    uint8_t tail = data[len - 1];
    if (head != 0xa5fc || tail != 0xfb) return;
    // 获取数据长度
    uint16_t data_len = (data[2] << 8) | data[3];
    // 获取消息类型
    uint8_t msg_type = data[4];
    Serial.printf("msg_type: %d", msg_type);
    // 获取消息序号
    uint8_t msg_seq = data[5];
    Serial.printf("msg_seq: %d", msg_seq);
    // 获取数据内容
    char* data_str = (char*)malloc(data_len + 1);
    memcpy(data_str, data + 6, data_len);
    data_str[data_len] = '\0';
    Serial.printf("data_str: %s", data_str);
    free(data_str);
    // 获取校验和
    uint16_t check_sum = 0;
}
