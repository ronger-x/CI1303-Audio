//
// Created by ronger on 2025/1/11.
//
#include <Arduino.h>
#include "FreeRTOS.h"
#include "task.h"
#include "voice_module_uart_protocol.h"

#define TIMEOUT_ONE_PACKET_INTERVAL (1000)/*ms, in this code, it should be bigger than portTICK_PERIOD_MS */
#define MAX_DATA_RECEIVE_PER_PACKET (80)/*???*/


#define VMUP_PACKET_MIN_SIZE (8)
#define VMUP_PACKET_MAX_SIZE (VMUP_MSG_DATA_MAX_SIZE + VMUP_PACKET_MIN_SIZE)

/* 唤醒互斥锁 */
SemaphoreHandle_t WakeupMutex = nullptr;

/* 退出唤醒定时器 */
xTimerHandle exit_wakeup_timer = nullptr;

/* 系统消息队列 */
static QueueHandle_t sys_msg_queue = nullptr;

/**
 * @brief 系统消息任务资源初始化
 *
 */
void sys_msg_task_initial()
{
    sys_msg_queue = xQueueCreate(16, sizeof(sys_msg_t));
    if (!sys_msg_queue)
    {
        Serial.printf("not enough memory:%d,%s\n",__LINE__, __FUNCTION__);
    }

    WakeupMutex = xSemaphoreCreateMutex();
    if (!WakeupMutex)
    {
        Serial.printf("not enough memory:%d,%s\n",__LINE__, __FUNCTION__);
    }
}


/**
 * @brief 检查当前是否位于中断/异常状态
 *
 * @retval 0 未处于中断/异常状态
 * @retval 1 处于中断/异常状态
 */
int32_t check_curr_trap()
{
    return xPortInIsrContext();
}

/**
 * @brief A simple code for other component send system message to this module, just wrap freertos queue function
 *
 * @param send_msg : system message
 * @param xHigherPriorityTaskWoken : if call this not from isr set NULL
 * @return BaseType_t
 */
BaseType_t send_msg_to_sys_task(sys_msg_t* send_msg, BaseType_t* xHigherPriorityTaskWoken)
{
    if (0 != check_curr_trap())
    {
        return xQueueSendFromISR(sys_msg_queue, send_msg, xHigherPriorityTaskWoken);
    }
    else
    {
        return xQueueSend(sys_msg_queue, send_msg, 0);
    }
}

int vmup_port_send_packet_rev_msg(sys_msg_com_data_t* msg)
{
    sys_msg_t send_msg;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    send_msg.msg_type = SYS_MSG_TYPE_COM;
    if (sizeof(sys_msg_com_data_t) > sizeof(send_msg.msg_data))
    {
        Serial.printf("vmup_port_send_packet_rev_msg: msg data size error\n");
    }
    memcpy(&send_msg.msg_data, msg, sizeof(sys_msg_com_data_t));

    const BaseType_t xResult = send_msg_to_sys_task(&send_msg, &xHigherPriorityTaskWoken);

    if ((xResult != pdFAIL) && (pdTRUE == xHigherPriorityTaskWoken))
    {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
    return xHigherPriorityTaskWoken;
}

int vmup_port_send_audio_data_msg(sys_msg_com_data_t* msg)
{
    sys_msg_t send_msg;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    send_msg.msg_type = SYS_MSG_TYPE_PLAY_OUTSIDE;
    if (sizeof(sys_msg_com_data_t) > sizeof(send_msg.msg_data))
    {
        Serial.printf("vmup_port_send_packet_rev_msg: msg data size error\n");
    }
    memcpy(&send_msg.msg_data, msg, sizeof(sys_msg_com_data_t));

    const BaseType_t xResult = send_msg_to_sys_task(&send_msg, &xHigherPriorityTaskWoken);

    if ((xResult != pdFAIL) && (pdTRUE == xHigherPriorityTaskWoken))
    {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
    return xHigherPriorityTaskWoken;
}


TickType_t last_time;

static bool vmup_port_timeout_one_packet()
{
    TickType_t now_time = xTaskGetTickCountFromISR();

    TickType_t timeout = (now_time - last_time); /*uint type, so overflow just used - */

    last_time = now_time;

    if (timeout > TIMEOUT_ONE_PACKET_INTERVAL / portTICK_PERIOD_MS) /*also as timeout = timeout*portTICK_PERIOD_MS;*/
    {
        return true;
    }
    else
    {
        return false;
    }
}


static uint16_t vmup_port_checksum(uint16_t init_val, uint8_t* data, uint16_t length)
{
    uint16_t check_sum_ = init_val;
    for (uint32_t i = 0; i < length; i++)
    {
        check_sum_ += data[i];
    }

    return check_sum_;
}


/********************************************************************
                     receive function
********************************************************************/
/*receive use state machine method, so two char can arrive different time*/
typedef enum
{
    REV_STATE_HEAD0 = 0x00,
    REV_STATE_HEAD1 = 0x01,
    REV_STATE_LENGTH0 = 0x02,
    REV_STATE_LENGTH1 = 0x03,
    REV_STATE_TYPE = 0x04,
    REV_STATE_CMD = 0x05,
    REV_STATE_SEQ = 0x06,
    REV_STATE_DATA = 0x07,
    REV_STATE_CHECK_SUM_0 = 0x08,
    REV_STATE_CHECK_SUM_1 = 0x09,
    REV_STATE_TAIL = 0x0a,
} vmup_receive_state_t;

sys_msg_com_data_t recever_packet;
static uint8_t rev_state = REV_STATE_HEAD0;
static uint16_t length0 = 0, length1 = 0;
static uint16_t check_sum_0 = 0, check_sum_1 = 0;
static uint16_t data_rev_count = 0;


void vmup_receive_packet(uint8_t receive_char)
{
    if (true == vmup_port_timeout_one_packet())
    {
        rev_state = REV_STATE_HEAD0;
    }

    switch (rev_state)
    {
    case REV_STATE_HEAD0:
        if (VMUP_MSG_HEAD_LOW == receive_char)
        {
            rev_state = REV_STATE_HEAD1;
        }
        else
        {
            rev_state = REV_STATE_HEAD0;
        }
        break;
    case REV_STATE_HEAD1:
        if (VMUP_MSG_HEAD_HIGH == receive_char)
        {
            rev_state = REV_STATE_LENGTH0;
            recever_packet.header = VMUP_MSG_HEAD;
        }
        else
        {
            if (VMUP_MSG_HEAD_LOW != receive_char)
            {
                rev_state = REV_STATE_HEAD0;
            }
        }
        break;
    case REV_STATE_LENGTH0:
        length0 = receive_char;
        rev_state = REV_STATE_LENGTH1;
        break;
    case REV_STATE_LENGTH1:
        length1 = receive_char;
        length1 <<= 8;
        length1 += length0;
        if (length1 <= (VMUP_MSG_DATA_MAX_SIZE - 10))
        {
            recever_packet.data_length = length1;
            rev_state = REV_STATE_TYPE;
        }
        else
        {
            rev_state = REV_STATE_HEAD0;
        }
        break;
    case REV_STATE_TYPE:
        recever_packet.msg_type = receive_char;
        rev_state = REV_STATE_CMD;
        break;
    case REV_STATE_CMD:
        recever_packet.msg_cmd = receive_char;
        rev_state = REV_STATE_SEQ;
        break;
    case REV_STATE_SEQ:
        recever_packet.msg_seq = receive_char;
        if (length1 > 0)
        {
            rev_state = REV_STATE_DATA;
            data_rev_count = 0;
        }
        else
        {
            rev_state = REV_STATE_CHECK_SUM_0;
        }
        break;
    case REV_STATE_DATA:
        recever_packet.msg_data[data_rev_count++] = receive_char;
        if (data_rev_count == length1)
        {
            rev_state = REV_STATE_CHECK_SUM_0;
        }
        break;
    case REV_STATE_CHECK_SUM_0:
        check_sum_0 = receive_char;
        rev_state = REV_STATE_CHECK_SUM_1;
        break;
    case REV_STATE_CHECK_SUM_1:
        {
            check_sum_1 = receive_char;
            check_sum_1 <<= 8;
            check_sum_1 += check_sum_0;
            /*recever_packet->chksum = check_sum_1; just used as judgement*/
            uint16_t packet_check_sum = vmup_port_checksum(0, (uint8_t*)&recever_packet.msg_type, 3);
            packet_check_sum = vmup_port_checksum(packet_check_sum, recever_packet.msg_data,
                                                  recever_packet.data_length);
            if (check_sum_1 == packet_check_sum)
            {
                rev_state = REV_STATE_TAIL;
            }
            else
            {
                rev_state = REV_STATE_HEAD0;
            }
            break;
        }
    case REV_STATE_TAIL:
        if (receive_char == VMUP_MSG_TAIL) /*receive ok*/
        {
            /*recever_packet->tail = receive_char; just used as judgement*/
            vmup_port_send_packet_rev_msg(&recever_packet);
        }
        else
        {
            data_rev_count = 0;
        }
        rev_state = REV_STATE_HEAD0;
        break;
    default:
        rev_state = REV_STATE_HEAD0;
        break;
    }
}
