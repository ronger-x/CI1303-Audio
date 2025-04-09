//
// Created by ronger on 2025/4/6.
//

#ifndef VOICE_MODULE_PROTOCOL_H
#define VOICE_MODULE_PROTOCOL_H

#include <cstdint>
#include <vector> // 使用 vector 来管理缓冲区更方便
#include <cstddef> // for size_t

#define VOICE_MODULE_PROTOCOL_HEADER 0x5a5aa5a5
#define VOICE_MODULE_PROTOCOL_TAIL 0xFB

#define VOICE_MODULE_MAX_PAYLOAD_SIZE  1024
#define VOICE_MODULE_HEADER_SIZE       (4 + 1 + 2 + 2) // header, type, seq, length
#define VOICE_MODULE_FOOTER_SIZE       (2 + 1)         // crc16, tail
#define VOICE_MODULE_MAX_PACKET_SIZE   (VOICE_MODULE_HEADER_SIZE + VOICE_MODULE_MAX_PAYLOAD_SIZE + VOICE_MODULE_FOOTER_SIZE)
#define VOICE_MODULE_MIN_PACKET_SIZE   (VOICE_MODULE_HEADER_SIZE + VOICE_MODULE_FOOTER_SIZE) // Packet with 0 payload

typedef enum {
  PACKET_TYPE_CMD = 0x01,             // 命令词指令
  PACKET_TYPE_START = 0x02,           // 开始传输指令
  PACKET_TYPE_DATA = 0x03,            // 数据指令
  PACKET_TYPE_END = 0x04,             // 结束传输指令
  PACKET_TYPE_ACK = 0x05,             // 应答指令（确认包 CI1303 -> ESP32）
  PACKET_TYPE_NACK = 0x06,            // 应答指令（否认/重传请求包 CI1303 -> ESP32）
  PACKET_TYPE_NOTIFY = 0x07,          // 通知指令
  PACKET_TYPE_STATUS = 0x08,          // 状态查询/回复
  PACKET_TYPE_ERROR = 0xFF,
} DataPacketType_t;

typedef struct {
    uint32_t header;
    uint8_t type;
    uint16_t seq;                     // 包序列号 (从0开始递增)
    uint16_t length;
    uint8_t* data;
    uint16_t crc16;                   // CRC16校验和 (覆盖 type 到 data 的所有字节)
    uint8_t tail;
} DataPacket_t;

// --- Unpacked Data Structure ---
// 注意：这里的 data 指针将指向外部提供的缓冲区或原始输入缓冲区的一部分
// Unpack 函数本身不负责 payload data 的内存分配管理
typedef struct {
    DataPacketType_t type;
    uint16_t seq;
    uint16_t length;           // Payload length
    const uint8_t* data;       // Pointer to the payload data within the source buffer
    // --- 不包含 header, crc, tail, 因为它们在解包时已被验证和移除 ---
} UnpackedPacketInfo_t;

class VoiceModuleProtocol {
public:
    /**
       * @brief 计算 CRC16-CCITT (Kermit variant)
       * @param data 指向要计算 CRC 的数据缓冲区
       * @param length 数据的长度（字节）
       * @return 计算出的 16 位 CRC 值
       */
    static uint16_t calculateCRC16(const uint8_t* data, size_t length);

    /**
     * @brief 将数据打包成协议格式的字节流
     * @param type 包类型 (DataPacketType_t)
     * @param seq 包序列号
     * @param payload 指向要包含的有效负载数据
     * @param payloadLength 有效负载数据的长度（字节）
     * @param outputPacket [out] 用于存储打包后数据的 std::vector<uint8_t>
     * @return true 如果打包成功, false 如果失败 (例如 payload 过长)
     */
    static bool pack(DataPacketType_t type, uint16_t seq, const uint8_t* payload, uint16_t payloadLength, std::vector<uint8_t>& outputPacket);

    /**
     * @brief 从字节流中解包数据
     * @param inputBuffer 指向包含原始数据的缓冲区
     * @param bufferLength inputBuffer 的长度（字节）
     * @param unpackedInfo [out] 用于存储解包后信息的结构体
     * @param consumedBytes [out] 如果解包成功，返回处理的字节数（即完整包的长度）
     * @return true 如果成功解包一个完整且有效的包, false 如果缓冲区数据不足、校验失败或格式错误
     */
    static bool unpack(const uint8_t* inputBuffer, size_t bufferLength, UnpackedPacketInfo_t& unpackedInfo, size_t& consumedBytes);
};



#endif //VOICE_MODULE_PROTOCOL_H
