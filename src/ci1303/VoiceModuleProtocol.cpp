//
// Created by ronger on 2025/4/6.
//

#include "VoiceModuleProtocol.h"
#include <cstring> // for memcpy
#include <stdexcept> // for error reporting (optional)

// --- CRC16 Lookup Table ---
// Static linkage: Makes the table local to this translation unit (this .cpp file)
static const uint16_t table_crc16_ccitt[256]=
{
    0x0000,0x1021,0x2042,0x3063,0x4084,0x50a5,0x60c6,0x70e7,
    0x8108,0x9129,0xa14a,0xb16b,0xc18c,0xd1ad,0xe1ce,0xf1ef,
    0x1231,0x0210,0x3273,0x2252,0x52b5,0x4294,0x72f7,0x62d6,
    0x9339,0x8318,0xb37b,0xa35a,0xd3bd,0xc39c,0xf3ff,0xe3de,
    0x2462,0x3443,0x0420,0x1401,0x64e6,0x74c7,0x44a4,0x5485,
    0xa56a,0xb54b,0x8528,0x9509,0xe5ee,0xf5cf,0xc5ac,0xd58d,
    0x3653,0x2672,0x1611,0x0630,0x76d7,0x66f6,0x5695,0x46b4,
    0xb75b,0xa77a,0x9719,0x8738,0xf7df,0xe7fe,0xd79d,0xc7bc,
    0x48c4,0x58e5,0x6886,0x78a7,0x0840,0x1861,0x2802,0x3823,
    0xc9cc,0xd9ed,0xe98e,0xf9af,0x8948,0x9969,0xa90a,0xb92b,
    0x5af5,0x4ad4,0x7ab7,0x6a96,0x1a71,0x0a50,0x3a33,0x2a12,
    0xdbfd,0xcbdc,0xfbbf,0xeb9e,0x9b79,0x8b58,0xbb3b,0xab1a,
    0x6ca6,0x7c87,0x4ce4,0x5cc5,0x2c22,0x3c03,0x0c60,0x1c41,
    0xedae,0xfd8f,0xcdec,0xddcd,0xad2a,0xbd0b,0x8d68,0x9d49,
    0x7e97,0x6eb6,0x5ed5,0x4ef4,0x3e13,0x2e32,0x1e51,0x0e70,
    0xff9f,0xefbe,0xdfdd,0xcffc,0xbf1b,0xaf3a,0x9f59,0x8f78,
    0x9188,0x81a9,0xb1ca,0xa1eb,0xd10c,0xc12d,0xf14e,0xe16f,
    0x1080,0x00a1,0x30c2,0x20e3,0x5004,0x4025,0x7046,0x6067,
    0x83b9,0x9398,0xa3fb,0xb3da,0xc33d,0xd31c,0xe37f,0xf35e,
    0x02b1,0x1290,0x22f3,0x32d2,0x4235,0x5214,0x6277,0x7256,
    0xb5ea,0xa5cb,0x95a8,0x8589,0xf56e,0xe54f,0xd52c,0xc50d,
    0x34e2,0x24c3,0x14a0,0x0481,0x7466,0x6447,0x5424,0x4405,
    0xa7db,0xb7fa,0x8799,0x97b8,0xe75f,0xf77e,0xc71d,0xd73c,
    0x26d3,0x36f2,0x0691,0x16b0,0x6657,0x7676,0x4615,0x5634,
    0xd94c,0xc96d,0xf90e,0xe92f,0x99c8,0x89e9,0xb98a,0xa9ab,
    0x5844,0x4865,0x7806,0x6827,0x18c0,0x08e1,0x3882,0x28a3,
    0xcb7d,0xdb5c,0xeb3f,0xfb1e,0x8bf9,0x9bd8,0xabbb,0xbb9a,
    0x4a75,0x5a54,0x6a37,0x7a16,0x0af1,0x1ad0,0x2ab3,0x3a92,
    0xfd2e,0xed0f,0xdd6c,0xcd4d,0xbdaa,0xad8b,0x9de8,0x8dc9,
    0x7c26,0x6c07,0x5c64,0x4c45,0x3ca2,0x2c83,0x1ce0,0x0cc1,
    0xef1f,0xff3e,0xcf5d,0xdf7c,0xaf9b,0xbfba,0x8fd9,0x9ff8,
    0x6e17,0x7e36,0x4e55,0x5e74,0x2e93,0x3eb2,0x0ed1,0x1ef0
};

/**
 * @brief (Internal) Calculates CRC16-CCITT using the lookup table.
 *        NOTE: The parameter 'pre_crc' allows for calculating CRC over multiple chunks,
 *              but our protocol calculates it over a single contiguous block.
 * @param pre_crc Starting CRC value (usually 0 for a new calculation).
 * @param data Pointer to the data buffer.
 * @param length Length of the data in bytes.
 * @return The calculated CRC16 value.
 */
static uint16_t crc16_ccitt_lookup(uint16_t pre_crc, const uint8_t * data, size_t length) // Changed uint32_t to size_t for consistency
{
    uint16_t crc = pre_crc; // Use the provided starting CRC
    while(length--)
    {
        // Calculate the index for the table lookup
        uint8_t index = (uint8_t)((crc >> 8) ^ *data++); // Index uses the high byte of CRC XORed with data byte
        // Update CRC: Shift left by 8 and XOR with the table value
        crc = (crc << 8) ^ table_crc16_ccitt[index];
    }
    return crc;
}


// --- Modify the existing calculateCRC16 method to use the lookup function ---
// This static method remains the public interface for CRC calculation within the class
uint16_t VoiceModuleProtocol::calculateCRC16(const uint8_t* data, size_t length) {
    // For this protocol, we always start the CRC calculation from 0
    // for the specified data block (type to payload).
    return crc16_ccitt_lookup(0x0000, data, length);
}

// --- Pack Implementation ---
bool VoiceModuleProtocol::pack(DataPacketType_t type, uint16_t seq, const uint8_t* payload, uint16_t payloadLength, std::vector<uint8_t>& outputPacket) {
    if (payloadLength > VOICE_MODULE_MAX_PAYLOAD_SIZE) {
        // Payload too large
        return false;
    }

    size_t totalPacketSize = VOICE_MODULE_HEADER_SIZE + payloadLength + VOICE_MODULE_FOOTER_SIZE;
    outputPacket.resize(totalPacketSize);

    uint8_t* buffer = outputPacket.data();
    size_t offset = 0;

    // 1. Header (4 bytes, assuming little endian)
    uint32_t header = VOICE_MODULE_PROTOCOL_HEADER;
    memcpy(buffer + offset, &header, sizeof(header));
    offset += sizeof(header);

    // 2. Type (1 byte)
    buffer[offset++] = static_cast<uint8_t>(type);

    // 3. Sequence Number (2 bytes, assuming little endian)
    memcpy(buffer + offset, &seq, sizeof(seq));
    offset += sizeof(seq);

    // 4. Payload Length (2 bytes, assuming little endian)
    memcpy(buffer + offset, &payloadLength, sizeof(payloadLength));
    offset += sizeof(payloadLength);

    // 5. Payload Data (payloadLength bytes)
    if (payloadLength > 0 && payload != nullptr) {
        memcpy(buffer + offset, payload, payloadLength);
        offset += payloadLength;
    } else if (payloadLength > 0 && payload == nullptr){
        // Error: Declared payload length but data pointer is null
         return false;
    }
    // If payloadLength is 0, offset remains unchanged, which is correct.

    // --- Calculate CRC ---
    // CRC covers from Type up to the end of Payload Data
    const uint8_t* crcDataStart = buffer + sizeof(uint32_t); // Start from Type field
    size_t crcDataLength = sizeof(uint8_t) + sizeof(uint16_t) + sizeof(uint16_t) + payloadLength; // type + seq + length + payload_data
    uint16_t crc = calculateCRC16(crcDataStart, crcDataLength);

    // 6. CRC16 (2 bytes, assuming little endian)
    memcpy(buffer + offset, &crc, sizeof(crc));
    offset += sizeof(crc);

    // 7. Tail (1 byte)
    buffer[offset++] = VOICE_MODULE_PROTOCOL_TAIL;

    // Final size check (should match calculation)
    if (offset != totalPacketSize) {
       // Internal logic error
       outputPacket.clear(); // Clear potentially corrupt data
       return false;
    }

    return true;
}

// --- Unpack Implementation ---
bool VoiceModuleProtocol::unpack(const uint8_t* inputBuffer, size_t bufferLength, UnpackedPacketInfo_t& unpackedInfo, size_t& consumedBytes) {
    // Check minimum possible packet length
    if (bufferLength < VOICE_MODULE_MIN_PACKET_SIZE) {
        return false; // Not enough data for even a header+footer
    }

    size_t offset = 0;

    // 1. Check Header
    uint32_t header;
    memcpy(&header, inputBuffer + offset, sizeof(header));
    if (header != VOICE_MODULE_PROTOCOL_HEADER) {
        // Invalid header. Consider searching for header if used in streaming context,
        // but for this function, we assume the buffer starts with a packet.
        return false;
    }
    offset += sizeof(header);

    // 2. Read Type
    DataPacketType_t type = static_cast<DataPacketType_t>(inputBuffer[offset++]);

    // 3. Read Sequence Number
    uint16_t seq;
    memcpy(&seq, inputBuffer + offset, sizeof(seq));
    offset += sizeof(seq);

    // 4. Read Payload Length
    uint16_t payloadLength;
    memcpy(&payloadLength, inputBuffer + offset, sizeof(payloadLength));
    offset += sizeof(payloadLength);

    // Check if declared payload length is valid and if we have enough data
    if (payloadLength > VOICE_MODULE_MAX_PAYLOAD_SIZE) {
        return false; // Declared payload exceeds max size
    }

    size_t expectedTotalPacketSize = VOICE_MODULE_HEADER_SIZE + payloadLength + VOICE_MODULE_FOOTER_SIZE;
    if (bufferLength < expectedTotalPacketSize) {
        return false; // Not enough data in the buffer for the declared packet size
    }

    // Pointer to payload start within the input buffer
    const uint8_t* payloadDataStart = inputBuffer + offset;
    offset += payloadLength; // Move offset past the payload

    // --- CRC Check ---
    // Read the CRC from the buffer
    uint16_t receivedCrc;
    memcpy(&receivedCrc, inputBuffer + offset, sizeof(receivedCrc));
    offset += sizeof(receivedCrc);

    // Calculate CRC on the received data (Type to Payload)
    const uint8_t* crcDataStart = inputBuffer + sizeof(uint32_t); // Start from Type
    size_t crcDataLength = sizeof(uint8_t) + sizeof(uint16_t) + sizeof(uint16_t) + payloadLength; // type + seq + length + payload
    uint16_t calculatedCrc = calculateCRC16(crcDataStart, crcDataLength);

    if (receivedCrc != calculatedCrc) {
        return false; // CRC mismatch
    }

    // 5. Check Tail
    uint8_t tail = inputBuffer[offset++];
    if (tail != VOICE_MODULE_PROTOCOL_TAIL) {
        return false; // Invalid tail
    }

    // --- All checks passed, fill the output struct ---
    unpackedInfo.type = type;
    unpackedInfo.seq = seq;
    unpackedInfo.length = payloadLength;
    unpackedInfo.data = payloadDataStart; // Point directly into the input buffer's payload section

    consumedBytes = expectedTotalPacketSize; // Report how many bytes this valid packet occupied

    return true;
}
