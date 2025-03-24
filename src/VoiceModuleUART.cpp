/**
 * @file VoiceModuleUART.cpp
 * @brief Voice Module UART Protocol Library for ESP32
 * @version 1.0.0
 * @date 2025-01-11
 */

#include "VoiceModuleUART.h"

#define TIMEOUT_ONE_PACKET_INTERVAL 1000 // ms
#define VMUP_PACKET_MIN_SIZE 8
#define VMUP_PACKET_MAX_SIZE (VMUP_MSG_DATA_MAX_SIZE + VMUP_PACKET_MIN_SIZE)

VoiceModuleUART::VoiceModuleUART() :
    _serial(nullptr),
    _msgSeq(0),
    _isInitialized(false),
    _receiveState(REV_STATE_HEAD0),
    _dataRevCount(0),
    _lastReceiveTime(0)
{
    memset(&_receivedPacket, 0, sizeof(_receivedPacket));
}

VoiceModuleUART::~VoiceModuleUART() {
    // Nothing to clean up
}

bool VoiceModuleUART::begin(HardwareSerial& serial, int rxPin, int txPin, uint32_t baudRate) {
    _serial = &serial;
    _serial->begin(baudRate, SERIAL_8N1, rxPin, txPin);
    _isInitialized = true;
    _lastReceiveTime = millis();
    // Check if serial is available
    return _serial->available() >= 0;
}

void VoiceModuleUART::update() {
    if (!_isInitialized || !_serial) {
        return;
    }

    while (_serial->available()) {
        uint8_t receivedByte = _serial->read();
        processIncomingByte(receivedByte);
    }
}

void VoiceModuleUART::processIncomingByte(uint8_t receivedByte) {
    if (isPacketTimeout()) {
        _receiveState = REV_STATE_HEAD0;
    }

    switch (_receiveState) {
        case REV_STATE_HEAD0:
            {
                if (VMUP_MSG_HEAD_LOW == receivedByte) {
                    _receiveState = REV_STATE_HEAD1;
                } else {
                    _receiveState = REV_STATE_HEAD0;
                }
                break;
            }

        case REV_STATE_HEAD1:
            {
                if (VMUP_MSG_HEAD_HIGH == receivedByte) {
                    _receiveState = REV_STATE_LENGTH0;
                    _receivedPacket.header = VMUP_MSG_HEAD;
                } else {
                    if (VMUP_MSG_HEAD_LOW != receivedByte) {
                        _receiveState = REV_STATE_HEAD0;
                    }
                }
                break;
            }

        case REV_STATE_LENGTH0:
            {
                _length0 = receivedByte;
                _receiveState = REV_STATE_LENGTH1;
                break;
            }

        case REV_STATE_LENGTH1:
            {
                _length1 = receivedByte;
                _length1 <<= 8;
                _length1 += _length0;
                if (_length1 <= (VMUP_MSG_DATA_MAX_SIZE - 10)) {
                    _receivedPacket.data_length = _length1;
                    _receiveState = REV_STATE_TYPE;
                } else {
                    _receiveState = REV_STATE_HEAD0;
                }
                break;
            }

        case REV_STATE_TYPE:
            {
                _receivedPacket.msg_type = receivedByte;
                _receiveState = REV_STATE_CMD;
                break;
            }

        case REV_STATE_CMD:
            {
                _receivedPacket.msg_cmd = receivedByte;
                _receiveState = REV_STATE_SEQ;
                break;
            }

        case REV_STATE_SEQ:
           {
                _receivedPacket.msg_seq = receivedByte;
                if (_length1 > 0) {
                    _receiveState = REV_STATE_DATA;
                    _dataRevCount = 0;
                } else {
                    _receiveState = REV_STATE_CHECK_SUM_0;
                }
                break;
            }

        case REV_STATE_DATA:
            {
                _receivedPacket.msg_data[_dataRevCount++] = receivedByte;
                if (_dataRevCount == _length1) {
                    _receiveState = REV_STATE_CHECK_SUM_0;
                }
                break;
            }

        case REV_STATE_CHECK_SUM_0:
           {
                _checkSum0 = receivedByte;
                _receiveState = REV_STATE_CHECK_SUM_1;
                break;
            }

        case REV_STATE_CHECK_SUM_1:
            {
                _checkSum1 = receivedByte;
                _checkSum1 <<= 8;
                _checkSum1 += _checkSum0;

                uint16_t packetCheckSum = calculateChecksum(0, (uint8_t*)&_receivedPacket.msg_type, 3);
                packetCheckSum = calculateChecksum(packetCheckSum, _receivedPacket.msg_data, _receivedPacket.data_length);

                if (_checkSum1 == packetCheckSum) {
                    _receiveState = REV_STATE_TAIL;
                } else {
                    _receiveState = REV_STATE_HEAD0;
                }
                break;
            }

        case REV_STATE_TAIL:
           {
                if (receivedByte == VMUP_MSG_TAIL) {
                    // Message received successfully
                    handleReceivedMessage();
                }
                _receiveState = REV_STATE_HEAD0;
                break;
           }

        default:
           {
              _receiveState = REV_STATE_HEAD0;
               break;
           }
    }
}

uint16_t VoiceModuleUART::calculateChecksum(uint16_t initVal, const uint8_t* data, uint16_t length) {
    uint16_t checkSum = initVal;
    for (uint32_t i = 0; i < length; i++) {
        checkSum += data[i];
    }
    return checkSum;
}

bool VoiceModuleUART::isPacketTimeout() {
    uint32_t currentTime = millis();
    uint32_t elapsed = currentTime - _lastReceiveTime;

    _lastReceiveTime = currentTime;

    return (elapsed > TIMEOUT_ONE_PACKET_INTERVAL);
}

void VoiceModuleUART::handleReceivedMessage() {
    // Process the received message
    if (_messageCallback) {
        _messageCallback(_receivedPacket);
    }

    // Handle specific message types
    if (_receivedPacket.msg_cmd == VMUP_MSG_CMD_ASR_RESULT && _asrResultCallback) {
        _asrResultCallback(_receivedPacket.msg_seq, _receivedPacket.msg_data, _receivedPacket.data_length);
    }
    else if (_receivedPacket.msg_cmd == VMUP_MSG_CMD_NOTIFY_STATUS && _statusCallback) {
        if (_receivedPacket.data_length > 0) {
            _statusCallback(_receivedPacket.msg_data[0]);
        }
    }
}

bool VoiceModuleUART::sendCommand(uint8_t msgType, uint8_t msgCmd, const uint8_t* data, uint16_t dataLen) {
    if (!_isInitialized || !_serial) {
        return false;
    }

    if (dataLen > VMUP_MSG_DATA_MAX_SIZE) {
        return false;
    }

    sys_msg_com_data_t packet;

    // Build message header
    packet.header = VMUP_MSG_HEAD;
    packet.data_length = dataLen;
    packet.msg_type = msgType;
    packet.msg_cmd = msgCmd;
    packet.msg_seq = _msgSeq++;

    // Copy data if any
    if (dataLen > 0 && data != nullptr) {
        memcpy(packet.msg_data, data, dataLen);
    }

    // Calculate checksum
    packet.chk_sum = calculateChecksum(0, (uint8_t*)&packet.msg_type, 3);
    packet.chk_sum = calculateChecksum(packet.chk_sum, packet.msg_data, packet.data_length);

    // Set tail
    packet.tail = VMUP_MSG_TAIL;

    // Write packet to serial
    _serial->write((uint8_t*)&packet.header, 2);
    _serial->write((uint8_t*)&packet.data_length, 2);
    _serial->write(&packet.msg_type, 1);
    _serial->write(&packet.msg_cmd, 1);
    _serial->write(&packet.msg_seq, 1);

    if (dataLen > 0) {
        _serial->write(packet.msg_data, dataLen);
    }

    _serial->write((uint8_t*)&packet.chk_sum, 2);
    _serial->write(&packet.tail, 1);

    return true;
}

bool VoiceModuleUART::playVoiceById(uint8_t voiceId) {
    uint8_t data[2] = {VMUP_MSG_DATA_PLAY_BY_VOICEID, voiceId};
    return sendCommand(VMUP_MSG_TYPE_CMD_DOWN, VMUP_MSG_CMD_PLAY_VOICE, data, 2);
}

bool VoiceModuleUART::playVoiceBySemanticId(uint8_t semanticId) {
    uint8_t data[2] = {VMUP_MSG_DATA_PLAY_BY_SEMANTIC_ID, semanticId};
    return sendCommand(VMUP_MSG_TYPE_CMD_DOWN, VMUP_MSG_CMD_PLAY_VOICE, data, 2);
}

bool VoiceModuleUART::controlPlayback(uint8_t control) {
    uint8_t data[1] = {control};
    return sendCommand(VMUP_MSG_TYPE_CMD_DOWN, VMUP_MSG_CMD_PLAY_VOICE, data, 1);
}

bool VoiceModuleUART::getVersion(uint8_t versionType) {
    uint8_t data[1] = {versionType};
    return sendCommand(VMUP_MSG_TYPE_CMD_DOWN, VMUP_MSG_CMD_GET_VERSION, data, 1);
}

bool VoiceModuleUART::resetModule() {
    return sendCommand(VMUP_MSG_TYPE_CMD_DOWN, VMUP_MSG_CMD_RESET_MODULE, nullptr, 0);
}

bool VoiceModuleUART::setVolume(uint8_t volume) {
    if (volume > 100) volume = 100;
    uint8_t data[2] = {VMUP_MSG_CMD_SET_VOLUME, volume};
    return sendCommand(VMUP_MSG_TYPE_CMD_DOWN, VMUP_MSG_CMD_SET_CONFIG, data, 2);
}

bool VoiceModuleUART::setWakeupMode(bool enter) {
    uint8_t data[2] = {VMUP_MSG_CMD_SET_ENTERWAKEUP, static_cast<uint8_t>(enter ? 1 : 0)};
    return sendCommand(VMUP_MSG_TYPE_CMD_DOWN, VMUP_MSG_CMD_SET_CONFIG, data, 2);
}

bool VoiceModuleUART::setMute(bool mute) {
    uint8_t data[2] = {VMUP_MSG_CMD_SET_MUTE, static_cast<uint8_t>(mute ? 1 : 0)};
    return sendCommand(VMUP_MSG_TYPE_CMD_DOWN, VMUP_MSG_CMD_SET_CONFIG, data, 2);
}

bool VoiceModuleUART::getFlashUID() {
    return sendCommand(VMUP_MSG_TYPE_CMD_DOWN, VMUP_MSG_CMD_GET_FLASHUID, nullptr, 0);
}

void VoiceModuleUART::setMessageCallback(MessageCallback callback) {
    _messageCallback = callback;
}

void VoiceModuleUART::setAsrResultCallback(AsrResultCallback callback) {
    _asrResultCallback = callback;
}

void VoiceModuleUART::setStatusNotifyCallback(StatusNotifyCallback callback) {
    _statusCallback = callback;
}