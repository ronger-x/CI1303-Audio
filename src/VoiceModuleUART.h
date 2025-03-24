/**
 * @file VoiceModuleUART.h
 * @brief Voice Module UART Protocol Library for ESP32
 * @version 1.0.0
 * @date 2025-01-11
 * @author ronger
 */

#ifndef VOICE_MODULE_UART_H
#define VOICE_MODULE_UART_H

#include <Arduino.h>
#include <functional>

#define VMUP_MSG_DATA_MAX_SIZE (20)

// Message header definitions
#define VMUP_MSG_HEAD_LOW  (0xA5)
#define VMUP_MSG_HEAD_HIGH (0xFC)
#define VMUP_MSG_HEAD   ((VMUP_MSG_HEAD_HIGH<<8)|VMUP_MSG_HEAD_LOW)

// Message tail definition
#define VMUP_MSG_TAIL   (0xFB)

// Message types
#define VMUP_MSG_TYPE_CMD_UP   (0xA0)
#define VMUP_MSG_TYPE_CMD_DOWN (0xA1)
#define VMUP_MSG_TYPE_ACK      (0xA2)
#define VMUP_MSG_TYPE_NOTIFY   (0xA3)

// Message commands
#define VMUP_MSG_CMD_ASR_RESULT    (0x91)           // Report voice recognition result
#define VMUP_MSG_CMD_PLAY_VOICE    (0x92)           // Play local voice
#define VMUP_MSG_CMD_GET_FLASHUID  (0x93)           // Read FLASH serial number
#define VMUP_MSG_CMD_GET_VERSION   (0x94)           // Read version number
#define VMUP_MSG_CMD_RESET_MODULE  (0x95)           // Reset voice module
#define VMUP_MSG_CMD_SET_CONFIG    (0x96)           // Settings
#define VMUP_MSG_CMD_ENTER_OTA_MODE (0x97)          // Enter upgrade mode
#define VMUP_MSG_CMD_NOTIFY_STATUS (0x9A)           // Event notification
#define VMUP_MSG_CMD_ACK_COMMON    (0xAA)
// User commands start from here
#define VMUP_MSG_CMD_USER_START    (0xB0)

// Play voice command data definitions
#define VMUP_MSG_DATA_PLAY_START   (0x80)
#define VMUP_MSG_DATA_PLAY_PAUSE   (0x81)
#define VMUP_MSG_DATA_PLAY_RESUME  (0x82)
#define VMUP_MSG_DATA_PLAY_STOP    (0x83)
#define VMUP_MSG_DATA_PLAY_BY_VOICEID (0x90)
#define VMUP_MSG_DATA_PLAY_BY_SEMANTIC_ID   (0x91)
#define VMUP_MSG_DATA_PLAY_BY_CMD_ID   (0x92)

// Get version command data definitions
#define VMUP_MSG_DATA_VER_PROTOCOL   (0x80)         // UART protocol version
#define VMUP_MSG_DATA_VER_SDK        (0x81)         // SDK version
#define VMUP_MSG_DATA_VER_ASR        (0x82)         // ASR component version
#define VMUP_MSG_DATA_VER_PREPROCESS (0x83)         // Voice preprocessing algorithm version
#define VMUP_MSG_DATA_VER_PLAYER     (0x84)         // Player version
#define VMUP_MSG_DATA_VER_APP        (0x8A)         // Application version

// Notification status data definitions
#define VMUP_MSG_DATA_NOTIFY_POWERON     (0xB0)
#define VMUP_MSG_DATA_NOTIFY_WAKEUPENTER (0xB1)
#define VMUP_MSG_DATA_NOTIFY_WAKEUPEXIT  (0xB2)
#define VMUP_MSG_DATA_NOTIFY_PLAYSTART   (0xB3)
#define VMUP_MSG_DATA_NOTIFY_PLAYEND     (0xB4)

// Set config command data definitions
#define VMUP_MSG_CMD_SET_VOLUME        (0x80)
#define VMUP_MSG_CMD_SET_ENTERWAKEUP   (0x81)
#define VMUP_MSG_CMD_SET_PRT_MID_RST   (0x82)
#define VMUP_MSG_CMD_SET_MUTE          (0x83)
#define VMUP_MSG_CMD_SET_NEEDACK       (0x90)
#define VMUP_MSG_CMD_SET_NEEDSTRING    (0x91)

#pragma pack(1)
typedef struct
{
    uint16_t header;
    uint16_t data_length;
    uint8_t msg_type;
    uint8_t msg_cmd;
    uint8_t msg_seq;
    uint8_t msg_data[VMUP_MSG_DATA_MAX_SIZE];
    uint16_t chk_sum;
    uint8_t tail;
} sys_msg_com_data_t;
#pragma pack()

typedef enum
{
    SYS_MSG_TYPE_ASR = 0,
    SYS_MSG_TYPE_CMD_INFO,
    SYS_MSG_TYPE_KEY,
    SYS_MSG_TYPE_COM,
    SYS_MSG_TYPE_PLAY,
    SYS_MSG_TYPE_NET,
    SYS_MSG_TYPE_MNG,
    SYS_MSG_TYPE_AUDIO_IN_STARTED,
    SYS_MSG_TYPE_I2C,
    SYS_MSG_TYPE_SED,
    SYS_MSG_TYPE_NLP,
    SYS_MSG_TYPE_TEXT,
    SYS_MSG_TYPE_SET_PALY_PARAMETER // Volume, speed, pitch
} sys_msg_type_t;

typedef struct
{
    sys_msg_type_t msg_type;
    union
    {
        sys_msg_com_data_t com_data;
    } msg_data;
} sys_msg_t;

// Callback function types
typedef std::function<void(const sys_msg_com_data_t&)> MessageCallback;
typedef std::function<void(uint8_t, const uint8_t*, uint16_t)> AsrResultCallback;
typedef std::function<void(uint8_t)> StatusNotifyCallback;

class VoiceModuleUART {
public:
    VoiceModuleUART();
    ~VoiceModuleUART();

    /**
     * @brief Initialize the voice module communication
     *
     * @param serial HardwareSerial to use for communication
     * @param rxPin RX pin to use
     * @param txPin TX pin to use
     * @param baudRate Communication baud rate (default 921600)
     * @return true if initialization was successful
     */
    bool begin(HardwareSerial& serial, int rxPin, int txPin, uint32_t baudRate = 921600);

    /**
     * @brief Process incoming data from voice module
     *
     * Should be called regularly from loop()
     */
    void update();

    /**
     * @brief Process a single incoming byte from voice module
     *
     * @param receivedByte The byte to process
     */
    void processIncomingByte(uint8_t receivedByte);

    /**
     * @brief Send a generic command to the voice module
     *
     * @param msgType Message type
     * @param msgCmd Command code
     * @param data Data to send
     * @param dataLen Length of data
     * @return true if command was sent successfully
     */
    bool sendCommand(uint8_t msgType, uint8_t msgCmd, const uint8_t* data, uint16_t dataLen);

    /**
     * @brief Play voice by ID
     *
     * @param voiceId ID of voice to play
     * @return true if command was sent successfully
     */
    bool playVoiceById(uint8_t voiceId);

    /**
     * @brief Play voice by semantic ID
     *
     * @param semanticId Semantic ID of voice to play
     * @return true if command was sent successfully
     */
    bool playVoiceBySemanticId(uint8_t semanticId);

    /**
     * @brief Control voice playback
     *
     * @param control Control command (start, pause, resume, stop)
     * @return true if command was sent successfully
     */
    bool controlPlayback(uint8_t control);

    /**
     * @brief Get module version
     *
     * @param versionType Type of version to request
     * @return true if command was sent successfully
     */
    bool getVersion(uint8_t versionType);

    /**
     * @brief Reset the voice module
     *
     * @return true if command was sent successfully
     */
    bool resetModule();

    /**
     * @brief Set module volume
     *
     * @param volume Volume level (0-100)
     * @return true if command was sent successfully
     */
    bool setVolume(uint8_t volume);

    /**
     * @brief Enter or exit wakeup mode
     *
     * @param enter true to enter wakeup mode, false to exit
     * @return true if command was sent successfully
     */
    bool setWakeupMode(bool enter);

    /**
     * @brief Set mute status
     *
     * @param mute true to mute, false to unmute
     * @return true if command was sent successfully
     */
    bool setMute(bool mute);

    /**
     * @brief Get Flash UID
     *
     * @return true if command was sent successfully
     */
    bool getFlashUID();

    /**
     * @brief Set message callback handler
     *
     * @param callback Function to call when messages are received
     */
    void setMessageCallback(MessageCallback callback);

    /**
     * @brief Set ASR result callback handler
     *
     * @param callback Function to call when ASR results are received
     */
    void setAsrResultCallback(AsrResultCallback callback);

    /**
     * @brief Set status notification callback handler
     *
     * @param callback Function to call when status notifications are received
     */
    void setStatusNotifyCallback(StatusNotifyCallback callback);

private:
    HardwareSerial* _serial;
    uint8_t _msgSeq;
    bool _isInitialized;

    // Callback handlers
    MessageCallback _messageCallback;
    AsrResultCallback _asrResultCallback;
    StatusNotifyCallback _statusCallback;

    // Receive state machine variables
    enum {
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
    };

    uint8_t _receiveState;
    uint16_t _length0, _length1;
    uint16_t _checkSum0, _checkSum1;
    uint16_t _dataRevCount;
    sys_msg_com_data_t _receivedPacket;
    uint32_t _lastReceiveTime;

    // Helper methods
    uint16_t calculateChecksum(uint16_t initVal, const uint8_t* data, uint16_t length);
    bool isPacketTimeout();
    void handleReceivedMessage();
};

#endif // VOICE_MODULE_UART_H