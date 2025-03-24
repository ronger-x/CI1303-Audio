#ifndef CI1303_AUDIO_SLAVE_MESSAGE_HANDLE_H
#define CI1303_AUDIO_SLAVE_MESSAGE_HANDLE_H

#include "Arduino.h"
#include "driver/uart.h" // For ESP32 UART configuration

// --- Configuration Constants ---
constexpr uint32_t CIAS_STANDARD_MAGIC = 0x5a5aa5a5;
constexpr uint16_t CIAS_SEND_MSG_BUF_LEN = 1040; // 1024 + 16
constexpr uint16_t CIAS_PCM_UPLOAD_QUEUE_SIZE = 10;

// --- Data Structures (Using packed attributes for efficient memory usage) ---

/**
 * @brief Standard message header structure
 */
typedef struct __attribute__((packed)) {
    uint32_t magic;       // Start data for frame (0x5a5aa5a5)
    uint16_t checksum;    // Checksum
    uint16_t type;        // Command type
    uint16_t len;         // Data stream length
    uint16_t version;     // Version information
    uint32_t fill_data;   // Fill data
} cias_standard_head_t;

/**
 * @brief Structure for sending messages
 */
typedef struct __attribute__((packed)) {
    uint8_t data[CIAS_SEND_MSG_BUF_LEN];
    uint16_t length;
    uint16_t type;
    uint32_t ack_flag;  // Need ACK
} cias_send_msg_t;

/**
 * @brief Fill type enumeration
 */
typedef enum {
    DEF_FILL = 0x12345678,
    SPEAK_FILL = 0x12345677,
    INVAILD_SPEAK_FILL = 0x12345666,
    MUSIC_START_FILL = 0x12345699,
    MP3_FILL = 0x12345688,
    M4A_FILL = 0x123456aa,
    WAV_FILL = 0x123456bb,
    REPEAT_FILL = 0x123456ab,
} cias_fill_type_t;

/**
 * @brief Slave receiving state enumeration
 */
typedef enum {
    MSG_FIND_HEAD = 0,
    MSG_RECV_MSG = 1,
    MSG_VERIFY = 2,
} slave_recv_state_t;

// --- Constants and Extern Variables ---

#define AUDIO_MUSIC_BUFF_COUNT (29)
#define AUDIO_MUSIC_BUFF_LEN   (AUDIO_MUSIC_BUFF_COUNT * 1024)

extern volatile bool audio_player_need_data_1L;
extern volatile bool audio_player_need_data_2L;
extern volatile bool audio_player_need_data_3L;

#define TEST_PLAYER 0 // Play raw data test

//  We'll use the ESP32's UART driver instead of HardwareSerial directly in the class

// --- Queue Handles ---

extern QueueHandle_t _messageRecvQueue;
extern QueueHandle_t _messageSendQueue;

/**
 * @brief WiFi communication command enumeration
 */
typedef enum {
    // Voice recognition related
    LOCAL_ASR_RESULT_NOTIFY             =  0x0101,  // Local voice recognition notification
    WAKE_UP                             =  0x0102,  // Wake up
    VAD_END                             =  0x0103,  // Cloud VAD END
    SKIP_INVAILD_SPEAK                  =  0x0104,  // Skip invalid voice
    PCM_MIDDLE                          =  0x0105,  // PCM data middle packet
    PCM_FINISH                          =  0x0106,  // PCM data end packet
    PCM_IDLE                            =  0x0107,  // PCM data idle

    // Network playback related
    NET_PLAY_START                      =  0x0201,  // Start playing
    NET_PLAY_PAUSE                      =  0x0202,  // Play pause
    NET_PLAY_RESUME                     =  0x0203,  // Resume playing
    NET_PLAY_STOP                       =  0x0204,  // Stop playing
    NET_PLAY_RESTART                    =  0x0205,  // Replay
    NET_PLAY_NEXT                       =  0x0206,  // Play next song
    NET_PLAY_LOCAL_TTS                  =  0x0207,  // Play local TTS
    NET_PLAY_END                        =  0x0208,  // end of play
    NET_PLAY_RECONECT_URL               =  0x0209,  // Get connection again
    PLAY_DATA_GET                       =  0x020a,  // Get subsequent playback data
    PLAY_DATA_RECV                      =  0x020b,  // Receive playback data
    PLAY_DATA_END                       =  0x020c,  // Playback data received
    PLAY_TTS_END                        =  0x020d,  // Play tts end
    PLAY_EMPTY                          =  0x020e,  // Play empty command
    PLAY_NEXT                           =  0x020f,  // After playing the previous song, actively play the next song

    // IOT custom protocol
    QCLOUD_IOT_CMD                      =  0x0301,  // Cloud IOT command
    NET_VOLUME                          =  0x0302,  // Cloud volume
    LOCAL_VOLUME                        =  0x0303,  // Local volume
    VOLUME_INC                          =  0x0304,  // Increase volume
    VOLUME_DEC                          =  0x0305,  // Decrease volume
    VOLUME_MAXI                         =  0x0306,  // Maximum volume
    VOLUME_MINI                         =  0x0307,  // Minimum volume

    // Network related
    ENTER_NET_CONFIG                    =  0x0401,  // Enter network configuration mode
    NET_CONFIGING                       =  0x0402,  // Network configuration in progress
    EXIT_NET_CONFIG                     =  0x0403,  // Exit network configuration mode
    INIT_SMARTCONFIG                    =  0x0404,  // Initial password state Factory configuration state
    WIFI_DISCONNECTED                   =  0x0405,  // The network is disconnected
    WIFI_CONNECTED                      =  0x0406,  // Network connection successful
    GET_PROFILE                         =  0x0407,  // Authentication file obtained
    NEED_PROFILE                        =  0x0408,  // Authentication file required
    CLOUD_CONNECTED                     =  0x0409,  // Cloud connected
    CLOUD_DISCONNECTED                  =  0x040a,  // Cloud disconnected
    NET_CONFIG_SUCESS                   =  0x040b,  // Network configuration successful
    NET_CONFIG_FALI                     =  0x040c,  // Network configuration failed

    // OTA and factory testing
    CIAS_OTA_START                      =  0x0501,  // Start OTA
    CIAS_OTA_DATA                       =  0x0502,  // OTA data
    CIAS_OTA_SUCESS                     =  0x0503,  // OTA upgrade successful
    CIAS_FACTORY_START                  =  0x0504,  // Production test
    CIAS_FACTORY_OK                     =  0x0505,  // Production test successful
    CIAS_FACTORY_FAIL                   =  0x0506,  // Production test failed
    CIAS_FACTORY_SELF_TEST_START        =  0x0507,  // Self test
    CIAS_IR_DATA                        =  0x0508,  // Infrared data sending
    CIAS_IR_LOADING_DATA                =  0x0509,  // Infrared code library downloading
    CIAS_IR_LOAD_DATA_OVER              =  0x050a,  // Infrared code library download completed
    CIAS_IR_LOAD_DATA_START             =  0x050b,  // Infrared download code library start
} wifi_communicate_cmd_t;

// --- Class Definition ---

class SlaveMessageHandle {
private:
    uart_port_t uart_num; // UART port number
    TaskHandle_t recvTaskHandle = nullptr;  // Handle for the receive task
    TaskHandle_t sendTaskHandle = nullptr;  // Handle for the send task
    TaskHandle_t recvDealTaskHandle = nullptr; // Handle for the receive and deal task

    // Internal helper functions (made private)
    int32_t communication_recv(uint8_t *addr, int32_t length);
    static void slaveMessageRecvTask(void* pvParameters);
    static void slaveMessageSendTask(void* pvParameters);
    static void salveMessageRecvDealTask(void *pvParameters);
    uint16_t calculateChecksum(const uint8_t *data, uint16_t length);

public:
    SlaveMessageHandle() {} //Default constructor
    SlaveMessageHandle(uart_port_t uart_num); // Parameterized constructor
    ~SlaveMessageHandle();
		int communicationTaskInit();
    int voicePortInit(uart_port_t uart_num);
    void sendMessage(uint16_t cmd, cias_fill_type_t type, const uint8_t* data, uint16_t length);
    //Added for ease of access and thread safety. In a real application,
    //you would likely have a mutex around shared resources like the audio state.
    uint16_t getAudioState();
    uint16_t setAudioState(int state);
 };

#endif //CI1303_AUDIO_SLAVE_MESSAGE_HANDLE_H