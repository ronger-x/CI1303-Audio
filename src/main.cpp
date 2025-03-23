#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <VoiceModuleUART.h>

// WiFi 连接信息
const char* ssid = "ronger6";
const char* password = "D724b28ff.";

// WiFi 连接次数
uint8_t connectCount = 10;

/************************* 串口变量声明 ****************************/
#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)
#define EX_UART_NUM 1 // set uart1
#define PATTERN_CHR_NUM (3)

#define UART_PIN_TX GPIO_NUM_9 // 串口发送引脚GPIO_9
#define UART_PIN_RX GPIO_NUM_10 // 串口接收引脚GPIO_10
#define UART_BAUD_RATE 921600 // 串口接收引脚GPIO_10

// Create voice module instance
VoiceModuleUART voiceModule;
// Variables for demo
uint32_t lastActionTime = 0;
uint8_t voiceModuleState = 0;

void wifiConnect();

void voiceModuleInit();

void onMessageReceived(const sys_msg_com_data_t& msg);

void onAsrResult(uint8_t seq, const uint8_t* data, uint16_t dataLen);

void onStatusNotify(uint8_t status);

void setup()
{
    // 初始化 ESP32 日志串口
    Serial.begin(115200);
    // 初始化 WiFi 连接
    wifiConnect();
    // 初始化语音模块
    voiceModuleInit();
}

void loop()
{
    // vTaskDelay(300);
    // 豫章故郡，洪都新府。星分翼轸，地接衡庐。
    // 襟三江而带五湖，控蛮荆而引瓯越。物华天宝，龙光射牛斗之墟；
    // 人杰地灵，徐孺下陈蕃之榻。雄州雾列，俊采星驰。
    // 台隍枕夷夏之交 ，宾主尽东南之美。都督阎公之雅望，棨戟遥临；
    // 宇文新州之懿范，襜帷暂驻。十旬休假，胜友如云；
    // 千里逢迎，高朋满座。腾蛟起凤，孟学士之词宗；
    // 紫电青霜，王将军之武库。家君作宰，路出名区；童子何知，躬逢胜饯
    // test_http("https://static.rymcu.com/article/1736769289579.mp3"); // 16kbps 16kHz 1bit
    // test_http("https://static.rymcu.com/article/1736865358460.wav"); // 16kbps 16kHz 1bit
    // vTaskDelay(60000 * 5);

    // Process incoming messages from voice module
    voiceModule.update();

    // Simple demo sequence
    uint32_t currentTime = millis();
    if (currentTime - lastActionTime > 5000)
    {
        // Every 5 seconds
        lastActionTime = currentTime;

        // Cycle through different commands
        switch (voiceModuleState)
        {
        case 0:
            {
                Serial.println("Getting module version...");
                voiceModule.getVersion(VMUP_MSG_DATA_VER_APP);
                break;
            }

        case 1:
            {
                Serial.println("Playing voice ID 1...");
                voiceModule.playVoiceById(1);
                break;
            }

        case 2:
            {
                Serial.println("Stopping playback...");
                voiceModule.controlPlayback(VMUP_MSG_DATA_PLAY_STOP);
                break;
            }

        case 3:
            {
                Serial.println("Setting wakeup mode: ON");
                voiceModule.setWakeupMode(true);
                break;
            }

        case 4:
            {
                Serial.println("Setting wakeup mode: OFF");
                voiceModule.setWakeupMode(false);
                break;
            }
        default:
            {
                break;
            }
        }

        // Cycle through states
        voiceModuleState = (voiceModuleState + 1) % 5;
    }
}

void wifiConnect()
{
    Serial.println("Init WiFi...");
    WiFi.begin(ssid, password);
    while (WiFiClass::status() != WL_CONNECTED && connectCount > 0)
    {
        delay(500);
        Serial.print(".");
        connectCount--;
    }
    if (WiFiClass::status() != WL_CONNECTED)
    {
        Serial.println("WiFi connection failed!");
        return;
    }
    Serial.println("WiFi connected!");
}

void voiceModuleInit()
{
    Serial.println("Init voice module...");
    // Initialize voice module with Serial1
    if (!voiceModule.begin(Serial1, UART_PIN_RX, UART_PIN_TX, UART_BAUD_RATE))
    {
        Serial.println("Voice module initialization failed!");
        return;
    }

    // Set callback functions
    voiceModule.setMessageCallback(onMessageReceived);
    voiceModule.setAsrResultCallback(onAsrResult);
    voiceModule.setStatusNotifyCallback(onStatusNotify);

    // Initial configuration
    voiceModule.setVolume(80); // Set volume to 80%

    Serial.println("Voice Module initialized successfully");
    lastActionTime = millis();
}

// Callback for all received messages
void onMessageReceived(const sys_msg_com_data_t& msg)
{
    Serial.print("Message received - Type: 0x");
    Serial.print(msg.msg_type, HEX);
    Serial.print(", Command: 0x");
    Serial.print(msg.msg_cmd, HEX);
    Serial.print(", Sequence: ");
    Serial.print(msg.msg_seq);
    Serial.print(", Data Length: ");
    Serial.println(msg.data_length);

    // Print data bytes if any
    if (msg.data_length > 0)
    {
        Serial.print("Data: ");
        for (int i = 0; i < msg.data_length; i++)
        {
            Serial.print("0x");
            Serial.print(msg.msg_data[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
    }
}

// Callback specifically for ASR results
void onAsrResult(uint8_t seq, const uint8_t* data, uint16_t dataLen)
{
    Serial.println("ASR Result received:");

    if (dataLen > 0)
    {
        Serial.print("Command ID: ");
        Serial.println(data[0]);

        // If additional data is available (like confidence score)
        if (dataLen > 1)
        {
            Serial.print("Additional data: ");
            for (int i = 1; i < dataLen; i++)
            {
                Serial.print(data[i]);
                Serial.print(" ");
            }
            Serial.println();
        }
    }
}

// Callback for status notifications
void onStatusNotify(uint8_t status)
{
    Serial.print("Status notification: ");

    switch (status)
    {
    case VMUP_MSG_DATA_NOTIFY_POWERON:
        Serial.println("Power On");
        break;

    case VMUP_MSG_DATA_NOTIFY_WAKEUPENTER:
        Serial.println("Wakeup Entered");
        break;

    case VMUP_MSG_DATA_NOTIFY_WAKEUPEXIT:
        Serial.println("Wakeup Exited");
        break;

    case VMUP_MSG_DATA_NOTIFY_PLAYSTART:
        Serial.println("Playback Started");
        break;

    case VMUP_MSG_DATA_NOTIFY_PLAYEND:
        Serial.println("Playback Ended");
        break;

    default:
        Serial.print("Unknown (0x");
        Serial.print(status, HEX);
        Serial.println(")");
        break;
    }
}

uint16_t msg_seq = 1;

void test_http(const char* url)
{
    // 开始下载音频文件
    HTTPClient audioHttp;
    audioHttp.begin(url);
    int audioHttpCode = audioHttp.GET();
    if (audioHttpCode > 0)
    {
        Serial.println("Audio file downloaded successfully");
        if (audioHttpCode == HTTP_CODE_OK)
        {
            // Get the size of the audio file (optional, for logging)
            int fileSize = audioHttp.getSize();
            Serial.printf("Audio file size: %d bytes\n", fileSize);

            // Get the data stream
            WiFiClient* stream = audioHttp.getStreamPtr();

            // Buffer to hold the data chunks
            uint8_t buffer[RD_BUF_SIZE];

            // Read and send the audio data in chunks
            while (audioHttp.connected() && stream->available())
            {
                // Read a chunk of data
                size_t len = stream->readBytes(buffer, RD_BUF_SIZE);

                // Send the chunk to the voice chip via serial
                // Serial1.write(buffer, len);

                // Log the progress (optional)
                Serial.printf("Sent %d bytes to voice chip\n", len);
            }
        }
        Serial.println("Audio download and transmission complete.");
    }
    else
    {
        Serial.println("Audio file download failed");
    }
    audioHttp.end();
}
