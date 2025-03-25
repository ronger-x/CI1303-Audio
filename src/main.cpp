#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "ci1303/CommunicationPortForAudio.h"
#include "ci1303/SlaveMessageHandle.h"

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

// Create voice module instance
SlaveMessageHandle slaveMessageHandle;

void wifiConnect();

void setup()
{
    // 初始化 ESP32 日志串口
    Serial.begin(115200);
    // 初始化 WiFi 连接
    wifiConnect();
    // 初始化通信端口
    if (voiceModulePort.communicationPortInit(UART_NUM_1) != VOICE_OK) {
        Serial.println("Failed to initialize voice port");
        return;
    }
    // 初始化SlaveMessageHandle的任务
    if (slaveMessageHandle.messageHandlerInit() != VOICE_OK) {
        Serial.println("Failed to initialize slave message tasks");
        return;
    }
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
