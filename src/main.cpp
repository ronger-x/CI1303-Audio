#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include "opus.h"

// WiFi 连接信息
const char* ssid = "ronger6";
const char* password = "D724b28ff.";

// WiFi 连接重试次数
const uint8_t WIFI_MAX_ATTEMPTS = 10;

// Opus 流 URL
const char* opus_stream_url = "https://static.rymcu.com/article/1744194366944.opus";

// Opus 配置
#define OPUS_SAMPLE_RATE 48000          // Opus 采样率
#define OPUS_CHANNELS 1                 // 单声道
#define OPUS_MAX_FRAME_SIZE_MS 60       // 最大帧持续时间（毫秒）
#define MAX_PCM_SAMPLES_PER_CHANNEL (OPUS_SAMPLE_RATE * OPUS_MAX_FRAME_SIZE_MS / 1000)
#define PCM_BUFFER_SIZE (MAX_PCM_SAMPLES_PER_CHANNEL * OPUS_CHANNELS)

// 缓冲区大小
#define NETWORK_BUFFER_SIZE (1024 * 20)        // 网络缓冲区大小（优化为更大块读取）
#define MAX_OPUS_PACKET_SIZE 1400       // Opus 包最大大小（考虑 Ogg 开销）

// 全局变量
OpusDecoder* opusDecoder = nullptr;
int16_t pcmOutputBuffer[PCM_BUFFER_SIZE]; // 解码后的 PCM 数据
uint8_t networkBuffer[NETWORK_BUFFER_SIZE]; // 网络读取缓冲区
uint8_t currentOpusPacket[MAX_OPUS_PACKET_SIZE]; // 当前 Opus 包
int currentOpusPacketSize = 0; // 当前 Opus 包大小
int networkBufferPos = 0; // 网络缓冲区读取位置
int networkBufferLen = 0; // 网络缓冲区有效数据长度

// Ogg 解析状态
enum OggParseState
{
    EXPECT_OGG_HEADER,
    PROCESS_PAGE_HEADER,
    PROCESS_SEGMENT_TABLE,
    PROCESS_PAGE_SEGMENTS
};

OggParseState oggState = EXPECT_OGG_HEADER;
int pageSegmentsRemaining = 0;
int currentSegmentIndex = 0;
uint8_t segmentTable[255]; // Ogg 页面段表
int segmentBytesExpected = 0;
int segmentBytesRead = 0;
bool opusHeaderProcessed = false;
bool opusTagsProcessed = false;

// 网络任务句柄
TaskHandle_t networkTaskHandle = nullptr;

// 函数声明
void network_stream_task(void* parameter);
bool parse_ogg_page();
void process_opus_packet(uint8_t* packet, int size);
void wifiConnect();
void initOpusDecoder();

// 初始化
void setup()
{
    Serial.begin(115200);
    wifiConnect();
    initOpusDecoder();
}

void loop()
{
    vTaskDelay(300 / portTICK_PERIOD_MS); // 主循环空闲
}

// WiFi 连接函数
void wifiConnect()
{
    Serial.println("Init WiFi...");
    WiFi.begin(ssid, password);
    int attempts = WIFI_MAX_ATTEMPTS;
    while (WiFi.status() != WL_CONNECTED && attempts > 0)
    {
        vTaskDelay(500 / portTICK_PERIOD_MS);
        Serial.print(".");
        attempts--;
    }
    if (WiFi.status() != WL_CONNECTED)
    {
        Serial.println("WiFi connection failed! Retrying in 5 seconds...");
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        wifiConnect(); // 重试
    }
    else
    {
        Serial.println("WiFi connected!");
    }
}

// 初始化 Opus 解码器并创建网络任务
void initOpusDecoder()
{
    int opusErr;
    opusDecoder = opus_decoder_create(OPUS_SAMPLE_RATE, OPUS_CHANNELS, &opusErr);
    if (opusDecoder == nullptr)
    {
        Serial.printf("Failed to create Opus decoder: %s\n", opus_strerror(opusErr));
        return;
    }
    Serial.println("Opus Decoder Created.");

    // 创建网络流处理任务
    Serial.println("Starting network stream task...");
    xTaskCreatePinnedToCore(
        network_stream_task, // 任务函数
        "NetworkStreamTask", // 任务名称
        32768, // 堆栈大小（优化后减少至 32KB，需测试）
        (void*)opus_stream_url, // 参数
        5, // 优先级
        &networkTaskHandle, // 任务句柄
        1 // 固定到核心 1
    );

    if (networkTaskHandle == nullptr)
    {
        Serial.println("!!! Failed to create network task!");
    }
    else
    {
        Serial.println("Network task created successfully.");
    }
}

// 网络流处理任务
void network_stream_task(void* parameter)
{
    const char* url = (const char*)parameter;
    HTTPClient http;
    WiFiClient* client = nullptr;
    long totalBytes = 0;
    long bytesRead = 0;

    Serial.printf("[TASK] Starting download & decode for: %s\n", url);
    http.begin(url);
    int httpCode = http.GET();

    if (httpCode != HTTP_CODE_OK)
    {
        Serial.printf("[TASK] HTTP GET failed: %s (Code: %d)\n", http.errorToString(httpCode).c_str(), httpCode);
        goto cleanup;
    }

    client = http.getStreamPtr();
    totalBytes = http.getSize();
    Serial.printf("[TASK] HTTP GET OK. File size: %ld bytes\n", totalBytes);

    while (client->connected() || networkBufferPos < networkBufferLen)
    {
        // 填充网络缓冲区
        if (networkBufferPos >= networkBufferLen && client->available())
        {
            networkBufferLen = client->read(networkBuffer, NETWORK_BUFFER_SIZE);
            networkBufferPos = 0;
            if (networkBufferLen > 0)
            {
                bytesRead += networkBufferLen;
                Serial.printf("[TASK] Read %d bytes (Total: %ld/%ld)\n", networkBufferLen, bytesRead, totalBytes);
            }
            else
            {
                Serial.println("[TASK] No more data to read.");
                break; // 无数据可读
            }
        }
        else if (!client->connected())
        {
            Serial.println("[TASK] Connection closed by server.");
            break;
        }
        else
        {
            // 等待更多数据，避免忙等待
            vTaskDelay(10 / portTICK_PERIOD_MS);
            continue;
        }

        // 解析 Ogg 数据

        if (!parse_ogg_page())
        {
            Serial.println("[TASK] Ogg parsing failed. Stopping.");
            break;
        }
    }

    Serial.println();
    Serial.printf("[TASK] Finished processing. Total bytes read: %ld\n", bytesRead);
    Serial.printf("[TASK] Stack high water mark: %u bytes remaining\n", uxTaskGetStackHighWaterMark(nullptr));

cleanup:
    Serial.println("[TASK] Cleaning up...");
    if (opusDecoder)
    {
        opus_decoder_destroy(opusDecoder);
        opusDecoder = nullptr;
        Serial.println("[TASK] Opus decoder destroyed.");
    }
    http.end();
    Serial.println("[TASK] HTTP connection ended.");
    networkTaskHandle = nullptr;
    vTaskDelete(nullptr);
}

// 解析 Ogg 页面
bool parse_ogg_page()
{
    if (networkBufferPos >= networkBufferLen) return true; // 数据不足

    while (networkBufferPos < networkBufferLen)
    {
        switch (oggState)
        {
        case EXPECT_OGG_HEADER:
            {
                if (networkBufferPos >= networkBufferLen || networkBufferLen - networkBufferPos < 4)
                {
                    Serial.println("[OGG] Reached end of buffer while skipping.");
                    return true; // 等待新数据
                }
                if (memcmp(networkBuffer + networkBufferPos, "OggS", 4) == 0)
                {
                    Serial.println("[OGG] Found OggS capture pattern.");
                    oggState = PROCESS_PAGE_HEADER;
                    networkBufferPos += 4;
                }
                else
                {
                    // Serial.printf("[OGG] Skipping invalid byte at pos %d: %02x\n",
                    //         networkBufferPos, networkBuffer[networkBufferPos]);
                    networkBufferPos++;
                }
                break;
            }

        case PROCESS_PAGE_HEADER:
            {
                if (networkBufferLen - networkBufferPos < 23) { return true; } // 27 - 4 ("OggS")
                pageSegmentsRemaining = networkBuffer[networkBufferPos + 22];
                Serial.printf("[OGG] Page Header. Segments: %d\n", pageSegmentsRemaining);
                networkBufferPos += 23;
                oggState = PROCESS_SEGMENT_TABLE;
                currentSegmentIndex = 0;
                break;
            }

        case PROCESS_SEGMENT_TABLE:
            {
                if (networkBufferLen - networkBufferPos < pageSegmentsRemaining) { return true; }
                Serial.printf("[OGG] Segment Table %d: ", pageSegmentsRemaining);
                for (int i = 0; i < pageSegmentsRemaining; i++)
                {
                    segmentTable[i] = networkBuffer[networkBufferPos + i];
                    Serial.printf("%d ", segmentTable[i]);
                }
                // if (pageSegmentsRemaining > 5) { Serial.print("..."); }
                Serial.println();
                networkBufferPos += pageSegmentsRemaining;
                currentSegmentIndex = 0;
                segmentBytesRead = 0;
                segmentBytesExpected = segmentTable[currentSegmentIndex];
                oggState = PROCESS_PAGE_SEGMENTS;
                break;
            }

        case PROCESS_PAGE_SEGMENTS:
            {
                if (currentSegmentIndex >= pageSegmentsRemaining)
                {
                    oggState = EXPECT_OGG_HEADER;
                    break;
                }

                int bytesNeeded = segmentBytesExpected - segmentBytesRead;
                int bytesToCopy = min(networkBufferLen - networkBufferPos, bytesNeeded);

                if (currentOpusPacketSize + bytesToCopy > MAX_OPUS_PACKET_SIZE)
                {
                    Serial.println("[ERROR] Opus packet buffer overflow!");
                    return false;
                }

                memcpy(currentOpusPacket + currentOpusPacketSize, networkBuffer + networkBufferPos, bytesToCopy);
                networkBufferPos += bytesToCopy;
                segmentBytesRead += bytesToCopy;
                currentOpusPacketSize += bytesToCopy;

                if (segmentBytesRead == segmentBytesExpected)
                {
                    bool packetCompleted = (segmentTable[currentSegmentIndex] < 255);
                    if (packetCompleted && currentOpusPacketSize > 0)
                    {
                        if (!opusHeaderProcessed)
                        {
                            if (currentOpusPacketSize >= 8 && memcmp(currentOpusPacket, "OpusHead", 8) == 0)
                            {
                                Serial.printf("[OGG] Processed OpusHead (%d bytes).\n", currentOpusPacketSize);
                                opusHeaderProcessed = true;
                            }
                            else if (currentOpusPacketSize >= 8 && memcmp(currentOpusPacket, "OpusTags", 8) == 0)
                            {
                                Serial.printf("[OGG] Processed OpusTags (%d bytes).\n", currentOpusPacketSize);
                                opusTagsProcessed = true;
                            }
                            else
                            {
                                Serial.println("[WARN] Ignoring unexpected packet before OpusHead/Tags.");
                            }
                        }
                        else if (!opusTagsProcessed)
                        {
                            if (currentOpusPacketSize >= 8 && memcmp(currentOpusPacket, "OpusTags", 8) == 0)
                            {
                                Serial.printf("[OGG] Processed OpusTags (%d bytes).\n", currentOpusPacketSize);
                                opusTagsProcessed = true;
                            }
                            else
                            {
                                Serial.println("[OGG] Assuming first audio packet.");
                                opusTagsProcessed = true;
                                process_opus_packet(currentOpusPacket, currentOpusPacketSize);
                            }
                        }
                        else
                        {
                            process_opus_packet(currentOpusPacket, currentOpusPacketSize);
                        }
                        currentOpusPacketSize = 0; // 重置包大小
                    }

                    currentSegmentIndex++;
                    if (currentSegmentIndex < pageSegmentsRemaining)
                    {
                        segmentBytesRead = 0;
                        segmentBytesExpected = segmentTable[currentSegmentIndex];
                    }
                    else
                    {
                        oggState = EXPECT_OGG_HEADER;
                    }
                }
                break;
            }

        default:
            {
                Serial.printf("[ERROR] Unknown Ogg state: %d\n", oggState);
                return false;
            }
        }
    }
    return true;
}

// 处理 Opus 包
void process_opus_packet(uint8_t* packet, int size)
{
    if (!opusDecoder || size <= 0) { return; }

    int decodedSamples = opus_decode(opusDecoder, packet, size, pcmOutputBuffer, MAX_PCM_SAMPLES_PER_CHANNEL, 0);
    if (decodedSamples < 0)
    {
        Serial.printf("[DECODE] Opus decode failed: %s. Skipping.\n", opus_strerror(decodedSamples));
        return;
    }

    size_t bytesToWrite = decodedSamples * OPUS_CHANNELS * sizeof(int16_t);
    if (bytesToWrite > 0)
    {
        Serial.printf("[I2S] Writing %d PCM bytes (%d samples)\n", bytesToWrite, decodedSamples);
        // size_t bytesWritten = Serial.write(reinterpret_cast<uint8_t*>(pcmOutputBuffer), bytesToWrite);
        // if (bytesWritten != bytesToWrite)
        // {
        //     Serial.printf("[ERROR] I2S write failed: %d/%d bytes\n", bytesWritten, bytesToWrite);
        // }
    }
}
