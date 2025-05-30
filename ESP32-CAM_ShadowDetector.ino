// Shadow Detector with Grayscale BMP Capture, Timestamp Overlay, and Time Sync
// Working version confirmed by user - May 17, 2025

#include "esp_camera.h"
#include "FS.h"
#include "SD_MMC.h"
#include "esp_now.h"
#include "WiFi.h"
#include "driver/rtc_io.h"
#include "time.h"

// Basic 6x8 font for digits and symbols
const uint8_t font6x8[][6] = {
  {0x3E,0x51,0x49,0x45,0x3E,0x00}, // 0
  {0x00,0x42,0x7F,0x40,0x00,0x00}, // 1
  {0x62,0x51,0x49,0x49,0x46,0x00}, // 2
  {0x22,0x49,0x49,0x49,0x36,0x00}, // 3
  {0x18,0x14,0x12,0x7F,0x10,0x00}, // 4
  {0x27,0x45,0x45,0x45,0x39,0x00}, // 5
  {0x3C,0x4A,0x49,0x49,0x30,0x00}, // 6
  {0x01,0x71,0x09,0x05,0x03,0x00}, // 7
  {0x36,0x49,0x49,0x49,0x36,0x00}, // 8
  {0x06,0x49,0x49,0x29,0x1E,0x00}, // 9
  {0x00,0x36,0x36,0x00,0x00,0x00}, // :
  {0x00,0x00,0x00,0x00,0x00,0x00}, // space
  {0x08,0x14,0x2A,0x41,0x00,0x00}, // A
  {0x7F,0x49,0x49,0x49,0x36,0x00}  // B
};

int charIndex(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c == ':') return 10;
  if (c == ' ') return 11;
  if (c == 'A') return 12;
  if (c == 'B') return 13;
  return -1;
}

void drawText(camera_fb_t *fb, int x, int y, const char* text) {
  int width = fb->width;
  int height = fb->height;
  for (int i = 0; text[i] != '\0'; i++) {
    int index = charIndex(text[i]);
    if (index < 0) continue;
    for (int col = 0; col < 6; col++) {
      uint8_t colData = font6x8[index][col];
      for (int row = 0; row < 8; row++) {
        if (colData & (1 << row)) {
          int px = x + i * 6 + col;
          int py = y + row;
          if (px >= 0 && px < width && py >= 0 && py < height) {
            fb->buf[py * width + px] = 255;
          }
        }
      }
    }
  }
}

#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

#define CHANNEL 1
#define THRESHOLD 5
#define WAIT_TIME 33.3

uint8_t masterMAC[] = {0x3C, 0x8A, 0x1F, 0xD3, 0x93, 0x98};
bool timeSynced = false;

typedef struct struct_message {
  char timestamp[25];
} struct_message;

struct_message incomingTime;
camera_fb_t *fb_before = NULL;
camera_fb_t *fb_after = NULL;

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) return;
  esp_now_register_recv_cb(OnDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, masterMAC, 6);
  peerInfo.channel = CHANNEL;
  peerInfo.encrypt = false;
  if (!esp_now_is_peer_exist(masterMAC)) {
    if (esp_now_add_peer(&peerInfo) != ESP_OK) return;
  }

  if (!SD_MMC.begin()) return;

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_GRAYSCALE;
  config.frame_size = FRAMESIZE_QQVGA;
  config.jpeg_quality = 12;
  config.fb_count = psramFound() ? 2 : 1;

  if (esp_camera_init(&config) != ESP_OK) return;

  while (!timeSynced) delay(100);
}

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len == sizeof(incomingTime)) {
    memcpy(&incomingTime, data, sizeof(incomingTime));
    struct tm tm;
    if (strptime(incomingTime.timestamp, "%Y-%m-%d %H:%M:%S", &tm)) {
      time_t t = mktime(&tm);
      struct timeval now = { .tv_sec = t };
      settimeofday(&now, NULL);
      timeSynced = true;
    }
  }
}

int calculateDifference(camera_fb_t *a, camera_fb_t *b) {
  if (a->len != b->len) return 10000;
  int diff = 0;
  for (size_t i = 0; i < a->len; i++) {
    diff += abs(a->buf[i] - b->buf[i]);
  }
  return diff / a->len;
}

String getTimestampFilename(const char* prefix) {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return "/invalid.bmp";
  char timestamp[32];
  strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", &timeinfo);
  return String("/") + prefix + "_" + timestamp + ".bmp";
}

bool saveAsBMP(camera_fb_t *fb, const String &filename, const char* label) {
  drawText(fb, 2, 2, label);
  File file = SD_MMC.open(filename, FILE_WRITE);
  if (!file) return false;

  uint32_t width = fb->width;
  uint32_t height = fb->height;
  uint32_t rowSize = (width + 3) & ~3;
  uint32_t imageSize = rowSize * height;
  uint32_t fileSize = 54 + 1024 + imageSize;

  uint8_t bmpHeader[54] = {0};
  bmpHeader[0] = 'B';
  bmpHeader[1] = 'M';
  bmpHeader[2] = fileSize;
  bmpHeader[3] = fileSize >> 8;
  bmpHeader[4] = fileSize >> 16;
  bmpHeader[5] = fileSize >> 24;
  uint32_t offset = 54 + 1024;
  bmpHeader[10] = offset;
  bmpHeader[11] = offset >> 8;
  bmpHeader[12] = offset >> 16;
  bmpHeader[13] = offset >> 24;
  bmpHeader[14] = 40;
  bmpHeader[18] = width;
  bmpHeader[19] = width >> 8;
  bmpHeader[22] = height;
  bmpHeader[23] = height >> 8;
  bmpHeader[26] = 1;
  bmpHeader[28] = 8;
  uint32_t colorsUsed = 256;
  bmpHeader[46] = colorsUsed;
  bmpHeader[47] = colorsUsed >> 8;
  bmpHeader[48] = colorsUsed >> 16;
  bmpHeader[49] = colorsUsed >> 24;

  file.write(bmpHeader, 54);
  for (int i = 0; i < 256; i++) {
    file.write((uint8_t)i);
    file.write((uint8_t)i);
    file.write((uint8_t)i);
    file.write((uint8_t)0);
  }
  for (int y = height - 1; y >= 0; y--) {
    file.write(&fb->buf[y * width], width);
    for (int p = 0; p < (rowSize - width); p++) file.write((uint8_t)0);
  }
  file.close();
  return true;
}

void loop() {
  fb_before = esp_camera_fb_get();
  delay(WAIT_TIME);
  fb_after = esp_camera_fb_get();

  int difference = calculateDifference(fb_before, fb_after);
  Serial.printf("Frame difference: %d\n", difference);

  if (difference > THRESHOLD) {
    Serial.println("Shadow Detected!");
    struct tm timeinfo;
    getLocalTime(&timeinfo);
    char label[16];
    sprintf(label, "B_%02d%02d", timeinfo.tm_hour, timeinfo.tm_min);
    String beforePath = getTimestampFilename("before");
    saveAsBMP(fb_before, beforePath, label);
    sprintf(label, "A_%02d%02d", timeinfo.tm_hour, timeinfo.tm_min);
    String afterPath = getTimestampFilename("after");
    saveAsBMP(fb_after, afterPath, label);
  }

  esp_camera_fb_return(fb_before);
  esp_camera_fb_return(fb_after);
}
