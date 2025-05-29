#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include "RTClib.h"
#include <time.h>

// ====== Data Struct Expected by Camera ======
typedef struct struct_message {
  char timestamp[25];  // Format: "YYYY-MM-DD HH:MM:SS"
} struct_message;

RTC_DS3231 rtc;
struct_message timeData;

#define LED_PIN 2  // GPIO 2 for LED

// ====== Flash LED on Success ======
void flashSyncLED(int times = 3) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }
}

// ====== ESP-NOW Send Callback ======
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Send Status: ");
  if (status == ESP_NOW_SEND_SUCCESS) {
    Serial.println("✅ Success");
    flashSyncLED();  // Only flash if send succeeded
  } else {
    Serial.println("❌ Fail");
  }
}

// ====== Send Time in Formatted Timestamp ======
void sendCurrentTime() {
  DateTime now = rtc.now();

  struct tm tm_info;
  tm_info.tm_year = now.year() - 1900;
  tm_info.tm_mon  = now.month() - 1;
  tm_info.tm_mday = now.day();
  tm_info.tm_hour = now.hour();
  tm_info.tm_min  = now.minute();
  tm_info.tm_sec  = now.second();

  strftime(timeData.timestamp, sizeof(timeData.timestamp), "%Y-%m-%d %H:%M:%S", &tm_info);

  uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&timeData, sizeof(timeData));

  if (result == ESP_OK) {
    Serial.print("⏱️ Sent time: ");
    Serial.println(timeData.timestamp);
  } else {
    Serial.print("❌ Error sending time: ");
    Serial.println(result);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Init I2C and RTC
  Wire.begin();
  if (!rtc.begin()) {
    Serial.println("❌ RTC not found!");
    while (true);
  }

  if (rtc.lostPower()) {
    Serial.println("⚠️ RTC lost power. Setting to compile time.");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  // Set WiFi to STA mode
  WiFi.mode(WIFI_STA);
  delay(200);

  // Re-init ESP-NOW cleanly
  esp_now_deinit();
  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ ESP-NOW Init Failed");
    return;
  }
  Serial.println("✅ ESP-NOW Initialized");

  esp_now_register_send_cb(OnDataSent);

  // Add broadcast peer
  esp_now_peer_info_t peerInfo = {};
  uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("❌ Failed to add peer");
    return;
  } else {
    Serial.println("✅ Broadcast peer added");
  }

  delay(500);  // Wait before sending

  sendCurrentTime();  // ⏱️ Send formatted time string
}

void loop() {
  // Idle
}
