// ESP32-C3 Mounted Horizontal Fume Hood Sensor Sender
// Desc: Measures distance through transparent sash and relays data to receiver over ESPNow whenever panel is moved (sleeps otherwise to preserve battery)

#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include "SparkFun_VL53L1X.h"
#include <SPI.h>
#include <esp_sleep.h>

#define ACCELEROMETER_FREQ 50 // Frequency of measurements (Hz.)
#define ACCELEROMETER_THRESHOLD 0.3 // Force to trigger interrupt (g)
#define ACCELEROMETER_DURATION 0.01 // Length of interrupt (s)
#define SPI_MODE SPI_MODE3 // Use mode 3 for LIS2DE12TR
#define SPI_FREQ 10000000 
#define INTERRUPT_PIN 0
#define SHUTDOWN_PIN 10
#define TIME_UNTIL_SLEEP 5 // Seconds to stay awake once interrupt goes low

SFEVL53L1X distanceSensor;

// Receiver MAC address
uint8_t broadcastAddress[] = {0xC8, 0xF0, 0x9E, 0xA2, 0xF5, 0x2C};
int seconds_passed = 0;
unsigned long last_cleared = 0;
int times_cleared = 0;

// Structure example to send data
typedef struct struct_message {
  unsigned long ms;
  int overflow;
  float c;
  int id;
  int N_packet;
  int error;
} struct_message;

struct_message myData;

esp_now_peer_info_t peerInfo;

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void sendData(int error = 0) {
  static unsigned long now = millis();
  static int N_packet = 1;
  static int overflow = 0;
  if (millis() < now)
    overflow += 1;

  distanceSensor.startRanging();
  while (!distanceSensor.checkForDataReady())
  {
    delay(1);
  }
  int distance = distanceSensor.getDistance();
  distanceSensor.clearInterrupt();
  distanceSensor.stopRanging();
    
  myData.ms = millis();
  myData.overflow = overflow;
  myData.c = (float)distance;
  myData.id = 11;
  myData.N_packet = N_packet++;
  myData.error = error;
  
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }

  now = millis();
}

void goToSleep() {
  esp_sleep_enable_gpio_wakeup();
  esp_deep_sleep_enable_gpio_wakeup(1ULL << INTERRUPT_PIN, ESP_GPIO_WAKEUP_GPIO_HIGH);
  Serial.println("Sleeping...");
  LIS2DE12TR_Int1_Clear();
  delay(1000);
  esp_deep_sleep_start();
}
 
void setup() {
  Serial.begin(115200);

  // ********** ESP NOW **********
  if (esp_sleep_get_wakeup_cause() != ESP_SLEEP_WAKEUP_UNDEFINED) {
    Serial.println(); Serial.println("Waking from sleep!");
    Serial.print("Reason: "); Serial.println(esp_sleep_get_wakeup_cause());
  }
  else {
    Serial.println(); Serial.println("Device Booting");
    Serial.println("Seth Litman 2025");
  }
  
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  //esp_now_register_send_cb(onDataSent); // Seems to interfere with sleeping
  
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;       
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  // ********** DISTANCE SENSOR **********
  Wire.begin();
  pinMode(SHUTDOWN_PIN, OUTPUT);
  digitalWrite(SHUTDOWN_PIN, HIGH);

  if (distanceSensor.begin() != 0) {
    Serial.println("Distance sensor failed to initialize");
    while(1);
  }

  // ********** ACCELEROMETER **********
  pinMode(SS, OUTPUT);
  pinMode(INTERRUPT_PIN, INPUT_PULLUP);

  SPI.begin();

  LIS2DE12TR_StartupCheck();

  LIS2DE12TR_Initialize(ACCELEROMETER_FREQ);
  LIS2DE12TR_Int1_Setup(ACCELEROMETER_THRESHOLD, ACCELEROMETER_DURATION);

  // ********** FIRST MEASUREMENT **********
  sendData();
  LIS2DE12TR_Int1_Clear();
}
 
void loop() {

  while(!digitalRead(INTERRUPT_PIN)) {
    times_cleared = 0;
    delay(1000);
    seconds_passed++;
    Serial.println(seconds_passed);
    
    if (seconds_passed >= TIME_UNTIL_SLEEP) {
      sendData();
      delay(2000);
      goToSleep();
    }
  }
  seconds_passed = 0;

  // ********** ERROR HANDLING **********
  if (millis() >= last_cleared + 1000) { // clear interrupt reg in case of getting stuck
     LIS2DE12TR_Int1_Clear();
     last_cleared = millis();
     Serial.println(digitalRead(INTERRUPT_PIN));
     if (times_cleared++ >= 1 && !LIS2DE12TR_Int1_Clear()) {
        Serial.println("Error: Accelerometer stuck with zero data");
        sendData(1);
        while(1);
     }
     if (times_cleared++ >= 10 && LIS2DE12TR_Int1_Clear() >= 106) {
        Serial.println("Error: Accelerometer stuck with all high interrupts");
        sendData(2);
        while(1);
     }
  }
 
}
