// ESP32 Mounted Horizontal Fume Hood Sensor Receiver
// Desc: Receives data packets over ESPNow containing IR sensor data from sensors on sash panels

#include <esp_now.h>
#include <WiFi.h>

int runID = 0;
char runIDString[5];
unsigned long last_time = 0;
char temp_lifetime[20];
int counter = 0;
char senderID[10];
char filename[16];

typedef struct struct_message {
  unsigned long ms; // milliseconds since sender was powered on
  int overflow; // overflow of previous variable (for long periods of activity)
  float c; // millimeter measurement from IR sensor 
  int id; // identifies which sender sent packet
  int N_packet; // counts each packet sent after wake-up
  int error; // error code from sender, normally 0
} struct_message;
struct_message myData;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  
  //Serial.print("Bytes received: ");
  Serial.println(len);
  //Serial.print("Time Since Boot (ms): ");
  Serial.println(myData.ms);
  //Serial.print("Millisecond Overflows: ");
  Serial.println(myData.overflow);
  //Serial.print("Distance (mm): ");
  Serial.println(myData.c);
  //Serial.print("Sender ID: ");
  Serial.println(myData.id);
  //Serial.print("Packet Number: ");
  Serial.println(myData.N_packet);
  //Serial.print("Error Code: ");
  Serial.println(myData.error);
}
 
void setup() {
  Serial.begin(115200);
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  esp_now_register_recv_cb(OnDataRecv);
}
 
void loop() {

}
