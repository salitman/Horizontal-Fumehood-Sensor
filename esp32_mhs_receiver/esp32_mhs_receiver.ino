// ESP32 Mounted Horizontal Fume Hood Sensor Receiver
// Desc: Receives data packets over ESPNow containing IR sensor data from sensors on sash panels

#include <esp_now.h>
#include <WiFi.h>

#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <Wire.h>

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
} struct_message;

struct_message myData;

void writeFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Writing file: %s\n", path);

    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("Failed to open file for writing");
        return;
    }
    if(file.print(message)){
        Serial.println("File written");
    } else {
        Serial.println("Write failed");
    }
    file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message){
    //Serial.printf("Appending to file: %s\n", path);

    File file = fs.open(path, FILE_APPEND);
    if(!file){
        Serial.println("Failed to open file for appending");
        return;
    }
    if(file.print(message)){
        Serial.print("Message appended: ");
        Serial.println(message);
    } else {
        Serial.println("Append failed");
    }
    file.close();
}

int readFile(fs::FS &fs, const char * path){
    Serial.printf("Reading file: %s\n", path);

    File file = fs.open(path);
    if(!file){
        Serial.println("Failed to open file for reading");
        return 0;
    }
    file.close();
    return 1;
}

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Time Since Boot (ms): ");
  Serial.println(myData.ms);
  Serial.print("Millisecond Overflows: ");
  Serial.println(myData.overflow);
  Serial.print("Distance (mm): ");
  Serial.println(myData.c);
  Serial.print("Sender ID: ");
  Serial.println(myData.id);

  strlcpy(filename, runIDString, sizeof(filename));
  sprintf(senderID, "_%00005u.txt", myData.id);
  strlcat(filename, senderID, sizeof(filename));
  
  if (!readFile(SD, filename))
    writeFile(SD, filename, "");

  sprintf(temp_lifetime, "%lu___", myData.ms);
  appendFile(SD, filename, temp_lifetime);
  Serial.println();
}
 
void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  
  if(!SD.begin()){
        Serial.println("Card Mount Failed");
        return;
    }
    uint8_t cardType = SD.cardType();
  
    if(cardType == CARD_NONE){
        Serial.println("No SD card attached");
        return;
    }

  randomSeed(analogRead(0));
  runID = random(9999);
  sprintf(runIDString, "/%0004u", runID);

  strlcpy(filename, runIDString, sizeof(filename));
  //sprintf(iteration, "_%00005u.txt", counter++);
  //strlcat(filename, iteration, sizeof(filename));
  writeFile(SD, filename, "");
  
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
