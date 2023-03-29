#include <Arduino.h>
// DC:54:75:50:E3:E4: DFBot C3
// 40:22:D8:4F:03:C8 motu esp
// A0:76:4E:1A:92:98 esp32 stamp c3
// E0:5A:1B:75:DF:70
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>

uint8_t broadcastAddress[] = { 0xE0, 0x5A, 0x1B, 0x75, 0xDF, 0x70 };
// uint8_t broadcastAddress1[] = {0x40, 0x22, 0xD8, 0x4F, 0x03, 0xC8};
#define BOARD_ID 1
int pin =4, count=0;
unsigned long duration, prev_distance{0}, saved_distance{0}, distance_2;
uint16_t distance=0;
bool decreased=false, increased=true;

typedef struct struct_message {
  int id;
  float temp;
  float hum;
  int readingId;
} struct_message;
struct_message myData;

// Insert your SSID
constexpr char WIFI_SSID[] = "parth";
int32_t getWiFiChannel(const char *ssid) {
  if (int32_t n = WiFi.scanNetworks()) {
    for (uint8_t i = 0; i < n; i++) {
      if (!strcmp(ssid, WiFi.SSID(i).c_str())) {
        return WiFi.channel(i);
      }
    }
  }
  return 0;
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}


void setup() {
 
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  int32_t channel = getWiFiChannel(WIFI_SSID);

  WiFi.printDiag(Serial);  // Uncomment to verify channel number before
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  WiFi.printDiag(Serial);  // Uncomment to verify channel change after


  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);

  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  pinMode(pin, INPUT);
   delay(500); 

}
 
void loop() {
  
  duration = pulseIn(pin, HIGH);
  distance=duration/100;

  distance_2 = 0;
  if ((saved_distance - distance > 500) && decreased==false){
    decreased = true;
    saved_distance = distance;
  }
  if((distance - saved_distance) > 500 && decreased==true){
    

    myData.id = BOARD_ID;
    myData.temp = 10;
    myData.hum = 10;
    
    
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
    if (result == ESP_OK) {
      Serial.println("Sent with success");
    } else {
      Serial.println("Error sending the data");
    }
      decreased = false;
      saved_distance = distance;
  }


}