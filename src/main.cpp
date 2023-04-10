#include <Arduino.h>
#include "Adafruit_VL53L0X.h"
#include <Wire.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <EEPROM.h>

#define BOARD_ID 1

#define I2C_SDA 7
#define I2C_SCL 8

#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31

// set the pins to shutdown
#define SHT_LOX1 4
#define SHT_LOX2 5

// DC:54:75:50:E3:E4: DFBot C3
// 40:22:D8:4F:03:C8 motu esp
// A0:76:4E:1A:92:98 esp32 stamp c3
// E0:5A:1B:75:DF:70

// objects for the vl53l0x
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();

// this holds the measurement
VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;

uint8_t broadcastAddress[] = {0xE0, 0x5A, 0x1B, 0x75, 0xDF, 0x70};
// uint8_t broadcastAddress1[] = {0x40, 0x22, 0xD8, 0x4F, 0x03, 0xC8};

constexpr char WIFI_SSID[] = "parth";
int pin = 4, count = 0;
unsigned long duration, saved_distance_1{8190}, saved_distance_2{8190};
uint16_t distance = 0;
bool decreased = false, increased = true;
String esid;
bool direction_in{false}, phase_one_for_in{false}, phase_two_for_in{}, phase_one_for_out{}, phase_two_for_out{};

typedef struct struct_message
{
  int distance_1_mm;
  int distance_2_mm;
  int direction;
} struct_message;
struct_message myData;

typedef struct ssid_send
{
  String ssid;
} ssid_send;
ssid_send SsidRecieved;

void read_dual_sensors();
void setID();

int32_t getWiFiChannel(const char *ssid)
{
  if (int32_t n = WiFi.scanNetworks())
  {
    for (uint8_t i = 0; i < n; i++)
    {
      if (!strcmp(ssid, WiFi.SSID(i).c_str()))
      {
        return WiFi.channel(i);
      }
    }
  }
  return 0;
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{

}

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len)
{
  // Copies the sender mac address to a string
  char macStr[18];
  
  memcpy(&SsidRecieved, incomingData, sizeof(SsidRecieved));

  Serial.println("This is ssid");
  String qsid = SsidRecieved.ssid;

  if (qsid.length() > 0)
  {
    
    for (int i = 0; i < 96; ++i)
    {
      EEPROM.write(i, 0);
    }
    Serial.println(qsid);
    Serial.println("");
    Serial.println("writing eeprom ssid:");
    for (int i = 0; i < qsid.length(); ++i)
    {
      EEPROM.write(i, qsid[i]);
      Serial.print("Wrote: ");
      Serial.println(qsid[i]);
    }
    EEPROM.commit();
    ESP.restart();
  }
}

void setup()
{

  Serial.begin(115200);
  while (!Serial)
  {
    delay(1);
  }

  EEPROM.begin(512);
  delay(10);

  
  for (int i = 0; i < 32; ++i)
  {
    esid += char(EEPROM.read(i));
  }
  Serial.println();
  Serial.print("SSID: ");
  Serial.println(esid);

  Wire.setPins(I2C_SDA, I2C_SCL);
  Wire.begin();
  WiFi.mode(WIFI_STA);

  int32_t channel = getWiFiChannel(WIFI_SSID);

  WiFi.printDiag(Serial); // Uncomment to verify channel number before
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  WiFi.printDiag(Serial); // Uncomment to verify channel change after

  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);

  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add peer");
    return;
  }

  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);

  Serial.println(F("Shutdown pins inited..."));

  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);

  Serial.println(F("Both in reset mode...(pins are low)"));

  Serial.println(F("Starting..."));
  setID();
  delay(500);
}

long t1 = 0;
long t2 = 0;

void loop()
{
  lox1.rangingTest(&measure1, false); 
  lox2.rangingTest(&measure2, false); // pass in 'true' to get debug data printout!



  myData.distance_1_mm = measure1.RangeMilliMeter;
  myData.distance_2_mm = measure2.RangeMilliMeter;
  if (myData.distance_1_mm < 2000)
  {
    t1 = millis();
  }
  else if (myData.distance_2_mm < 2000)
  {
    t2 = millis();
  }
  if (t1 > 0 && t2 > 0)
  {
    if (t1 < t2)
    {

      phase_one_for_in = true;
      phase_one_for_out = false;
    }
    else if (t2 < t1)
    {

      phase_one_for_out = true;
      phase_one_for_in = false;
    }
    else
    {
      Serial.println("");
    }

    t1 = 0;
    t2 = 0;
  }
  if (myData.distance_1_mm > myData.distance_2_mm && phase_one_for_in && !phase_one_for_out)
  {
    Serial.println("in");
    phase_one_for_in = false;
    phase_one_for_out = false;
    myData.direction = 1;
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
    t1 = 0;
    t2 = 0;
  }
  if (myData.distance_1_mm < myData.distance_2_mm && phase_one_for_out && !phase_one_for_in)
  {
    Serial.println("out");
    phase_one_for_out = false;
    phase_one_for_in = false;
    phase_one_for_out = false;
    myData.direction = 0;
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
    t1 = 0;
    t2 = 0;
  }

  // if((saved_distance_1 - myData.distance_1_mm) > 1000 && (saved_distance_2 - myData.distance_2_mm) < 500){
  //   if (phase_one_for_out == true && t1 < t2){
  //     direction_in = false;
  //     phase_one_for_out = false;
  //     Serial.println("Direction out");
  //     t1=0;
  //     t2=0;
  //   }

  //   phase_one_for_in = true;
  //   saved_distance_1 = myData.distance_1_mm;
  //   saved_distance_2 = myData.distance_2_mm;
  // }
  // if (saved_distance_2 - myData.distance_2_mm > 100  && saved_distance_1 - myData.distance_1_mm < 500){
  //   if (phase_one_for_in == true){
  //     phase_two_for_in = true;
  //     phase_one_for_in = false;

  //   }

  //   if (phase_one_for_out == true){
  //     phase_two_for_out = true;
  //     phase_one_for_out = false;

  //   }
  // }
  // if (saved_distance_2 - myData.distance_2_mm > 4000  && saved_distance_1 - myData.distance_1_mm < 100){
  //   if (phase_one_for_in == true && t1 > t2){
  //     direction_in = true;
  //     phase_one_for_in = false;
  //     Serial.println("Direction in");
  //     t1=0;
  //     t2=0;
  //   }

  //   phase_one_for_out = true;

  //   saved_distance_1 = myData.distance_1_mm;
  //   saved_distance_2 = myData.distance_2_mm;
  // }

  // esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
  // if (result == ESP_OK)
  // {
  //   Serial.println("Sent with success");
  // }
  // else
  // {
  //   Serial.println("Error sending the data");
  // }
  // Serial.println();

  // delay(1000);
}

void setID()
{
  // all reset
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  delay(10);
  // all unreset
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  // activating LOX1 and resetting LOX2
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, LOW);

  // initing LOX1
  if (!lox1.begin(LOX1_ADDRESS))
  {
    Serial.println(F("Failed to boot first VL53L0X"));
    while (1)
      ;
  }
  delay(10);

  // activating LOX2
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  // initing LOX2
  if (!lox2.begin(LOX2_ADDRESS))
  {
    Serial.println(F("Failed to boot second VL53L0X"));
    while (1)
      ;
  }
}
