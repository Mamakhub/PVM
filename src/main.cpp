/*
This is the code for PVM
In this first prototype, PVM connects to the GPS module and send location data to another PVM over LoRa.
Functionality should include:
- GPS data parsing
- LoRa transmission (Receive + Send)
*/
#include <Arduino.h>
#include <LoRa.h>
#include <TinyGPS++.h>
#include "esp_system.h"
#include "LoRaPacket.h"

// GPS
#define GPS_BAUD 9600
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17
// LoRa
#define LORA_FREQ 433E6
// Button
#define BUTTON_PIN 26

HardwareSerial gpsSerial(2);
TinyGPSPlus gps;

// variables
uint16_t DEVICE_ID;
bool isButtonPressed = false;
unsigned long gpsTimer = 0;

// function declaration
void initGPS();
void initLoRa();
void initButton();
uint16_t setDeviceID();
bool everyNSeconds(unsigned long &lastTime, float seconds);
void displayGPSInfo();
bool checkButtonPress();

void setup()
{
  Serial.begin(115200);
  initGPS();
  initLoRa();
  initButton();
  DEVICE_ID = setDeviceID();
}

void loop()
{
  // Receive and Forward Packets
  int packetCondition = LoRa.parsePacket();
  if (packetCondition == sizeof(LoRaPacket))
  {
    // debugging
    // Serial.println(String(packetCondition) + " before packet receive");
    LoRaPacket packet = LoRaPacket::receivePacket();
    if (packet.verifyCRC(packet))
    {
      if (packet.deviceID != DEVICE_ID)
      {
        packet.forwardPacket(packet, DEVICE_ID);
        packet.updateNeighborList(packet);
        packet.printNeighborList();
      }
    }
  }

  // Send GPS Packet every 10 seconds
  if (everyNSeconds(gpsTimer, 10))
  {
    LoRaPacket gpsPacket = LoRaPacket::createPacket(DEVICE_ID, 0x01, 2, "GPS Data");
    gpsPacket.sendPacket(gpsPacket);
  }

  // Press Button and Send SOS Packet
  if (checkButtonPress())
  {
    LoRaPacket sosPacket = LoRaPacket::createPacket(DEVICE_ID, 0x02, 1, "SOS");
    sosPacket.sendPacket(sosPacket);
  }
}

// Functions
void initGPS()
{
  unsigned long start = millis();
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  while (millis() - start < 3000)
  {
    if (gpsSerial.available())
    {
      Serial.println("GPS Initialized");
      break;
    }
  }
}

void initLoRa()
{
  unsigned long start = millis();
  LoRa.setPins(5, 14, 2);
  while (!LoRa.begin(LORA_FREQ))
  {
    if (millis() - start > 5000)
    {
      Serial.println("LoRa init failed, restarting...");
      ESP.restart();
    }
    delay(500);
  }
  LoRa.setSyncWord(0xA5);
  Serial.println("LoRa Initializing OK!");
}

void initButton()
{
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  Serial.println("Button Initialized");
}

uint16_t setDeviceID()
{
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  return (uint16_t)(mac[4] << 8 | mac[5]);

  Serial.println("Device ID: " + String(DEVICE_ID));
}

void displayGPSInfo()
{
  Serial.println("Latitude: " + String(gps.location.lat(), 6));
  Serial.println("Longitude: " + String(gps.location.lng(), 6));
  Serial.println("Altitude: " + String(gps.altitude.meters()));

  if (gps.date.isValid())
  {
    Serial.println("Date: " + String(gps.date.month()) + "/" + String(gps.date.day()) + "/" + String(gps.date.year()));
  }
  else
  {
    Serial.println("Date: Not Available");
  }

  if (gps.time.isValid())
  {
    Serial.println("Time: " + String(gps.time.hour()) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second()));
  }
  else
  {
    Serial.println("Time: Not Available");
  }
}

bool everyNSeconds(unsigned long &lastTime, float seconds)
{
  unsigned long interval = (unsigned long)(seconds * 1000);
  if (millis() - lastTime >= interval)
  {
    lastTime = millis();
    return true;
  }
  return false;
}

bool checkButtonPress()
{
  bool pressed = digitalRead(BUTTON_PIN) == LOW;

  // Check for button press
  if (pressed && !isButtonPressed)
  {
    isButtonPressed = true;
    return true;
  }
  // Release button
  else if (!pressed)
  {
    isButtonPressed = false;
  }
  // Button is not pressed
  return false;
}