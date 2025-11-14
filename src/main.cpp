/*
This is the code for PVM
PVM connects to the GPS module and send location data over LoRa.
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
#define BUTTON_PIN 15
// SOS Led
#define SOS_LED_PIN 4
// Buzzer
#define BUZZER_PIN 21

HardwareSerial gpsSerial(2);
TinyGPSPlus gps;

// variables
uint16_t DEVICE_ID;
bool isSOStriggered = false;
bool lastButtonState = HIGH;
unsigned long gpsTimer = 0;
unsigned long sosTimer = 0;
String gpsCoordinates = "";
String gpsTimestamp = "";

// function declaration
void initGPS();
void initLoRa();
void initButton();
uint16_t setDeviceID();
bool everyNSeconds(unsigned long &lastTime, float seconds);
void displayGPSInfo();
bool checkButtonPress();
void playSOSMorseCode(); // SOS Morse code function

void setup()
{
  Serial.begin(115200);
  initGPS();
  initLoRa();
  initButton();
  DEVICE_ID = setDeviceID();
  pinMode(SOS_LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
}

void loop()
{
  // Read GPS Data
  while (gpsSerial.available() > 0)
  {
    if (gps.encode(gpsSerial.read()))
    {
      if (gps.location.isValid())
      {
        gpsCoordinates = String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6) + "," + String(gps.altitude.meters());
        gpsTimestamp = String(gps.date.day()) + "-" + String(gps.date.month()) + "-" + String(gps.date.year()) + " " + String(gps.time.hour()) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second());
      }
    }
  }

  // Receive and Forward Packets
  int packetCondition = LoRa.parsePacket();
  if (packetCondition == sizeof(LoRaPacket))
  {
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
  if (everyNSeconds(gpsTimer, 10) && !isSOStriggered)
  {
    LoRaPacket gpsPacket = LoRaPacket::createPacket(DEVICE_ID, 0x01, 2, gpsCoordinates.c_str(), gpsTimestamp.c_str());
    gpsPacket.sendPacket(gpsPacket);
    displayGPSInfo();
  }

  // Press Button will toggle SOS mode
  if (checkButtonPress())
  {
    // Toggle SOS state
    isSOStriggered = !isSOStriggered;
  }

  if (isSOStriggered)
  {
    // Play SOS Morse Code pattern on LED and Buzzer
    playSOSMorseCode();

    // Send SOS packet every 5 seconds
    if (everyNSeconds(sosTimer, 5))
    {
      LoRaPacket sosPacket = LoRaPacket::createPacket(DEVICE_ID, 0x02, 1, gpsCoordinates.c_str(), gpsTimestamp.c_str());
      sosPacket.sendPacket(sosPacket);
    }
  }
  else
  {
    // Turn off LED and Buzzer when SOS is not active
    digitalWrite(SOS_LED_PIN, LOW);
    digitalWrite(BUZZER_PIN, LOW);
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
  if (!gps.location.isValid())
  {
    Serial.println("No GPS Data Available");
    return;
  }
  else if (!gps.date.isValid() || !gps.time.isValid())
  {
    Serial.println("No Date Available");
    return;
  }

  Serial.println("\nGPS Info:");
  Serial.println("Latitude: " + String(gps.location.lat(), 6));
  Serial.println("Longitude: " + String(gps.location.lng(), 6));
  Serial.println("Altitude: " + String(gps.altitude.meters()));
  Serial.println("Date: " + String(gps.date.month()) + "/" + String(gps.date.day()) + "/" + String(gps.date.year()));
  Serial.println("Time: " + String(gps.time.hour()) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second()));
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
  bool currentButtonState = digitalRead(BUTTON_PIN);

  // Change in state (from HIGH to LOW)
  if (lastButtonState == HIGH && currentButtonState == LOW)
  {
    // Short delay
    delay(50);
    currentButtonState = digitalRead(BUTTON_PIN);

    // Confirm it's still pressed
    if (currentButtonState == LOW)
    {
      lastButtonState = currentButtonState;
      // Button was just pressed
      return true;
    }
  }

  lastButtonState = currentButtonState;
  // Button is not pressed or already processed
  return false;
}

void playSOSMorseCode()
{
  // Morse code durations
  const int DOT = 200;
  const int DASH = 600;
  const int GAP = 200;
  const int LETTER_GAP = 600;
  const int WORD_GAP = 2000;

  // SOS Pattern: S(. . .) O(- - -) S(. . .)
  // 1=ON, 0=OFF, negative values = special gaps
  static const int sosPattern[] = {
      // S - three dots
      DOT, GAP, DOT, GAP, DOT, -LETTER_GAP,
      // O - three dashes
      DASH, GAP, DASH, GAP, DASH, -LETTER_GAP,
      // S - three dots
      DOT, GAP, DOT, GAP, DOT, -WORD_GAP};

  static const int patternLength = sizeof(sosPattern) / sizeof(sosPattern[0]);
  static int currentStep = 0;
  static unsigned long stepStartTime = 0;
  static bool isTurnedOn = true;

  unsigned long currentTime = millis();

  // Initialize on first run
  if (stepStartTime == 0)
  {
    stepStartTime = currentTime;
    isTurnedOn = true;
    currentStep = 0;
  }

  int duration = abs(sosPattern[currentStep]);
  bool shouldBeOn = (sosPattern[currentStep] > 0) && isTurnedOn;

  // Control LED and Buzzer
  if (shouldBeOn)
  {
    digitalWrite(SOS_LED_PIN, HIGH);
    digitalWrite(BUZZER_PIN, HIGH);
  }
  else
  {
    digitalWrite(SOS_LED_PIN, LOW);
    digitalWrite(BUZZER_PIN, LOW);
  }

  // Check if it's time to move to next step
  if (currentTime - stepStartTime >= duration)
  {
    stepStartTime = currentTime;
    currentStep++;

    // Toggle between ON and OFF for each step
    isTurnedOn = !isTurnedOn;

    // Reset pattern when finished
    if (currentStep >= patternLength)
    {
      currentStep = 0;
      isTurnedOn = true;
    }
  }
}