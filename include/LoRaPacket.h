#ifndef LORAPACKET_H
#define LORAPACKET_H

#include <Arduino.h>

#define MAX_PAYLOAD_SIZE 100
#define MAX_TIMESTAMP_SIZE 20

// LoRa Packet Structure
struct __attribute__((packed)) LoRaPacket
{
    uint16_t deviceID;              // Unique ID for each PVM/ESP
    uint8_t type;                   // 0x01 = GPS, 0x02 = SOS
    uint8_t priority;               // 0 = normal, 1 = high (SOS has highest priority)
    char payload[MAX_PAYLOAD_SIZE]; // GPS data
    char timestamp[MAX_TIMESTAMP_SIZE];
    uint16_t crc;

    // Packet Handling
    static LoRaPacket createPacket(uint16_t deviceID, uint8_t type, uint8_t priority, const char *payload, const char *timestamp);
    static void sendPacket(const LoRaPacket &packet);
    static LoRaPacket receivePacket();
    static void printPacket(const LoRaPacket &packet);
    static bool verifyCRC(const LoRaPacket &packet);
    static uint16_t calculateCRC(const LoRaPacket &packet);

    // Forwarding Packets
    void forwardPacket(const LoRaPacket &packet, uint16_t myDeviceID);
    bool isPacketAlreadySeen(const LoRaPacket &packet);
    void markPacketAsSeen(const LoRaPacket &packet);

    // Neighbor List Management
    static void updateNeighborList(const LoRaPacket &packet);
    static void printNeighborList();
    void printSeenPackets();
};
#endif
