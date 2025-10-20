#include "LoRaPacket.h"
#include <LoRa.h>
#include <map>
#include <set>

extern uint16_t DEVICE_ID;
std::map<uint16_t, String> neighborList;    // deviceID -> "lat,lng,alt"
std::map<uint16_t, unsigned long> lastSeen; // deviceID -> last seen timestamp
std::set<String> seenPackets;               // [ [deviceID|type|payload], [deviceID|type|payload], ... ]

// Neighbor List Management
bool LoRaPacket::isPacketAlreadySeen(const LoRaPacket &packet)
{
    String key = String(packet.deviceID) + "|" + String(packet.type) + "|" + String(packet.payload);
    return seenPackets.count(key) > 0;
}

void LoRaPacket::markPacketAsSeen(const LoRaPacket &packet)
{
    String key = String(packet.deviceID) + "|" + String(packet.type) + "|" + String(packet.payload);
    seenPackets.insert(key);
}

void LoRaPacket::printSeenPackets()
{
    Serial.println("Seen Packets:");
    for (const String &packet : seenPackets)
    {
        Serial.println(" - " + packet);
    }
}

// Packet Handling
uint16_t LoRaPacket::calculateCRC(const LoRaPacket &packet)
{
    uint16_t crc = 0xFFFF;
    uint8_t *data = (uint8_t *)&packet;
    for (int i = 0; i < sizeof(LoRaPacket) - sizeof(packet.crc); i++)
    {
        crc ^= data[i];
        for (int j = 0; j < 8; j++)
        {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}

LoRaPacket LoRaPacket::createPacket(uint16_t deviceID, uint8_t type, uint8_t priority, const char *payload, const char *timestamp)
{
    LoRaPacket packet;
    packet.deviceID = deviceID;
    packet.type = type;
    packet.priority = priority;

    strncpy(packet.payload, payload, MAX_PAYLOAD_SIZE - 1);
    packet.payload[MAX_PAYLOAD_SIZE - 1] = '\0';

    strncpy(packet.timestamp, timestamp, MAX_TIMESTAMP_SIZE - 1);
    packet.timestamp[MAX_TIMESTAMP_SIZE - 1] = '\0';

    packet.crc = LoRaPacket::calculateCRC(packet);
    return packet;
}

void LoRaPacket::printPacket(const LoRaPacket &packet)
{
    Serial.println("------------------------------------------------------------------------------");
    // Serial.printf("| DeviceID: %u | Type: %u | Priority: %u | Payload: %s | CRC: 0x%04X | \n",
    //    packet.deviceID,
    //    packet.type,
    //    packet.priority,
    //    packet.payload,
    //    packet.crc);
    Serial.println(" | DeviceID: " + String(packet.deviceID) +
                   " | Type: " + String(packet.type) +
                   " | Priority: " + String(packet.priority) +
                   " | Payload: " + String(packet.payload) +
                   " | CRC: 0x" + String(packet.crc, HEX) +
                   " | Timestamp: " + String(packet.timestamp) +
                   " |");
    Serial.println("------------------------------------------------------------------------------");
}

void LoRaPacket::sendPacket(const LoRaPacket &packet)
{
    printPacket(packet);
    LoRa.beginPacket();
    LoRa.write((uint8_t *)&packet, sizeof(LoRaPacket));
    LoRa.endPacket();
    if (packet.type == 0x02 && packet.deviceID == DEVICE_ID)
        Serial.println("\033[91mSOS Packet Sent\033[0m");
    else if (packet.type == 0x01 && packet.deviceID == DEVICE_ID)
        Serial.println("\033[32mGPS Packet Sent\033[0m");
}

LoRaPacket LoRaPacket::receivePacket()
{
    LoRaPacket packet;
    LoRa.readBytes((uint8_t *)&packet, sizeof(LoRaPacket));
    printPacket(packet);
    Serial.println("\033[36mPacket Received\033[0m");

    return packet;
}

bool LoRaPacket::verifyCRC(const LoRaPacket &packet)
{
    return (packet.crc == calculateCRC(packet));
}

// Forwarding Packets
void LoRaPacket::forwardPacket(const LoRaPacket &packet, uint16_t myDeviceID)
{
    if (packet.deviceID == myDeviceID)
        return;

    if (packet.type == 0x02)
    { // SOS packet, always forward
        sendPacket(packet);
        Serial.println("\033[31mSOS Packet Received and Forwarded\033[0m");
    }
    // else if (!isPacketAlreadySeen(packet))
    else if (packet.type == 0x01)
    {
        sendPacket(packet);
        markPacketAsSeen(packet);
        Serial.println("\033[33mGPS Packet Received and Forwarded\033[0m");
    }
}

void LoRaPacket::updateNeighborList(const LoRaPacket &packet)
{
    if (packet.type == 0x01)
    {
        String gpsInfo = String(packet.payload);
        String timestamp = String(packet.timestamp);

        neighborList[packet.deviceID] = gpsInfo;
        lastSeen[packet.deviceID] = timestamp.toInt();
    }
}

void LoRaPacket::printNeighborList()
{
    Serial.println("\nKnown neighbors:");
    for (auto &entry : neighborList)
    {
        Serial.println("Device " + String(entry.first) + " -> " + entry.second + "\n");
    }
}
