#include "LoRaPacket.h"
#include <LoRa.h>
#include <map>
#include <set>

std::map<uint16_t, String> neighborList; // deviceID -> "lat,lng,alt"
std::map<uint16_t, unsigned long> lastSeen;
std::set<String> seenPackets;

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

LoRaPacket LoRaPacket::createPacket(uint16_t deviceID, uint8_t type, uint8_t priority, const char *payload)
{
    LoRaPacket packet;
    packet.deviceID = deviceID;
    packet.type = type;
    packet.priority = priority;
    strncpy(packet.payload, payload, MAX_PAYLOAD_SIZE);
    packet.crc = LoRaPacket::calculateCRC(packet);
    return packet;
}

void LoRaPacket::printPacket(const LoRaPacket &packet)
{
    Serial.printf("DeviceID: %u | Type: %u | Priority: %u | Payload: %s | CRC: 0x%04X\n",
                  packet.deviceID,
                  packet.type,
                  packet.priority,
                  packet.payload,
                  packet.crc);
    Serial.println();
}

void LoRaPacket::sendPacket(const LoRaPacket &packet)
{
    printPacket(packet);
    LoRa.beginPacket();
    LoRa.write((uint8_t *)&packet, sizeof(LoRaPacket));
    LoRa.endPacket();
    if (packet.type == 0x02)
        Serial.println("SOS Packet Sent");
    else
        Serial.println("GPS Packet Sent");
}

LoRaPacket LoRaPacket::receivePacket()
{
    LoRaPacket packet;
    LoRa.readBytes((uint8_t *)&packet, sizeof(LoRaPacket));
    printPacket(packet);
    Serial.println("Packet Received");

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
        Serial.println("SOS Packet Received and Forwarded");
    }
    else if (!isPacketAlreadySeen(packet))
    {
        sendPacket(packet);
        markPacketAsSeen(packet);
        Serial.println("GPS Packet Received and Forwarded");
    }
}

void LoRaPacket::updateNeighborList(const LoRaPacket &packet)
{
    if (packet.type == 0x01)
    {
        String gpsInfo = String(packet.payload);
        neighborList[packet.deviceID] = gpsInfo;
        lastSeen[packet.deviceID] = millis();
    }
}

void LoRaPacket::printNeighborList()
{
    Serial.println("Known neighbors:");
    for (auto &entry : neighborList)
    {
        Serial.println("\nDevice " + String(entry.first) + " -> " + entry.second);
    }
}
