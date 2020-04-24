#ifndef PACKET_H
#define PACKET_H

#include "Arduino.h"

class Packet
{
public:
    enum PROTOCOL_VERSION
    {
        VERSION_1 = 0
    };

    enum QoS
    {
        ONE_PACKET_AT_MOST = 0,
        AT_LEAST_ONE_PACKET = 1
    };

    enum PACKET_TYPE
    {
        DISCOVER = 0,
        OTA = 1,
        DATA = 2,
        NACK = 3
    };

    Packet(size_t dataSize = 0, uint8_t *data = NULL);
    ~Packet();

    bool parse(size_t pkt_size, uint8_t *packet);
    bool attachData(size_t dataSize, uint8_t *data);
    uint8_t *get();

    PROTOCOL_VERSION getProtocolVersion();
    QoS getQoS();
    PACKET_TYPE getType();
    size_t getPktSize();
    size_t getHeaderSize();
    size_t getDataSize();
    bool isSplit();
    uint8_t getSourceID();
    uint8_t getDestID();

    bool setProtocolVersion(PROTOCOL_VERSION version);
    bool setQoS(QoS qos);
    bool setType(PACKET_TYPE type);
    bool setSourceID(uint8_t id);
    bool setDestID(uint8_t id);
    bool setSplit(bool split);

private:
    uint8_t *_pkt;
    size_t _pktSize;
    uint8_t *_header;
    size_t _headerSize;
    uint8_t *_data;
    size_t _dataSize;
};

#endif