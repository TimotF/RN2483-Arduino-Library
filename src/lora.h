#ifndef LORA_H
#define LORA_H
#include "Arduino.h"
#include "rn2xx3.h"


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

class LoRa
{
public:
    LoRa(Stream &serial, const bool &useP2P = false) : _lora(serial)
    {
        init(useP2P);
    }
    bool init(const bool &useP2P);
    bool sendData(uint8_t *);
    bool receivedData();
    void loop();
    //TODO : implement callback for received packets

private:
    bool _useP2P = false;
    rn2xx3 _lora;
    // TODO : 2 queues de packets, 1 pour le RX, 1 pour le TX
};

#endif