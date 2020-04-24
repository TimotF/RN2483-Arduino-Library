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

    Packet(size_t dataSize = 0, uint8_t *data = NULL)
    {
        _dataSize = dataSize;
        _pktSize = _dataSize + _headerSize;
        _pkt = new uint8_t[_pktSize];
        _header = _pkt;
        _data = _header + _headerSize;
        if (_dataSize > 0)
        {
            memcpy(_data, data, _dataSize);
        }
    }
    ~Packet(){
        delete [] _pkt;
    }

    bool parse(size_t pkt_size, uint8_t *packet);
    uint8_t *get() { return _pkt;}

    PROTOCOL_VERSION getProtocolVersion(){return (PROTOCOL_VERSION)((_header[0]&0xE0)>>5);}
    QoS getQoS(){return (QoS)((_header[0]&10)>>4);}
    PACKET_TYPE getType(){return (PACKET_TYPE)(_header[0]&0x07);}
    size_t getPktSize() { return _pktSize; }
    size_t getHeaderSize() { return _headerSize; }
    size_t getDataSize() { return _dataSize; }
    bool isSplit(){return _header[0]&0x08 > 0;}
    uint8_t getSourceID() { return _header[2];}
    uint8_t getDestID() { return _header[3];}
    uint8_t getPktNumber() { return _header[4];}

    bool setProtocolVersion(PROTOCOL_VERSION version);
    bool setQoS(QoS qos);
    bool setType(PACKET_TYPE type);
    bool setSourceID(uint8_t id);
    bool setDestID(uint8_t id);
    bool setSplit(bool split);

private:
    uint8_t *_pkt;
    size_t _pktSize = 0;
    /* header : protocol_version (3), QoS (1), pktSplit (1), pktType (3), 
    futurUse (8), 
    source dev ID (8), 
    dest dev ID (8), 
    pktNumber (8) */
    uint8_t *_header; 
    const size_t _headerSize = 5;
    uint8_t *_data;
    size_t _dataSize = 0;
};

#endif