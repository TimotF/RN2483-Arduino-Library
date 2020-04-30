#ifndef PACKET_H
#define PACKET_H

#include "Arduino.h"

class Packet
{
public:
    enum PROTOCOL_VERSION
    {
        VERSION_1 = 0,
        VERSION_8 = 7
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
        ACK = 3
    };

    Packet(size_t dataSize = 0, const uint8_t *data = NULL, bool dataContainsHeader = false)
    {
        _sent = false;
        if (dataContainsHeader && (dataSize >= _headerSize))
        {
            _pktSize = dataSize;
            _dataSize = dataSize - _headerSize;
            _pkt = new uint8_t[_pktSize];
            _header = _pkt;
            _data = _header + _headerSize;
            if (data != NULL)
            {
                memcpy(_pkt, data, _pktSize);
            }
        }
        else
        {
            _dataSize = dataSize;
            _pktSize = _dataSize + _headerSize;
            _pkt = new uint8_t[_pktSize];
            _header = _pkt;
            _data = _header + _headerSize;
            if (_dataSize > 0 && data != NULL)
            {
                memcpy(_data, data, _dataSize);
            }
            memset(_header, 0, _headerSize);
        }
    }
    Packet(const Packet &pkt)
    {
        _sent = pkt._sent;
        _timeout = pkt._timeout;
        _sentTimestamp = pkt._sentTimestamp;
        _maxRetry = pkt._maxRetry;
        _dataSize = pkt._dataSize;
        _pktSize = pkt._pktSize;
        _pkt = new uint8_t[_pktSize];
        _header = _pkt;
        _data = _header + _headerSize;
        memcpy(_pkt, pkt._pkt, _pktSize);
    }

    static Packet buildPktFromBase16str(const String &s);

    ~Packet()
    {
        delete[] _pkt;
    }

    Packet &operator=(const Packet &pkt);
    bool operator==(const Packet &pkt)
    {
        if (this->_pktSize != pkt._pktSize)
            return false;
        return !memcmp(this->_pkt, pkt._pkt, _pktSize);
    }

    uint8_t *get() { return _pkt; }
    uint8_t *getData() { return _data; }
    uint8_t *getHeader() { return _header; }

    PROTOCOL_VERSION getProtocolVersion() { return (PROTOCOL_VERSION)((_header[0] & 0xE0) >> 5); }
    QoS getQoS() { return (QoS)((_header[0] & 0x10) >> 4); }
    PACKET_TYPE getType() { return (PACKET_TYPE)(_header[0] & 0x07); }
    size_t getPktSize() { return _pktSize; }
    size_t getHeaderSize() { return _headerSize; }
    size_t getDataSize() { return _dataSize; }
    bool isSplit() { return (_header[0] & 0x08) > 0; }
    uint8_t getSourceID() { return _header[2]; }
    uint8_t getDestID() { return _header[3]; }
    uint8_t getPktNumber() { return _header[4]; }
    uint8_t getSent() { return _sent; }
    uint8_t getMaxRetry() { return _maxRetry; }
    uint32_t getSentTimestamp() { return _sentTimestamp; }
    uint32_t getTimeout() { return _timeout; }

    void setProtocolVersion(PROTOCOL_VERSION version);
    void setQoS(QoS qos);
    void setType(PACKET_TYPE type);
    void setSplit(bool split);
    void setSourceID(uint8_t id) { _header[2] = id; }
    void setDestID(uint8_t id) { _header[3] = id; }
    void setPktNumber(uint8_t nb) { _header[4] = nb; }
    void hasJustBeenSent();

    void print();

private:
    uint8_t *_pkt;
    size_t _pktSize = 0;
    /* header : protocol_version (3), QoS (1), pktSplit (1), pktType (3), 
    futurUse (8), 
    source dev ID (8), 
    dest dev ID (8), 
    pktNumber (8) */
    uint8_t *_header;
    static const size_t _headerSize;
    uint8_t *_data;
    size_t _dataSize = 0;
    uint32_t _timeout = 0;       /* if no ack received during this time, then the packet is resent */
    uint32_t _sentTimestamp = 0; /* timestamp of the time that the packet was sent */
    uint8_t _sent = 0;           /* The number of times the packet was sent */
    uint8_t _maxRetry = 5;       /* The maximum number of times a packet is sent before dropping it */
};

#endif