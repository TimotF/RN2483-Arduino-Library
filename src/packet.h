#ifndef PACKET_H
#define PACKET_H

#include "Arduino.h"
#include <hwcrypto/aes.h>

class Packet
{
public:
    enum PROTOCOL_VERSION /* protocol version definition */
    {
        VERSION_1 = 0 /* protocol version 1, developped in April 2020 */
    };

    enum QoS /* SoS definition */
    {
        ONE_PACKET_AT_MOST = 0, /* No QoS required, packet may be lost */
        AT_LEAST_ONE_PACKET = 1 /* Packet will be received at least once */
    };

    enum PACKET_TYPE /* packet type definition */
    {
        PING = 0, /* Discover packet type*/
        OTA = 1,      /* OTA packet type */
        DATA = 2,     /* DATA packet type */
        ACK = 3       /* ACK packet type */
    };

    Packet(size_t dataSize = 0, const uint8_t *data = NULL, bool dataContainsHeader = false) /* Packet constructor */
    {
        _sent = false;                                       /* reset _sent counter */
        if (dataContainsHeader && (dataSize >= _headerSize)) /* if header is contained inside the data array */
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
        else /* else, setup packet with empty header */
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
    Packet(const Packet &pkt) /* Packet copy constructor */
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

    static Packet buildPktFromBase16str(const String &s, const String cypherKey="notUsed"); /* Method to build a packet from an hex formated string */

    ~Packet() /* packet destructor */
    {
        delete[] _pkt; /* delete memory allocated to the packet */
    }

    Packet &operator=(const Packet &pkt); /* Overload of operator= */
    bool operator==(const Packet &pkt)    /* Overload of operator== */
    {
        if (this->_pktSize != pkt._pktSize)             /* if packets size are different, */
            return false;                               /* comparison is false */
        return !memcmp(this->_pkt, pkt._pkt, _pktSize); /* else, return the comparison of the memory allocated to the packet */
    }

    uint8_t *get() { return _pkt; }          /* getter for the packet array */
    uint8_t *getData() { return _data; }     /* getter for the data array */
    uint8_t *getHeader() { return _header; } /* getter for the header array */

    PROTOCOL_VERSION getProtocolVersion() { return (PROTOCOL_VERSION)((_header[0] & 0xE0) >> 5); } /* getter for the protocol version */
    QoS getQoS() { return (QoS)((_header[0] & 0x10) >> 4); }                                       /* getter for the QOS */
    PACKET_TYPE getType() { return (PACKET_TYPE)(_header[0] & 0x07); }                             /* getter for the packet type */
    size_t getPktSize() { return _pktSize; }                                                       /* getter for the packet size */
    size_t getHeaderSize() { return _headerSize; }                                                 /* getter for the header size */
    size_t getDataSize() { return _dataSize; }                                                     /* getter for the data size */
    bool isSplit() { return (_header[0] & 0x08) > 0; }                                             /* getter for the packet split flag */
    uint8_t getSourceID() { return _header[2]; }                                                   /* getter for source ID */
    uint8_t getDestID() { return _header[3]; }                                                     /* getter for dest ID */
    uint8_t getPktNumber() { return _header[4]; }                                                  /* getter for the packet number*/
    uint8_t getSent() { return _sent; }                                                            /* get the number of times a packet was sent */
    uint8_t getMaxRetry() { return _maxRetry; }                                                    /* get the maximum number of times a packet can be resent if considered lost */
    uint32_t getSentTimestamp() { return _sentTimestamp; }                                         /* get the timestamp of the time that the packet was sent */
    uint32_t getTimeout() { return _timeout; }                                                     /* get the maximum time before considering a packet is lost */

    void setProtocolVersion(PROTOCOL_VERSION version); /* setter for protocol version field */
    void setQoS(QoS qos);                              /* setter for QoS field */
    void setType(PACKET_TYPE type);                    /* Setter for pkt type field */
    void setSplit(bool split);                         /* setter for packet split flag */
    void setSourceID(uint8_t id) { _header[2] = id; }  /* setter for SourceID */
    void setDestID(uint8_t id) { _header[3] = id; }    /* setter for destID */
    void setPktNumber(uint8_t nb) { _header[4] = nb; } /* setter for pkt number */
    void hasJustBeenSent();                            /* used to mark the packet as sent */
    void print();                                      /* print infos and content of a packet */

private:
    uint8_t *_pkt;       /* packet array, containing the header array and the data array */
    size_t _pktSize = 0; /* packet size which is the sum of headerSize and dataSize */
    /* header : protocol_version (3), QoS (1), pktSplit (1), pktType (3), 
    futurUse (8), 
    source dev ID (8), 
    dest dev ID (8), 
    pktNumber (8) */
    uint8_t *_header;                /* pointer to the part of the packet array that represents the header */
    static const size_t _headerSize; /* header size (fixed) */
    uint8_t *_data;                  /* pointer to the part of the packet array that represents the data */
    size_t _dataSize = 0;            /* length of data */
    uint32_t _timeout = 0;           /* if no ack received during this time, then the packet is resent */
    uint32_t _sentTimestamp = 0;     /* timestamp of the time that the packet was sent */
    uint8_t _sent = 0;               /* The number of times the packet was sent */
    uint8_t _maxRetry = 10;          /* The maximum number of times a packet is sent before dropping it */
};

#endif