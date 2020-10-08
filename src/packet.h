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
        OTA = 1,  /* OTA packet type */
        DATA = 2, /* DATA packet type */
        ACK = 3   /* ACK packet type */
    };

    enum PRIORITY /* packet priority */
    {
        PRIORITY_LOW,    /* no priority requirement, pkt will be sent after pkts with higher priority */
        PRIORITY_HIGH,   /* high priority (ex: OTA) */
        PRIORITY_HIGHEST /* critical priority (ex: ACK) */
    };

    Packet(size_t dataSize = 0, const uint8_t *data = NULL, bool dataContainsHeader = false) /* Packet constructor */
    {
        _sent = false;                                       /* reset _sent counter */
        if (dataContainsHeader && (dataSize >= _headerSize)) /* if header is contained inside the data array */
        {
            _pktSize = dataSize;
            _pkt = new uint8_t[_pktSize];
            _header = _pkt;
            _data = _header + _headerSize;
            if (data != NULL)
                memcpy(_pkt, data, _pktSize);
            else
                memset(_pkt, 0, _pktSize);
            _paddingCount = _header[1];
            _dataSize = _pktSize - _headerSize - _paddingCount;
        }
        else /* else, setup packet with empty header */
        {
            _dataSize = dataSize;
            _paddingCount = _paddingModule - ((_dataSize + _headerSize) % _paddingModule);
            _pktSize = _dataSize + _headerSize + _paddingCount;
            _pkt = new uint8_t[_pktSize];
            _header = _pkt;
            _data = _header + _headerSize;
            if (_dataSize > 0 && data != NULL)
            {
                memcpy(_data, data, _dataSize);
            }
            memset(_header, 0, _headerSize);
            _header[1] = _paddingCount;
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
        _paddingCount = pkt._paddingCount;
        _priority = pkt._priority;
        _pktNumberSet = pkt._pktNumberSet;
        memcpy(_pkt, pkt._pkt, _pktSize);
    }

    static Packet buildPktFromBase16str(const String &s, const String cypherKey = "notUsed"); /* Method to build a packet from an hex formated string */

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

    uint8_t *get() { return _pkt; }                               /* getter for the packet array */
    uint8_t *getData() { return _data; }                          /* getter for the data array */
    uint8_t *getHeader() { return _header; }                      /* getter for the header array */
    uint8_t *getCyphered(uint8_t *pktCyphered, String cypherKey); /* padd the packet and return a copy of its content cyphered */

    PROTOCOL_VERSION getProtocolVersion() const { return (PROTOCOL_VERSION)((_header[0] & 0xE0) >> 5); } /* getter for the protocol version */
    QoS getQoS() const { return (QoS)((_header[0] & 0x10) >> 4); }                                       /* getter for the QOS */
    PACKET_TYPE getType() const { return (PACKET_TYPE)(_header[0] & 0x07); }                             /* getter for the packet type */
    size_t getPktSize() const { return _pktSize; }                                                       /* getter for the packet size */
    size_t getHeaderSize() const { return _headerSize; }                                                 /* getter for the header size */
    size_t getDataSize() const { return _dataSize; }                                                     /* getter for the data size */
    bool isSplit() const { return (_header[0] & 0x08) > 0; }                                             /* getter for the packet split flag */
    uint8_t getSourceID() const { return (_header[2] & 0xF0) >> 4; }                                     /* getter for source ID */
    uint8_t getDestID() const { return _header[2] & 0x0F; }                                              /* getter for dest ID */
    uint8_t getPktNumber() const { return _header[4]; }                                                  /* getter for the packet number*/
    uint8_t getSent() const { return _sent; }                                                            /* get the number of times a packet was sent */
    uint8_t getMaxRetry() const { return _maxRetry; }                                                    /* get the maximum number of times a packet can be resent if considered lost */
    uint32_t getSentTimestamp() const { return _sentTimestamp; }                                         /* get the timestamp of the time that the packet was sent */
    uint32_t getTimeout() const { return _timeout; }                                                     /* get the maximum time before considering a packet is lost */
    PRIORITY getPriority() const { return _priority; }                                                   /* get the priority of a packet */
    bool pktNbSet() const { return _pktNumberSet; }                                                      /* returns true if pktnb has been set at least once */

    void setProtocolVersion(PROTOCOL_VERSION version);   /* setter for protocol version field */
    void setQoS(QoS qos);                                /* setter for QoS field */
    void setType(PACKET_TYPE type);                      /* Setter for pkt type field */
    void setSplit(bool split);                           /* setter for packet split flag */
    void setSourceID(uint8_t id);                        /* setter for SourceID */
    void setDestID(uint8_t id);                          /* setter for destID */
    void setPktNumber(uint8_t nb);                       /* setter for pkt number */
    void hasJustBeenSent();                              /* used to mark the packet as sent */
    void print();                                        /* print infos and content of a packet */
    void setPiority(PRIORITY prio) { _priority = prio; } /* setter for packet priority */

    uint8_t computeChecksum(); /* compute, set, and return a 1 byte xor checksum */
    bool checkIntegity();      /* returns true when integrity check is ok */

private:
    uint8_t *_pkt;       /* packet array, containing the header array and the data array */
    size_t _pktSize = 0; /* packet size which is the sum of headerSize and dataSize */
    /* header : protocol_version (3), QoS (1), pktSplit (1), pktType (3), 
    paddingCount (8), 
    SourceID (4), DestID(4), 
    pkt checksum (8), 
    pktNumber (8) */
    uint8_t *_header;                  /* pointer to the part of the packet array that represents the header */
    static const size_t _headerSize;   /* header size (fixed) */
    uint8_t *_data;                    /* pointer to the part of the packet array that represents the data */
    size_t _dataSize = 0;              /* length of data */
    uint32_t _timeout = 0;             /* if no ack received during this time, then the packet is resent */
    uint32_t _sentTimestamp = 0;       /* timestamp of the time that the packet was sent */
    uint8_t _sent = 0;                 /* The number of times the packet was sent */
    uint8_t _maxRetry = 10;            /* The maximum number of times a packet is sent before dropping it */
    uint8_t _paddingCount = 0;         /* The number of padded bytes added at the end of the data */
    uint8_t _paddingModule = 16;       /* The packet length should be a multiple of this */
    PRIORITY _priority = PRIORITY_LOW; /* The priority of the packet */
    bool _pktNumberSet = false;        /* true if packet number has been set at least once */
};

class PktQueueTx
{
public:
    bool addPacket(Packet pkt);         /* Add a packet in the queue and automatically set the packet number. Returns true if operation was successful. */
    bool addACK(const Packet &pkt2ack); /* Add in the TX queue a packet of type ACK corresponding to the packet passed in param */

    bool getNextPacket(Packet *pkt);       /* Returns the next packet to be send as param. Function returns true if a valid packet has been found.*/
    bool markPktAsSent(const Packet &pkt); /* Mark a packet as sent. Returns true if successful. */

    bool removePkt(const uint8_t &destID, const uint8_t &pktNumber); /* Remove a pkt from the queue. Returns true if the pkt was successfully removed, false otherwise. */
    bool removePkt(const Packet &pkt);
    void clear();   /* Erase all packets from queue */
    bool cleanUp(); /* Remove packets that have been sent too many times. Returns true if at least 1 packet was removed. */

    int size() const; /* Returns the number of packets in the queue */

private:
    std::vector<Packet> _pktQueue;                      /* pkt queue */
    uint32_t _lastPktSentTime = 0;                      /* timestamp of the moment the last packet was sent */
    SemaphoreHandle_t _mutex = xSemaphoreCreateMutex(); /* queue mutex */
    uint8_t _nextPktNumber[16] = {0};                   /* stores the packet number depending on the destID */
};

class PktQueueRx
{
public:
    bool addPacket(Packet pkt); /* Add a packet in the queue. Returns true if operation was successful. */

    bool receivedCompletePkt(Packet *pkt); /* Returns the next packet to be send as param. Function returns true if a valid packet has been found.*/

    bool removePkt(); /* Remove a pkt from the queue. Returns true if the pkt was successfully removed, false otherwise. */
    void cleanUp();   /* Erase all packets from queue */

    int card(); /* Returns the number of packets in the queue */

private:
    std::vector<Packet> _pktQueue;                      /* Packet queue, in the order of arrival. */
    uint32_t _lastPktRecvTime = 0;                      /* Timestamp of the last received packet. */
    SemaphoreHandle_t _mutex = xSemaphoreCreateMutex(); /* Mutex to protect queue access */
};

#endif