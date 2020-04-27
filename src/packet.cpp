#include "packet.h"

const size_t Packet::_headerSize = 5;

/* header : protocol_version (3), QoS (1), pktSplit (1), pktType (3), 
    futurUse (8), 
    source dev ID (8), 
    dest dev ID (8), 
    pktNumber (8) */
void Packet::setProtocolVersion(PROTOCOL_VERSION version)
{
    _header[0] &= 0x1F;
    _header[0] |= ((version & 0x07) << 5);
}

void Packet::setQoS(QoS qos)
{
    _header[0] &= 0xEF;
    _header[0] |= ((qos & 0x01) << 4);
}

void Packet::setSplit(bool split)
{
    _header[0] &= 0xF7;
    _header[0] |= ((split & 0x01) << 3);
}

void Packet::setType(PACKET_TYPE type)
{
    _header[0] &= 0xF8;
    _header[0] |= type & 0x07;
}

Packet &Packet::operator=(const Packet &pkt)
{
    Serial.println("operator=");
    _dataSize = pkt._dataSize;
    _pktSize = pkt._pktSize;
    _pkt = new uint8_t[_pktSize];
    _header = _pkt;
    _data = _header + _headerSize;
    memcpy(_pkt, pkt._pkt, _pktSize);
    return *this;
}