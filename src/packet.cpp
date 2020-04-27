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
    _sent = pkt._sent;
    _timeout = pkt._timeout;
    _sentTimestamp = pkt._sentTimestamp;
    _maxRetry = pkt._maxRetry;
    _dataSize = pkt._dataSize;
    _pktSize = pkt._pktSize;
    delete[] _pkt;
    _pkt = new uint8_t[_pktSize];
    _header = _pkt;
    _data = _header + _headerSize;
    memcpy(_pkt, pkt._pkt, _pktSize);
    return *this;
}

Packet Packet::buildPktFromBase16str(const String &s)
{
    String input(s); // Make a deep copy to be able to do trim()
    input.trim();
    const size_t inputLength = input.length();
    const size_t outputLength = inputLength / 2;
    if (outputLength <= _headerSize) /* This is not a packet if the size is not even greater than the header size*/
        return Packet();

    Packet output(outputLength);
    uint8_t *outputPtr = output.get();

    for (size_t i = 0; i < outputLength; ++i)
    {
        char toDo[3];
        toDo[0] = input[i * 2];
        toDo[1] = input[i * 2 + 1];
        toDo[2] = '\0';
        int out = strtoul(toDo, 0, 16);
        outputPtr[i] = uint8_t(out);
    }

    return output;
}

void Packet::hasJustBeenSent()
{
    _sent++;
    _sentTimestamp = millis();
}