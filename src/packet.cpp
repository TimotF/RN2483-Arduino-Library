#include "packet.h"
#include "RemoteDebug.h"

#ifndef DEBUG_DISABLED
extern RemoteDebug Debug;
#else
    #undef debugA
    #undef debugP
    #undef debugV
    #undef debugD
    #undef debugI
    #undef debugW
    #undef debugE
    #ifdef DEBUG_SERIAL_PACKET
        #if DEBUG_SERIAL_PACKET >= 4
            #define debugA(fmt, ...) Serial.printf("[A][C%d][%ld][%s:%d] %s: \t" fmt "\n", xPortGetCoreID(), millis(), __FILE__, __LINE__, __func__, ##__VA_ARGS__)
            #define debugP(fmt, ...) Serial.printf("[P][C%d][%ld][%s:%d] %s: \t" fmt "\n", xPortGetCoreID(), millis(), __FILE__, __LINE__, __func__, ##__VA_ARGS__)
            #define debugV(fmt, ...) Serial.printf("[V][C%d][%ld][%s:%d] %s: \t" fmt "\n", xPortGetCoreID(), millis(), __FILE__, __LINE__, __func__, ##__VA_ARGS__)
        #else
            #define debugA(fmt, ...) 
            #define debugP(fmt, ...) 
            #define debugV(fmt, ...) 
        #endif
        #if DEBUG_SERIAL_PACKET >= 3
            #define debugD(fmt, ...) Serial.printf("[D][C%d][%ld][%s:%d] %s: \t" fmt "\n", xPortGetCoreID(), millis(), __FILE__, __LINE__, __func__, ##__VA_ARGS__)
        #else
            #define debugD(fmt, ...)
        #endif
        #if DEBUG_SERIAL_PACKET >= 2
            #define debugI(fmt, ...) Serial.printf("[I][C%d][%ld][%s:%d] %s: \t" fmt "\n", xPortGetCoreID(), millis(), __FILE__, __LINE__, __func__, ##__VA_ARGS__)
        #else
            #define debugI(fmt, ...)
        #endif
        #if DEBUG_SERIAL_PACKET >= 1
            #define debugW(fmt, ...) Serial.printf("[W][C%d][%ld][%s:%d] %s: \t" fmt "\n", xPortGetCoreID(), millis(), __FILE__, __LINE__, __func__, ##__VA_ARGS__)
        #else
            #define debugW(fmt, ...)
        #endif
        #if DEBUG_SERIAL_PACKET >= 0
            #define debugE(fmt, ...) Serial.printf("[E][C%d][%ld][%s:%d] %s: \t" fmt "\n", xPortGetCoreID(), millis(), __FILE__, __LINE__, __func__, ##__VA_ARGS__)
        #else
            #define debugE(fmt, ...)
        #endif
    #else
        #define debugA(fmt, ...) 
        #define debugP(fmt, ...) 
        #define debugV(fmt, ...) 
        #define debugD(fmt, ...) 
        #define debugI(fmt, ...) 
        #define debugW(fmt, ...) 
        #define debugE(fmt, ...) 
    #endif
#endif

const size_t Packet::_headerSize = 5; /* the header size is const. Header is defined as shown below */
/* header : protocol_version (3), QoS (1), pktSplit (1), pktType (3), 
    paddingCount (8), 
    futur use (8), 
    pkt checksum (8), 
    pktNumber (8) */

static void crypt(esp_aes_context *ctx, int mode, size_t length, const unsigned char *input, unsigned char *output)
{

    if (length % 16 != 0)
    {
        debugE("length not a multiple of 16, padding required!");
        return;
    }

    unsigned char inputBuffer[length];
    unsigned char outputBuffer[length];
    memset(inputBuffer, 0, length);
    memcpy(inputBuffer, input, length);

    for (int i = 0; i < length; i += 16)
    {
        esp_aes_crypt_ecb(ctx, mode, inputBuffer + i, outputBuffer + i);
    }
    memcpy(output, outputBuffer, length);
}

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

void Packet::setSourceID(uint8_t id)
{
    _header[2] &= 0x0F;
    _header[2] |= (id & 0x0F)<<4;
}

void Packet::setDestID(uint8_t id)
{
    _header[2] &= 0xF0;
    _header[2] |= id & 0x0F;
}

Packet &Packet::operator=(const Packet &pkt)
{
    _sent = pkt._sent;
    _timeout = pkt._timeout;
    _sentTimestamp = pkt._sentTimestamp;
    _maxRetry = pkt._maxRetry;
    _dataSize = pkt._dataSize;
    _pktSize = pkt._pktSize;
    _paddingCount = pkt._paddingCount;
    delete[] _pkt;
    _pkt = new uint8_t[_pktSize];
    _header = _pkt;
    _data = _header + _headerSize;
    memcpy(_pkt, pkt._pkt, _pktSize);
    return *this;
}

Packet Packet::buildPktFromBase16str(const String &s, const String cypherKey)
{
    String input(s); // Make a deep copy to be able to do trim()
    input.trim();
    const size_t inputLength = input.length();
    const size_t outputLength = inputLength / 2;
    if (outputLength < _headerSize) /* This is not a packet if the size is not even greater than the header size*/
        return Packet();

    if (cypherKey != "notUsed" && outputLength % 16 != 0)
    {
        debugE("cannot build this packet!");
        return Packet();
    }

    uint8_t receivedData[outputLength];

    for (size_t i = 0; i < outputLength; ++i)
    {
        char toDo[3];
        toDo[0] = input[i * 2];
        toDo[1] = input[i * 2 + 1];
        toDo[2] = '\0';
        int out = strtoul(toDo, 0, 16);
        receivedData[i] = uint8_t(out);
    }

    if (cypherKey.length() == 32) /*We need to decrypt the packet*/
    {
        uint8_t key[32];
        memcpy(key, cypherKey.c_str(), 32);
        esp_aes_context ctx;
        esp_aes_init(&ctx);
        esp_aes_setkey(&ctx, key, 256);
        crypt(&ctx, ESP_AES_DECRYPT, outputLength, receivedData, receivedData);
    }
    else if (cypherKey != "notUsed")
    {
        debugE("incorrect cypher key length (%d)", cypherKey.length());
    }

    Packet ret = Packet(outputLength, receivedData, true);
    debugD("Decyphered pkt : total length = %d, datalength = %d, paddingLength = %d", ret.getPktSize(), ret.getDataSize(), ret._paddingCount);
    ret.print();
    return ret;
}

void Packet::hasJustBeenSent()
{
    _sent++;
    debugD("pkt was sent %d times", _sent);
    _sentTimestamp = millis();
}

void Packet::print()
{
    uint8_t size = this->getPktSize();
    uint8_t *data = this->get();
    debugD("Pkt of length %d, protocol %d, Qos %d, Type %d, split %d, sourceID %d, destID %d, nb %d\n", this->getPktSize(), this->getProtocolVersion(), this->getQoS(), this->getType(), this->isSplit(), this->getSourceID(), this->getDestID(), this->getPktNumber());
    for (int i = 0; i < size; ++i)
    {
        debugD(" %02X", data[i]);
    }
    String asciiData = String((char *)(data + _headerSize));
    debugD("data = %s\n", asciiData.c_str());
}

uint8_t *Packet::getCyphered(uint8_t *pktCyphered, String cypherKey)
{
    uint8_t key[32];
    memcpy(key, cypherKey.c_str(), 32);
    esp_aes_context ctx;
    esp_aes_init(&ctx);
    esp_aes_setkey(&ctx, key, 256);
    crypt(&ctx, ESP_AES_ENCRYPT, _pktSize, _pkt, pktCyphered);
    return pktCyphered;
}

static uint8_t checksum(uint8_t *data, size_t size)
{
    uint8_t ret = 0;
    for (int i = 0; i < size; ++i)
    {
        ret ^= data[i];
    }
    return ret;
}

uint8_t Packet::computeChecksum()
{
    uint8_t sum = checksum(_pkt, _pktSize);
    _header[3] = sum;
    return sum;
}

bool Packet::checkIntegity()
{
    // checksum of the packet should be 0
    if (checksum(_pkt, _pktSize) != 0)
    {
        debugE("wrong checksum!");
        return false;
    }
    // packet type should be known
    if (!(getType() == Packet::DATA || getType() == Packet::PING || getType() == Packet::OTA || getType() == Packet::ACK))
    {
        debugE("Unknown packet type!");
        return false;
    }
    // packet version should be known
    if (!(getProtocolVersion() == Packet::VERSION_1))
    {
        debugE("Unknown protocol version!");
        return false;
    }

    return true;
}