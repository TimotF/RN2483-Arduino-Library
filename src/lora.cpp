#include "lora.h"

#define LOG(f_, ...)                              \
    {                                             \
        Serial.printf("[LoRa] [%ld] ", millis()); \
        Serial.printf((f_), ##__VA_ARGS__);       \
        Serial.printf("\n");                      \
    }

uint8_t LoRa::_pktCounter = 0;

bool LoRa::init(const bool &useP2P)
{
    if (useP2P)
    {
        return _lora.initP2P();
    }
    else
    {
        return false;
    }
}

bool LoRa::sendData(const uint8_t *data, uint16_t dataSize, bool ack)
{

    if ((data == NULL) | (dataSize == 0))
        return false;

    return formatData(data, dataSize);
}

bool LoRa::receivedData()
{
    return false;
}

void LoRa::loop()
{
    if (_packetsQueue.size() > 0) /* if needs to send data */
    {
        if (_rxListening)
        {
            _rxListening = false;
            _lora.stopPassiveRxP2P();
        }
        Packet pkt = _packetsQueue[0];
        LOG("Sending packet %d", pkt.getPktNumber());
        TX_RETURN_TYPE ret = _lora.txBytes(pkt.get(), pkt.getPktSize());
        switch (ret)
        {
        case TX_FAIL:
        {
            init(_useP2P);
            break;
        }
        case TX_SUCCESS:
        {
            _packetsQueue.erase(_packetsQueue.begin());
            break;
        }
        default:
        {
            LOG("WTF! received code %d", ret);
        }
        }
    }
    else /* Rx mode */
    {
        if (!_rxListening)
        {
            _rxListening = true;
            _lora.setPassiveRxP2P();
        }
        TX_RETURN_TYPE ret = _lora.listenLoop();
        switch (ret)
        {
        case TX_WITH_RX: /* received a message */
        {
            String received = _lora.getRx(); /* get received msg */
            _lora.setPassiveRxP2P();         /* go back into rx mode */
                                             /* process received msg */
            Packet pkt = Packet::buildPktFromBase16str(received);
            printPkt(pkt);

            break;
        }
        case RADIO_LISTEN_WITHOUT_RX: /* timeout */
        {
            _lora.setPassiveRxP2P();
            break;
        }
        case NO_EVENT: /* nothing happened */
        {
            break;
        }
        case UNKNOWN_MSG: /* received a message */
        {
            _lora.setPassiveRxP2P();
            break;
        }
        default: /* should never get inside */
        {
            LOG("WTF??")
            _lora.setPassiveRxP2P();
            break;
        }
        }
    }
}

bool LoRa::formatData(const uint8_t *data, uint16_t dataSize)
{
    if (dataSize < _maxPktSize) /* data can be stored in one packet */
    {
        Packet pkt((size_t)dataSize, data);
        pkt.setPktNumber(_pktCounter++);
        pkt.setDestID(0);
        pkt.setSourceID(0);
        pkt.setQoS(Packet::ONE_PACKET_AT_MOST);
        pkt.setProtocolVersion(Packet::VERSION_1);
        pkt.setType(Packet::DATA);
        pkt.setSplit(false);
        LOG("Put pakt %d in sending queue", _pktCounter - 1);
        _packetsQueue.push_back(pkt);
        return true;
    }
    else /* data must be split into multiple packets */
    {
        // TODO
        return false;
    }
}

void LoRa::printPkt(Packet &pkt)
{
    uint8_t size = pkt.getPktSize();
    uint8_t *data = pkt.get();
    Serial.printf("Pkt of length %d, protocol %d, Qos %d, Type %d, split %d, sourceID %d, destID %d, nb %d\n", pkt.getPktSize(), pkt.getProtocolVersion(), pkt.getQoS(), pkt.getType(), pkt.isSplit(), pkt.getSourceID(), pkt.getDestID(), pkt.getPktNumber());
    for (int i = 0; i < size; ++i)
    {
        Serial.printf(" %02X", data[i]);
    }
    Serial.printf("\nfree heap : %d\n", ESP.getFreeHeap());
}