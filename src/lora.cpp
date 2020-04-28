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

    return formatData(data, dataSize, ack);
}

bool LoRa::receivedData()
{
    return false;
}

void LoRa::loop()
{
    if (_packetsQueue.size() > 0) /* if needs to send data */
    {
        bool hasToSendPkt = false;
        std::vector<Packet>::iterator pkt;
        for (pkt = _packetsQueue.begin(); pkt != _packetsQueue.end(); ++pkt)
        {
            if (pkt->getSent())
            {
                if (millis() - pkt->getSentTimestamp() < pkt->getTimeout())
                {
                    continue;
                }
                else /* timeout ! */
                {
                    if (pkt->getSent() > pkt->getMaxRetry()) /* Too many retries, dropping packet */
                    {
                        LOG("Too many retries, dropping pkt %d", pkt->getPktNumber());
                        removePkt(pkt->getPktNumber());
                        break;
                    }
                    else
                    {
                        LOG("No ack, retry sending pkt %d", pkt->getPktNumber());
                        // pkt = &p;
                        hasToSendPkt = true;
                        break;
                    }
                }
            }
            else
            {
                // pkt = &p;
                hasToSendPkt = true;
                break;
            }
        }
        if (hasToSendPkt)
        {
            if (_rxListening)
            {
                _rxListening = false;
                _lora.stopPassiveRxP2P();
            }
            LOG("Sending packet %d", pkt->getPktNumber());
            TX_RETURN_TYPE ret = _lora.txBytes(pkt->get(), pkt->getPktSize());
            switch (ret)
            {
            case TX_FAIL:
            {
                init(_useP2P);
                break;
            }
            case TX_SUCCESS:
            {
                switch (pkt->getQoS())
                {
                case Packet::ONE_PACKET_AT_MOST:
                {
                    removePkt(pkt->getPktNumber());
                    break;
                }
                case Packet::AT_LEAST_ONE_PACKET:
                {
                    pkt->hasJustBeenSent();
                    break;
                }
                default:
                {
                    LOG("WTF! QoS is unknown!!");
                    break;
                }
                }

                delay(10);
                break;
            }
            default:
            {
                LOG("WTF! received code %d", ret);
            }
            }
        }
    }
    /* Rx mode */

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
                                         /* Then, process received msg */
        Packet pkt = Packet::buildPktFromBase16str(received);
        printPkt(pkt);
        //TODO : drop packets that do not belong to us
        if (pkt.getQoS() == Packet::AT_LEAST_ONE_PACKET) /* we need to send an ack */
        {
            LOG("Received pkt requires ACK repply");
            createACK(pkt.getPktNumber());
        }
        if (pkt.getType() == Packet::ACK) /* received ACK, need to remove corresponding pkt */
        {
            if (pkt.getDataSize() != 1)
            {
                LOG("Error : received ACK with more than 1 byte of payload!");
            }
            else
            {
                uint8_t *data = pkt.getData();
                LOG("received ACK, removing packet %d from queue", *data);
                removePkt(*data);
            }
        }

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

bool LoRa::formatData(const uint8_t *data, uint16_t dataSize, bool ack)
{
    if (dataSize < _maxPktSize) /* data can be stored in one packet */
    {
        Packet pkt((size_t)dataSize, data);
        pkt.setPktNumber(_pktCounter++);
        pkt.setDestID(0);
        pkt.setSourceID(0);
        if (ack)
            pkt.setQoS(Packet::AT_LEAST_ONE_PACKET);
        else
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

bool LoRa::removePkt(uint8_t pktNb)
{
    LOG("Remove pkt %d", pktNb);
    std::vector<Packet>::iterator it;
    for (it = _packetsQueue.begin(); it != _packetsQueue.end(); ++it)
    {
        if (it->getPktNumber() == pktNb)
        {
            _packetsQueue.erase(it);
            return true;
        }
    }
    return false;
}

void LoRa::createACK(const uint8_t pktNb)
{
    Packet ack = Packet(1, &pktNb);
    ack.setPktNumber(_pktCounter++);
    ack.setDestID(0);
    ack.setSourceID(0);
    ack.setQoS(Packet::ONE_PACKET_AT_MOST);
    ack.setProtocolVersion(Packet::VERSION_1);
    ack.setType(Packet::ACK);
    ack.setSplit(false);
    LOG("Added ACK for pkt %d in sending queue", pktNb);
    _packetsQueue.insert(_packetsQueue.begin(), ack);
}