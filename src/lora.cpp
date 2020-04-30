#include "lora.h"

#if 1
#define LOG(f_, ...)                          \
  {                                           \
    Serial.printf("[LoRa] [%ld] ", millis()); \
    Serial.printf((f_), ##__VA_ARGS__);       \
    Serial.printf("\n");                      \
  }
#else
#define LOG(f_, ...)                          \
  {                                           \
    NOP(); \
  }
#endif

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

    if (getNbPktInQueue() > MAX_PKTS_IN_QUEUE)
        return false;

    return formatData(data, dataSize, ack);
}

bool LoRa::receivedData()
{
    return false;
}

void LoRa::loop()
{
    switch (_state)
    {
    case INIT:
    {
        _state = GO_TO_RX;
        break;
    }
    case LORA_RX:
    {
        /* Rx mode */
        TX_RETURN_TYPE ret = _lora.listenLoop();
        switch (ret)
        {
        case TX_WITH_RX: /* received a message */
        {
            String received = _lora.getRx(); /* get received msg */
            _lora.setPassiveRxP2P();         /* go back into rx mode */
                                             /* Then, process received msg */
            _lastPktReicvTime = millis();
            Packet pkt = Packet::buildPktFromBase16str(received);
            // printPkt(pkt);
            // LOG("[q=%d][IN][Type=%d] pkt nb = %d", getNbPktInQueue(), pkt.getType(), pkt.getPktNumber());
            //TODO : drop packets that do not belong to us
            if (pkt.getQoS() == Packet::AT_LEAST_ONE_PACKET) /* we need to send an ack */
            {
                // LOG("Received pkt requires ACK repply");
                createACK(pkt.getPktNumber());
                _state = GO_TO_TX;
            }
            if (pkt.getType() == Packet::ACK) /* received ACK, need to remove corresponding pkt */
            {
                if (pkt.getDataSize() != 0)
                {
                    LOG("[IN] Error : received ACK with payload!");
                }
                else
                {
                    // LOG("[q=%d][IN][ACK%d]", getNbPktInQueue(), pkt.getPktNumber());
                    removePkt(pkt.getPktNumber());
                }
            }
            else
            {
                // LOG("[q=%d][IN][Type=%d] pkt nb = %d", getNbPktInQueue(), pkt.getType(), pkt.getPktNumber());
                if(_lastPktReceived == pkt){
                    LOG("Received dupplicate packet!");
                }
                else if (_reicvCallback != NULL)
                    _reicvCallback(pkt.getData(), pkt.getDataSize(), pkt.getType());

                _lastPktReceived = pkt;
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

        if (millis() - _lastPktReicvTime > (MIN_LISTENING_TIME + TIME_BEFORE_RX_WINDOW))
            _state = GO_TO_TX;
        break;
    }
    case LORA_TX:
    {
        _state = GO_TO_TX;
        std::vector<Packet>::iterator pkt = hasPktToSend();
        // LOG("[q=%d][OUT][Type=%d] pkt nb = %d", getNbPktInQueue(), pkt->getType(), pkt->getPktNumber());
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
                _state = GO_TO_RX;
                break;
            }
            default:
            {
                LOG("WTF! QoS is unknown!!");
                break;
            }
            }
            _lastPktSentTime = millis();
            break;
        }
        default:
        {
            LOG("WTF! received code %d", ret);
        }
        }

        break;
    }
    case GO_TO_RX:
    {
        if (!_rxListening)
        {
            _rxListening = true;
            _lora.setPassiveRxP2P();
        }
        _lastPktReicvTime = millis();
        _state = LORA_RX;
        break;
    }
    case GO_TO_TX:
    {
        if (millis() - _lastPktReicvTime > (MIN_LISTENING_TIME + TIME_BEFORE_RX_WINDOW + MAX_TX_TIME))
        {
            if (millis() - _lastPktSentTime > TIME_BEFORE_RX_WINDOW)
                _state = GO_TO_RX;
        }
        else if (hasPktToSend() != _packetsQueue.end())
        {
            if (_rxListening)
            {
                _rxListening = false;
                _lora.stopPassiveRxP2P();
            }
            if (millis() - _lastPktSentTime > MIN_TIME_BTW_PKT)
                _state = LORA_TX;
        }
        else if (millis() - _lastPktSentTime > TIME_BEFORE_RX_WINDOW)
            _state = GO_TO_RX;

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
        // LOG("Put pkt %d in sending queue", _pktCounter - 1);
        _packetsQueue.push_back(pkt);
        return true;
    }
    else /* data must be split into multiple packets */
    {
        // TODO
        return false;
    }
}

bool LoRa::removePkt(uint8_t pktNb)
{
    // LOG("Remove pkt %d", pktNb);
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
    // Packet ack = Packet(1, &pktNb);
    Packet ack = Packet();
    ack.setPktNumber(pktNb);
    ack.setDestID(0);
    ack.setSourceID(0);
    ack.setQoS(Packet::ONE_PACKET_AT_MOST);
    ack.setProtocolVersion(Packet::VERSION_1);
    ack.setType(Packet::ACK);
    ack.setSplit(false);
    /* ACK is prioritary over other type of packets, so we 
    * must insert it just before the first packet to send that is not 
    * of the type ACK */
    std::vector<Packet>::iterator it;
    for (it = _packetsQueue.begin(); it != _packetsQueue.end(); ++it)
    {
        if (it->getType() != Packet::ACK)
            break;
    }
    _packetsQueue.insert(it, ack);
    // _packetsQueue.push_back(ack);
}

std::vector<Packet>::iterator LoRa::hasPktToSend()
{
    cleanUpPacketQueue();
    std::vector<Packet>::iterator pkt;
    for (pkt = _packetsQueue.begin(); pkt != _packetsQueue.end(); ++pkt)
    {
        if (pkt->getSent())
        {
            if (millis() - pkt->getSentTimestamp() >= pkt->getTimeout())
            {
                if (pkt->getSent() <= pkt->getMaxRetry())
                {
                    // LOG("No ack, retry sending pkt %d", pkt->getPktNumber());
                    return pkt;
                }
            }
        }
        else
            return pkt;
    }
    return pkt;
}

void LoRa::cleanUpPacketQueue()
{
    std::vector<Packet>::iterator pkt;
    for (pkt = _packetsQueue.begin(); pkt != _packetsQueue.end(); ++pkt)
    {
        if ((millis() - pkt->getSentTimestamp() >= pkt->getTimeout()) && (pkt->getSent() > pkt->getMaxRetry())) /* Too many retries, dropping packet */
        {
            LOG("Too many retries, dropping pkt %d", pkt->getPktNumber());
            _packetsQueue.erase(pkt--);
            break;
        }
    }
}