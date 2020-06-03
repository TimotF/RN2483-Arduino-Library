#include "lora.h"

#if 1
#define LOG(f_, ...)                              \
    {                                             \
        Serial.printf("[LoRa] [%ld] ", millis()); \
        Serial.printf((f_), ##__VA_ARGS__);       \
        Serial.printf("\n");                      \
    }
#else
#define LOG(f_, ...) \
    {                \
        NOP();       \
    }
#endif

uint8_t LoRa::_pktCounter = 0;

bool LoRa::begin(String sf, const bool &useP2P)
{
    _sf = sf;
    if (_sf == "sf7")
    {
        _minListeningTime = MIN_LISTENING_TIME_SF7;
    }
    else if (_sf == "sf8")
    {
        _minListeningTime = MIN_LISTENING_TIME_SF8;
    }
    else if (_sf == "sf9")
    {
        _minListeningTime = MIN_LISTENING_TIME_SF9;
    }
    else if (_sf == "sf10")
    {
        _minListeningTime = MIN_LISTENING_TIME_SF10;
    }
    else if (_sf == "sf11")
    {
        _minListeningTime = MIN_LISTENING_TIME_SF11;
    }
    else if (_sf == "sf12")
    {
        _minListeningTime = MIN_LISTENING_TIME_SF12;
    }
    else
    {
        LOG("Error! unknown sf! Using SF7 as default value");
        _sf = "sf7";
        _minListeningTime = MIN_LISTENING_TIME_SF7;
    }
    if (useP2P)
    {
        return _lora.initP2P(sf);
    }
    else
    {
        return false;
    }
}

bool LoRa::send(const uint8_t *data, uint16_t dataSize, Packet::PACKET_TYPE pktType, bool ack)
{

    if ((data == NULL) | (dataSize == 0))
        return false;

    if (pktType == Packet::DATA && getNbPktInQueue() > MAX_PKTS_IN_QUEUE) // we dont want to refuse any payload other than DATA
        return false;

    return formatData(data, dataSize, pktType, ack);
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
            _snr = _lora.getSNR();           /* update SNR */
            _lora.setPassiveRxP2P();         /* go back into rx mode */
                                             /* Then, process received msg */
            _lastPktReicvTime = millis();

            LOG("received msg with snr %d", _snr);

            Packet pkt;
            if (_useCyphering)
                pkt = Packet::buildPktFromBase16str(received, _cypherKey);
            else
                pkt = Packet::buildPktFromBase16str(received);
            // printPkt(pkt);
            // LOG("[q=%d][IN][Type=%d] pkt nb = %d", getNbPktInQueue(), pkt.getType(), pkt.getPktNumber());
            //TODO : drop packets that do not belong to us

            if (!(pkt.getType() == Packet::DATA || pkt.getType() == Packet::PING || pkt.getType() == Packet::OTA || pkt.getType() == Packet::ACK))
            {
                LOG("Unknown packet type!");
                break;
            }

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
                if (_lastPktReceived == pkt)
                {
                    LOG("Received dupplicate packet!");
                }
                else
                {
                    if (pkt.isSplit())
                    {
                        LOG("received split packet!");
                        _splitPktQueue.push_back(pkt);
                        _hasSplitPacketsInBuffer = true;
                    }
                    else if (_hasSplitPacketsInBuffer)
                    {
                        LOG("received last split packet!");
                        _splitPktQueue.push_back(pkt);
                        std::vector<Packet>::iterator it = _splitPktQueue.begin();
                        size_t dataSize = 0;
                        Packet::PACKET_TYPE pktType = it->getType();
                        uint8_t pktNb = it->getPktNumber() - 1;
                        for (it = _splitPktQueue.begin(); it != _splitPktQueue.end(); ++it)
                        {
                            dataSize += it->getDataSize();
                            if (it->getType() != pktType)
                            {
                                LOG("Warning! met packet type inconsistency when rebuilding split packet...");
                            }
                            if (it->getPktNumber() != (pktNb + 1))
                            {
                                LOG("Warning! pkt number inconsistency : current nb is %d while last nb is %d", it->getPktNumber(), pktNb);
                            }

                            pktType = it->getType();
                            pktNb = it->getPktNumber();
                        }
                        LOG("total datasize for split packet is %d", dataSize);
                        uint8_t data[dataSize];
                        size_t dataIndex = 0;
                        for (it = _splitPktQueue.begin(); it != _splitPktQueue.end(); ++it)
                        {
                            memcpy(data + dataIndex, it->getData(), it->getDataSize());
                            dataIndex += it->getDataSize();
                        }
                        _splitPktQueue.clear();
                        _hasSplitPacketsInBuffer = false;
                        if (_reicvCallback != NULL)
                        {
                            LOG("Total retrieved payload : %d bytes", dataIndex);
                            _reicvCallback(data, dataIndex, pktType);
                        }
                    }
                    else if (_reicvCallback != NULL)
                    {
                        if (_lastPktReceived.getPktNumber() != (pkt.getPktNumber() - 1))
                            LOG("Missing packet !!!");
                        _reicvCallback(pkt.getData(), pkt.getDataSize(), pkt.getType());
                    }
                }
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

        if (millis() - _lastPktReicvTime > (_randAdditionalLisTime + (2 * _minListeningTime) + MIN_TIME_BTW_PKT))
        {
            _state = GO_TO_TX;
            _lastPktReicvTime = millis();
        }
        break;
    }
    case LORA_TX:
    {
        _state = GO_TO_TX;
        std::vector<Packet>::iterator pkt = hasPktToSend();
        // LOG("[q=%d][OUT][Type=%d] pkt nb = %d", getNbPktInQueue(), pkt->getType(), pkt->getPktNumber());
        if (pkt == _packetsQueue.end())
        {
            LOG("ERROR : trying to send non existing packet!");
            break;
        }
        size_t pktSize = pkt->getPktSize();
        uint8_t pktbuf[pktSize];
        TX_RETURN_TYPE ret = _lora.txBytes(_useCyphering ? pkt->getCyphered(pktbuf, _cypherKey) : pkt->get(), pktSize);
        switch (ret)
        {
        case TX_FAIL:
        {
            begin(_sf, _useP2P);
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
        _randAdditionalLisTime = random(_minListeningTime);
        _state = LORA_RX;
        break;
    }
    case GO_TO_TX:
    {
        if (millis() - _lastPktReicvTime > (_minListeningTime + MAX_TX_TIME))
        {
            if (millis() - _lastPktSentTime > _minListeningTime)
                _state = GO_TO_RX;
        }
        else if (hasPktToSend() != _packetsQueue.end()) /* Cannot use mutex here */
        {
            if (_rxListening)
            {
                _rxListening = false;
                _lora.stopPassiveRxP2P();
            }
            if (millis() - _lastPktSentTime > MIN_TIME_BTW_PKT)
                _state = LORA_TX;
        }
        else if (millis() - _lastPktSentTime > _minListeningTime)
            _state = GO_TO_RX;

        break;
    }
    }
}

bool LoRa::formatData(const uint8_t *data, uint16_t dataSize, Packet::PACKET_TYPE pktType, bool ack, bool split)
{
    if (dataSize <= _maxPktSize) /* data can be stored in one packet */
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
        pkt.setType(pktType);
        pkt.setSplit(split);
        // LOG("Put pkt %d in sending queue", _pktCounter - 1);
        xSemaphoreTake(_pktQueueMutex, portMAX_DELAY);
        if (pktType == Packet::OTA)
        {
            std::vector<Packet>::iterator it;
            for (it = _packetsQueue.begin(); it != _packetsQueue.end(); ++it)
            {
                if (it->getQoS() != Packet::ONE_PACKET_AT_MOST)
                    break;
            }
            _packetsQueue.insert(it, pkt);
        }
        else
        {
            _packetsQueue.push_back(pkt);
        }
        xSemaphoreGive(_pktQueueMutex);
        LOG("Added packet of size %d to the stack", dataSize);
        return true;
    }
    else /* data must be split into multiple packets */
    {
        // if (dataSize > 2048)
        // {
        //     LOG("cannot send this packet because it has more than 2048 bytes in it !");
        //     return false;
        // }
        LOG("data size is %d, must be split into several packets...", dataSize);
        for (int dataIndex = 0; dataIndex < dataSize; dataIndex += _maxPktSize)
        {
            if (dataSize - dataIndex > _maxPktSize)
                formatData(data + dataIndex, _maxPktSize, pktType, true, true);
            else
                formatData(data + dataIndex, dataSize - dataIndex, pktType, true, false);
        }
        return true;
    }
}

bool LoRa::removePkt(uint8_t pktNb)
{
    xSemaphoreTake(_pktQueueMutex, portMAX_DELAY);
    std::vector<Packet>::iterator it;
    for (it = _packetsQueue.begin(); it != _packetsQueue.end(); ++it)
    {
        if (it->getPktNumber() == pktNb)
        {
            _packetsQueue.erase(it);
            xSemaphoreGive(_pktQueueMutex);
            return true;
        }
    }
    xSemaphoreGive(_pktQueueMutex);
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
    xSemaphoreTake(_pktQueueMutex, portMAX_DELAY);
    std::vector<Packet>::iterator it;
    for (it = _packetsQueue.begin(); it != _packetsQueue.end(); ++it)
    {
        if (it->getType() != Packet::ACK)
            break;
    }
    _packetsQueue.insert(it, ack);
    xSemaphoreGive(_pktQueueMutex);
}

std::vector<Packet>::iterator LoRa::hasPktToSend()
{
    cleanUpPacketQueue();
    xSemaphoreTake(_pktQueueMutex, portMAX_DELAY);
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
                    xSemaphoreGive(_pktQueueMutex);
                    return pkt;
                }
            }
        }
        else
        {
            xSemaphoreGive(_pktQueueMutex);
            return pkt;
        }
    }
    xSemaphoreGive(_pktQueueMutex);
    return pkt;
}

void LoRa::cleanUpPacketQueue()
{
    xSemaphoreTake(_pktQueueMutex, portMAX_DELAY);
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
    xSemaphoreGive(_pktQueueMutex);
}

void LoRa::useCyphering(String key)
{
    if (key.length() != 32)
    {
        LOG("cannot use a key of length %d", key.length());
        return;
    }
    _cypherKey = String(key);
    _useCyphering = true;
}