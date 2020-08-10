#include "lora.h"
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
#ifdef DEBUG_SERIAL_LORA
#if DEBUG_SERIAL_LORA >= 4
#define debugA(fmt, ...) Serial.printf("[A][C%d][%ld][%s:%d] %s: \t" fmt "\n", xPortGetCoreID(), millis(), __FILE__, __LINE__, __func__, ##__VA_ARGS__)
#define debugP(fmt, ...) Serial.printf("[P][C%d][%ld][%s:%d] %s: \t" fmt "\n", xPortGetCoreID(), millis(), __FILE__, __LINE__, __func__, ##__VA_ARGS__)
#define debugV(fmt, ...) Serial.printf("[V][C%d][%ld][%s:%d] %s: \t" fmt "\n", xPortGetCoreID(), millis(), __FILE__, __LINE__, __func__, ##__VA_ARGS__)
#else
#define debugA(fmt, ...)
#define debugP(fmt, ...)
#define debugV(fmt, ...)
#endif
#if DEBUG_SERIAL_LORA >= 3
#define debugD(fmt, ...) Serial.printf("[D][C%d][%ld][%s:%d] %s: \t" fmt "\n", xPortGetCoreID(), millis(), __FILE__, __LINE__, __func__, ##__VA_ARGS__)
#else
#define debugD(fmt, ...)
#endif
#if DEBUG_SERIAL_LORA >= 2
#define debugI(fmt, ...) Serial.printf("[I][C%d][%ld][%s:%d] %s: \t" fmt "\n", xPortGetCoreID(), millis(), __FILE__, __LINE__, __func__, ##__VA_ARGS__)
#else
#define debugI(fmt, ...)
#endif
#if DEBUG_SERIAL_LORA >= 1
#define debugW(fmt, ...) Serial.printf("[W][C%d][%ld][%s:%d] %s: \t" fmt "\n", xPortGetCoreID(), millis(), __FILE__, __LINE__, __func__, ##__VA_ARGS__)
#else
#define debugW(fmt, ...)
#endif
#if DEBUG_SERIAL_LORA >= 0
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
        debugE("Error! unknown sf! Using SF7 as default value");
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
            toggleLed();
            _lora.setPassiveRxP2P(); /* go back into rx mode */
                                     /* Then, process received msg */
            _lastPktReicvTime = millis();

            debugV("received msg with snr %d", _snr);

            Packet pkt;
            if (_useCyphering)
                pkt = Packet::buildPktFromBase16str(received, _cypherKey);
            else
                pkt = Packet::buildPktFromBase16str(received);

            if (!pkt.checkIntegity())
            {
                debugE("Integrity error in received packet");
                break;
            }

            if (pkt.getQoS() == Packet::AT_LEAST_ONE_PACKET) /* we need to send an ack */
            {
                debugV("Received pkt requires ACK repply");
                createACK(pkt.getPktNumber());
                _state = GO_TO_TX;
            }
            if (pkt.getType() == Packet::ACK) /* received ACK, need to remove corresponding pkt */
            {
                if (pkt.getDataSize() != 0)
                {
                    debugE("[IN] Error : received ACK with payload!");
                }
                else
                {
                    debugV("[q=%d][IN][ACK%d]", getNbPktInQueue(), pkt.getPktNumber());
                    removePkt(pkt.getPktNumber());
                }
            }
            else
            {
                debugV("[q=%d][IN][Type=%d] pkt nb = %d", getNbPktInQueue(), pkt.getType(), pkt.getPktNumber());
                if (_lastPktReceived == pkt)
                {
                    debugW("Received dupplicate packet!");
                }
                else
                {
                    if (pkt.isSplit())
                    {
                        debugI("received split packet!");
                        _splitPktQueue.push_back(pkt);
                        _hasSplitPacketsInBuffer = true;
                    }
                    else if (_hasSplitPacketsInBuffer)
                    {
                        debugI("received last split packet!");
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
                                debugW("met packet type inconsistency when rebuilding split packet...");
                            }
                            if (it->getPktNumber() != (pktNb + 1))
                            {
                                debugW("pkt number inconsistency : current nb is %d while last nb is %d", it->getPktNumber(), pktNb);
                            }

                            pktType = it->getType();
                            pktNb = it->getPktNumber();
                        }
                        debugD("total datasize for split packet is %d", dataSize);
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
                            debugD("Total retrieved payload : %d bytes", dataIndex);
                            _reicvCallback(data, dataIndex, pktType);
                        }
                    }
                    else if (_reicvCallback != NULL)
                    {
                        if (_lastPktReceived.getPktNumber() != (pkt.getPktNumber() - 1))
                            debugW("Missing packet !!!");
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
            debugE("WTF??");
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
        debugV("[q=%d][OUT][Type=%d] pkt nb = %d", getNbPktInQueue(), pkt->getType(), pkt->getPktNumber());
        if (pkt == _packetsQueue.end())
        {
            debugE("trying to send non existing packet!");
            break;
        }
        pkt->computeChecksum();
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
                debugW("WTF! QoS is unknown!!");
                break;
            }
            }
            _lastPktSentTime = millis();
            break;
        }
        default:
        {
            debugW("WTF! received code %d", ret);
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
        debugD("Put pkt %d in sending queue", _pktCounter - 1);
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
        debugD("Added packet of size %d to the stack", dataSize);
        return true;
    }
    else /* data must be split into multiple packets */
    {
        // if (dataSize > 2048)
        // {
        //     debugE("cannot send this packet because it has more than 2048 bytes in it !");
        //     return false;
        // }
        debugD("data size is %d, must be split into several packets...", dataSize);
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
                    debugW("No ack, retry sending pkt %d", pkt->getPktNumber());
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
            debugW("Too many retries, dropping pkt %d", pkt->getPktNumber());
            _packetsQueue.erase(pkt--);
            break;
        }
    }
    xSemaphoreGive(_pktQueueMutex);
}

void LoRa::useCyphering(String key)
{
    if (key.length() < 32)
    {
        debugE("cannot use a key of length %d", key.length());
        return;
    }
    if (key.length() > 32)
    {
        debugW("cyphering key too long (%d), cropping it", key.length());
        _cypherKey = String(key.substring(0,32));
    }
    else
    {
        _cypherKey = String(key);
    }
    _useCyphering = true;
}

void LoRa::toggleLed()
{
    if (_loraLedGpio.length() > 0)
    {
        _lora.sendRawCommand("sys set pinmode " + _loraLedGpio + " digout");
        _ledState = !_ledState;
        _lora.sendRawCommand("sys set pindig " + _loraLedGpio + " " + String(_ledState));
    }
}