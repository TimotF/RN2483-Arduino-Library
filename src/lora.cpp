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

bool LoRa::send(const uint8_t *data, uint16_t dataSize, Packet::PACKET_TYPE pktType,
                uint8_t destID, bool ack, bool canBeDropped)
{

    if ((data == NULL) | (dataSize == 0))
        return false;

    if (pktType == Packet::DATA && getNbPktInQueue() > MAX_PKTS_IN_QUEUE && canBeDropped) // we dont want to refuse any payload other than DATA
        return false;

    return formatData(data, dataSize, pktType, destID, ack);
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

            debugD("[q=%d][IN][Type=%d][sourceID=%d][destID=%d] pkt nb = %d", getNbPktInQueue(), pkt.getType(), pkt.getSourceID(), pkt.getDestID(), pkt.getPktNumber());

            _loraClients.newPktFromClient(pkt, _snr);

            if (_loraClients.needToSendACK() ||
                _loraClients.IDpktpending())
            {
                _state = GO_TO_TX;
                debugD("ACK / ID required, going into TX mode");
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
        ledOn();
        _state = GO_TO_TX;
        Packet pkt;
        if (!_loraClients.getNextTXPacket(&pkt))
        {
            debugE("WTF? in TX mode but no packet to send!");
            ledOff();
            break;
        }

        debugD("[q=%d][OUT][Type=%d][sourceID=%d][destID=%d] pkt nb = %d", getNbPktInQueue(), pkt.getType(), pkt.getSourceID(), pkt.getDestID(), pkt.getPktNumber());

        size_t pktSize = pkt.getPktSize();
        uint8_t pktbuf[pktSize];
        TX_RETURN_TYPE ret = _lora.txBytes(_useCyphering ? pkt.getCyphered(pktbuf, _cypherKey) : pkt.get(), pktSize);
        switch (ret)
        {
        case TX_FAIL:
        {
            debugE("TX FAIL");
            begin(_sf, _useP2P);
            break;
        }
        case TX_SUCCESS:
        {
            _loraClients.markPktAsSent(pkt);
            debugD("Pkt marked as sent");
            if (pkt.getQoS() == Packet::AT_LEAST_ONE_PACKET)
            {
                debugD("Pkt sent require ACK, going into RX mode");
                _state = GO_TO_RX;
            }
            break;
        }
        default:
        {
            debugW("WTF! received code %d", ret);
        }
        }
        ledOff();
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
            if (millis() - _loraClients.getLastSentPktTs() > _minListeningTime)
                _state = GO_TO_RX;
        }
        else if (_loraClients.getNextTXPacket(nullptr)) /* Cannot use mutex here */
        {
            if (_rxListening)
            {
                _rxListening = false;
                _lora.stopPassiveRxP2P();
            }
            if (millis() - _loraClients.getLastSentPktTs() > MIN_TIME_BTW_PKT)
                _state = LORA_TX;
        }
        else if (millis() - _loraClients.getLastSentPktTs() > _minListeningTime)
            _state = GO_TO_RX;

        break;
    }
    }
}

bool LoRa::formatData(const uint8_t *data, uint16_t dataSize, Packet::PACKET_TYPE pktType,
                      uint8_t destID, bool ack, bool split)
{
    if (dataSize <= _maxPktSize) /* data can be stored in one packet */
    {
        Packet pkt((size_t)dataSize, data);
        pkt.setType(pktType);
        pkt.setSplit(split);
        if (ack)
            pkt.setQoS(Packet::AT_LEAST_ONE_PACKET);
        else
            pkt.setQoS(Packet::ONE_PACKET_AT_MOST);

        debugD("Put new pkt in sending queue");
        _loraClients.addTXPacket(pkt, destID);

        return true;
    }
    else /* data must be split into multiple packets */
    {

        debugD("data size is %d, must be split into several packets...", dataSize);
        for (int dataIndex = 0; dataIndex < dataSize; dataIndex += _maxPktSize)
        {
            if (dataSize - dataIndex > _maxPktSize)
                formatData(data + dataIndex, _maxPktSize, pktType, destID, true, true);
            else
                formatData(data + dataIndex, dataSize - dataIndex, pktType, destID, true, false);
        }
        return true;
    }
}

bool LoRa::useCyphering(String key)
{
    if (key.length() < 32)
    {
        debugE("cannot use a key of length %d", key.length());
        return false;
    }
    if (key.length() > 32)
    {
        debugW("cyphering key too long (%d), cropping it", key.length());
        _cypherKey = String(key.substring(0, 32));
    }
    else
    {
        _cypherKey = String(key);
    }
    _useCyphering = true;
    return true;
}

String LoRa::toggleLed()
{
    if (_loraLedGpio.length() > 0)
    {
        _lora.sendRawCommand("sys set pinmode " + _loraLedGpio + " digout");
        _ledState = !_ledState;
        return _lora.sendRawCommand("sys set pindig " + _loraLedGpio + " " + String(_ledState));
    }
    return "FAIL";
}

String LoRa::ledOn()
{
    if (_loraLedGpio.length() > 0)
    {
        _lora.sendRawCommand("sys set pinmode " + _loraLedGpio + " digout");
        _ledState = HIGH;
        return _lora.sendRawCommand("sys set pindig " + _loraLedGpio + " " + String(_ledState));
    }
    return "FAIL";
}

String LoRa::ledOff()
{
    if (_loraLedGpio.length() > 0)
    {
        _lora.sendRawCommand("sys set pinmode " + _loraLedGpio + " digout");
        _ledState = LOW;
        return _lora.sendRawCommand("sys set pindig " + _loraLedGpio + " " + String(_ledState));
    }
    return "FAIL";
}

void LoRa::setRcvCallback(void (*rcvCallback)(Packet pkt))
{
    _rcvCallback = rcvCallback;
    _loraClients.setRcvCallback(rcvCallback);
}