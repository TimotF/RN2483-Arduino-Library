#ifndef LORA_H
#define LORA_H
#include "Arduino.h"
#include "rn2xx3.h"
#include "packet.h"

#define TIME_BEFORE_RX_WINDOW 1000
#define MIN_LISTENING_TIME 1000
#define MIN_TIME_BTW_PKT 250
#define MAX_TX_TIME 5000
#define MAX_PKTS_IN_QUEUE 25

enum LoraStates
{
    INIT,
    LORA_RX,
    LORA_TX,
    GO_TO_RX,
    GO_TO_TX
};

class LoRa
{
public:
    LoRa(Stream &serial, const bool &useP2P = false) : _lora(serial)
    {
        init(useP2P);
    }
    bool init(const bool &useP2P);
    bool sendData(const uint8_t *data, uint16_t dataSize, bool ack = false);
    bool receivedData();
    void loop();
    int getNbPktInQueue() { return _packetsQueue.size(); }
    void setReicvCallback(void (*reicvCallback)(uint8_t *payload, uint8_t size, Packet::PACKET_TYPE pktType)) { _reicvCallback = reicvCallback; }

private:
    static const uint8_t _maxPktSize = 235;
    static uint8_t _pktCounter;
    bool _useP2P = false;
    rn2xx3 _lora;
    bool _rxListening = false;
    std::vector<Packet> _packetsQueue;
    uint32_t _lastPktSentTime = 0;
    uint32_t _lastPktReicvTime = 0;
    LoraStates _state = INIT;
    void (*_reicvCallback)(uint8_t *payload, uint8_t size, Packet::PACKET_TYPE pktType);
    Packet _lastPktReceived;

    bool formatData(const uint8_t *data, uint16_t dataSize, bool ack);
    void createACK(const uint8_t pktNb);
    bool removePkt(uint8_t pktNb);
    std::vector<Packet>::iterator hasPktToSend();
    void cleanUpPacketQueue();
};

#endif