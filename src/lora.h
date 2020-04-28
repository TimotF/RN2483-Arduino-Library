#ifndef LORA_H
#define LORA_H
#include "Arduino.h"
#include "rn2xx3.h"
#include "freertos/ringbuf.h"
#include "packet.h"


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
    void printPkt(Packet &pkt);
    //TODO : implement callback for received packets

private:
    static const uint8_t _maxPktSize = 235;
    static uint8_t _pktCounter;
    bool _useP2P = false;
    rn2xx3 _lora;
    bool _rxListening = false;
    std::vector<Packet> _packetsQueue;
    // TODO : 2 queues de packets, 1 pour le RX, 1 pour le TX

    bool formatData(const uint8_t *data, uint16_t dataSize);
    bool removePkt(Packet &pkt);
};

#endif