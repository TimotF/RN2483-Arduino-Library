#ifndef LORA_H
#define LORA_H
#include "Arduino.h"
#include "rn2xx3.h"
#include "freertos/ringbuf.h"
#include "packet.h"

#define LOG(f_, ...)                              \
    {                                             \
        Serial.printf("[LoRa] [%ld] ", millis()); \
        Serial.printf((f_), ##__VA_ARGS__);       \
        Serial.printf("\n");                      \
    }



class LoRa
{
public:
    LoRa(Stream &serial, const bool &useP2P = false) : _lora(serial)
    {
        init(useP2P);
        _bufHandle = xRingbufferCreate(1028, RINGBUF_TYPE_ALLOWSPLIT);
        if (_bufHandle == NULL)
        {
            LOG("Failed to create ring buffer\n");
        }
    }
    bool init(const bool &useP2P);
    bool sendData(uint8_t *, bool ack = false);
    bool receivedData();
    void loop();
    //TODO : implement callback for received packets

private:
    bool _useP2P = false;
    rn2xx3 _lora;
    RingbufHandle_t _bufHandle;
    // TODO : 2 queues de packets, 1 pour le RX, 1 pour le TX

    bool sendPktToBuf(const Packet &pkt);
};

#endif