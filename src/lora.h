#ifndef LORA_H
#define LORA_H
#include "Arduino.h"
#include "rn2xx3.h"
#include "packet.h"

#define TIME_BEFORE_RX_WINDOW 1000 /* time to wait before going from transmit mode to listening mode */
#define MIN_LISTENING_TIME 1000    /* minimum listening time */
#define MIN_TIME_BTW_PKT 250       /* minimum time between sending two packets */
#define MAX_TX_TIME 5000           /* maximum time the module is allowed to stay in TX mode */
#define MAX_PKTS_IN_QUEUE 25       /* maximum number of packets in queue */

enum LoraStates /* LoRa States difinition */
{
    INIT,     /* LoRa init state */
    LORA_RX,  /* LoRa receive state */
    LORA_TX,  /* LoRa transmit state */
    GO_TO_RX, /* LoRa transition state to receive state */
    GO_TO_TX  /* LoRa transition state to transmit state */
};

class LoRa
{
public:
    LoRa(Stream &serial) : _lora(serial) {} /* LoRa constructor, requires the serial stream to use */
    bool begin(const bool &useP2P = false); /* method to initialize the LoRa object. To call after declaration of the LoRa object */
    void loop();                            /* LoRa loop, to call in a regular basis */

    bool send(const uint8_t *data, uint16_t dataSize, Packet::PACKET_TYPE pktType, bool ack = false);                                             /* method to send data via LoRa */
    void setReicvCallback(void (*reicvCallback)(uint8_t *payload, uint8_t size, Packet::PACKET_TYPE pktType)) { _reicvCallback = reicvCallback; } /* set receive callback to call when a packet was just received */

    int getNbPktInQueue() /* get the number of packets currently stored in the queue */
    {
        xSemaphoreTake(_pktQueueMutex, portMAX_DELAY); /* take queue mutex */
        size_t ret = _packetsQueue.size();             /* get the size of the vector */
        xSemaphoreGive(_pktQueueMutex);                /* release mutex */
        return ret;
    }

private:
    rn2xx3 _lora;              /* lora object to handle communication with lora module */
    bool _rxListening = false; /* tells if module is currently listening to incoming messages */
    bool _useP2P = false;      /* decides if we want to use P2P communication, or LoRaWan */
    LoraStates _state = INIT;  /* LoRa state used for the state machine */

    static const uint8_t _maxPktSize = 235;                     /* maximum size of a packet */
    static uint8_t _pktCounter;                                 /* packet counter to set the packet number for each new packet */
    SemaphoreHandle_t _pktQueueMutex = xSemaphoreCreateMutex(); /* Mutex to protect _packetsQueue */
    std::vector<Packet> _packetsQueue;                          /* packets queue to store packets that need to be transmitted */
    uint32_t _lastPktSentTime = 0;                              /* timestamp of the moment the last packet was sent */

    uint32_t _lastPktReicvTime = 0;                                                      /* timestamp of the moment a new packet was received  */
    void (*_reicvCallback)(uint8_t *payload, uint8_t size, Packet::PACKET_TYPE pktType); /* callback function to call when a new packet was received */
    Packet _lastPktReceived;                                                             /* a copy of the last packet received */

    std::vector<Packet>::iterator hasPktToSend();                                                   /* returns an iterator of the next packet to send. If none, return end of queue */
    void cleanUpPacketQueue();                                                                      /* Check and removes the packets in the queue that were sent too many times */
    bool formatData(const uint8_t *data, uint16_t dataSize, Packet::PACKET_TYPE pktType, bool ack); /* create a packet from the data passed in param, and put the new packet in the queue */
    void createACK(const uint8_t pktNb);                                                            /* create an ACK packet and put it in the queue */
    bool removePkt(uint8_t pktNb);                                                                  /* remove a packet from the packet queue based on its packet number */
};

#endif