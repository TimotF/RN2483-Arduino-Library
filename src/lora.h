#ifndef LORA_H
#define LORA_H
#include "Arduino.h"
#include "rn2xx3.h"
#include "packet.h"
#include "loraClients.h"

/* Minimum listening time for each spreading factor. Time is defined as 
time on air for a 250 bytes payload + 20% margin. Time on air can be 
calculated on this site : https://www.loratools.nl/#/airtime */
#define MIN_LISTENING_TIME_SF7 470
#define MIN_LISTENING_TIME_SF8 850
#define MIN_LISTENING_TIME_SF9 1500
#define MIN_LISTENING_TIME_SF10 2700
#define MIN_LISTENING_TIME_SF11 5900
#define MIN_LISTENING_TIME_SF12 10500
#define MIN_TIME_BTW_PKT 250 /* minimum time between sending two packets */
#define MAX_TX_TIME 5000     /* maximum time the module is allowed to stay in TX mode */
#define MAX_PKTS_IN_QUEUE 25 /* maximum number of packets in queue */

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
    LoRa(Stream &serial, int loraResetPin = -1) : _lora(serial, loraResetPin) {} /* LoRa constructor, requires the serial stream to use */

    void setMac(uint8_t *macAddress) { _loraClients.setHostMac(macAddress); } /* set host mac */
    void setID(uint8_t id = DEFAULT_ID) { _loraClients.setHostID(id); }       /* set host ID */

    bool begin(String sf = "sf7", const bool &useP2P = false); /* method to initialize the LoRa object. To call after declaration of the LoRa object */
    void loop();                                               /* LoRa loop, to call in a regular basis */

    bool send(const uint8_t *data, uint16_t dataSize, Packet::PACKET_TYPE pktType,
              uint8_t destID = BROADCAST_ID, bool ack = false, bool canBeDropped = false); /* method to send data via LoRa */

    void setRcvCallback(void (*rcvCallback)(uint8_t *payload, size_t size, Packet::PACKET_TYPE pktType)); /* set receive callback to call when a packet was just received */

    bool useCyphering(String key);

    void stopCyphering() { _useCyphering = false; }

    int getNbPktInQueue() { return _loraClients.TXsize(); } /* get the number of packets currently stored in the queue */

    int getSNR() { return _snr; }

    void attachLed(String gpio) { _loraLedGpio = gpio; }
    String toggleLed(); /* remove a packet from the packet queue based on its packet number */

private:
    rn2xx3 _lora;              /* lora object to handle communication with lora module */
    bool _rxListening = false; /* tells if module is currently listening to incoming messages */
    bool _useP2P = false;      /* decides if we want to use P2P communication, or LoRaWan */
    LoraStates _state = INIT;  /* LoRa state used for the state machine */
    String _sf;                /* spreading factor to use */
    int _snr = -128;           /* SNR for last received packet */
    String _loraLedGpio = "";  /* String containing GPIO number of the attached status led */
    int _ledState = 0;         /* state of the attached led if any */

    LoRaClients _loraClients; /* TX / RX packets handler */

    void (*_rcvCallback)(uint8_t *payload, size_t size, Packet::PACKET_TYPE pktType); /* callback function to call when a new packet was received */

    static const uint8_t _maxPktSize = 230; /* maximum size of a packet */

    bool _useCyphering = false; /* tells if we are cyphering the packets we send through lora*/
    String _cypherKey;

    uint32_t _lastPktReicvTime = 0;  /* timestamp of the moment a new packet was received  */
    uint32_t _randAdditionalLisTime; /* An additional random number of milliseconds between 0 and MIN_LISTENING_TIME to wait before going out of RX state */
    uint32_t _minListeningTime;      /* minimum listening time */

    bool formatData(const uint8_t *data, uint16_t dataSize, Packet::PACKET_TYPE pktType,
                    uint8_t destID, bool ack, bool split = false); /* create a packet from the data passed in param, and put the new packet in the queue */
};

#endif