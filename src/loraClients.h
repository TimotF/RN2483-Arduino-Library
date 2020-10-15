#ifndef LORA_CLIENTS_H
#define LORA_CLIENTS_H

#include "Arduino.h"
#include "packet.h"

#define GATEWAY_ID 0x00
#define MAX_CLIENT_ID 0x0D
#define BROADCAST_ID 0x0E
#define DEFAULT_ID 0x0F

struct LoRaClient
{
    uint8_t _macAddress[6];
    uint32_t _lastSeenTimestamp;
    Packet::PROTOCOL_VERSION _protocolVersion;
    uint8_t _clientID;
    int8_t _snr;
    PktQueueRx _rxQueue;
};

class LoRaClients
{
public:
    enum ID_PACKET_OP_CODE
    {
        DISCOVER,    /* To request an ID */
        ATTRIBUTION, /* Reply to discover, attribution of an ID */
        ID_REQUIRED  /* Send when a packet was received from an unknown client to ask for identification */
    };

    LoRaClients(uint8_t *macAddress = nullptr, uint8_t id = DEFAULT_ID)
    {
        if (id == DEFAULT_ID)
            _host._clientID = id;
        else
        {
            if (!setHostID(id))
                _host._clientID = DEFAULT_ID;
        }
        setHostMac(macAddress);
        _host._lastSeenTimestamp = 0;
        _host._snr = -128;
        _host._protocolVersion = Packet::VERSION_2;
    }

    void newPktFromClient(Packet pkt, int8_t snr);          /* Extract infos from received pkt and update the corresponding client infos */
    int getID(uint8_t macAddress[6]);                       /* return the ID of a client or -1 if not found */
    uint8_t *getMac(uint8_t clientID);                      /* return the mac address of a client */
    int8_t getSNR(uint8_t clientID);                        /* return the SNR of a client */
    Packet::PROTOCOL_VERSION getProtocol(uint8_t clientID); /* return the available protocol of a client */
    uint32_t lastSeenTimestamp(uint8_t clientID);           /* return the timestamp of the moment the client was last seen */
    String getClientListAsJSON();                           /* return the client list as JSON */
    bool isClientIDKnown(uint8_t clientID);                 /* return true if clientID matches a known client */
    bool isClientMacKnown(uint8_t *macAddress);             /* return true if macAddress matches a known client */
    int attribID(uint8_t *macAddress);                      /* Attribution of an ID to a client */
    bool isHostIDset();                                     /* return true if host id has been set */

    bool setHostMac(uint8_t *macAddress);                  /* set host mac address */
    bool setHostID(uint8_t id);                            /* set host id */
    bool setClientLastSeen(uint8_t clientID, uint32_t ts); /* set last seen ts of a client */
    bool setClientSNR(uint8_t clientID, int8_t snr);       /* set snr of a client */

    void setRcvCallback(void (*rcvCallback)(Packet pkt)); /* set receive callback to call when a packet was just received */

    bool addClient(LoRaClient client); /* Add a client in the list */

    bool addTXPacket(Packet pkt, const uint8_t &destID); /* Add a packet in the TX queue and automatically set the packet number. Returns true if operation was successful. */
    bool getNextTXPacket(Packet *pkt);                   /* Returns the next packet to be send as param. Function returns true if a valid packet has been found.*/
    bool markPktAsSent(const Packet &pkt);               /* Mark a packet as sent. Returns true if successful. */
    int TXsize() const;                                  /* Returns the number of packets in the queue */
    bool needToSendACK();                                /* returns true if an ACK is waiting for to be sent */
    uint32_t getLastSentPktTs();                         /* get the timestamp of the last sent packet */
    bool IDpktpending();

private:
    std::vector<LoRaClient> _clients; /* list of known clients */
    LoRaClient _host;                 /* infos of the host, its rxQueue stores the broadcast packets */
    uint32_t _clientTimeoutDelay;
    PktQueueTx _txQueue;                                                                        /* Delay beyond which we forget a client if nothing was received */
    void (*_rcvCallback)(Packet pkt) = nullptr; /* callback function to call when a new packet was received */
};

#endif