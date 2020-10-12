#include "loraClients.h"

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
// #ifdef DEBUG_SERIAL_PACKET
#if 1
// #if DEBUG_SERIAL_PACKET >= 4
#if 1
#define debugA(fmt, ...) Serial.printf("[A][C%d][%ld][%s:%d] %s: \t" fmt "\n", xPortGetCoreID(), millis(), __FILE__, __LINE__, __func__, ##__VA_ARGS__)
#define debugP(fmt, ...) Serial.printf("[P][C%d][%ld][%s:%d] %s: \t" fmt "\n", xPortGetCoreID(), millis(), __FILE__, __LINE__, __func__, ##__VA_ARGS__)
#define debugV(fmt, ...) Serial.printf("[V][C%d][%ld][%s:%d] %s: \t" fmt "\n", xPortGetCoreID(), millis(), __FILE__, __LINE__, __func__, ##__VA_ARGS__)
#else
#define debugA(fmt, ...)
#define debugP(fmt, ...)
#define debugV(fmt, ...)
#endif
// #if DEBUG_SERIAL_PACKET >= 3
#if 1
#define debugD(fmt, ...) Serial.printf("[D][C%d][%ld][%s:%d] %s: \t" fmt "\n", xPortGetCoreID(), millis(), __FILE__, __LINE__, __func__, ##__VA_ARGS__)
#else
#define debugD(fmt, ...)
#endif
// #if DEBUG_SERIAL_PACKET >= 2
#if 1
#define debugI(fmt, ...) Serial.printf("[I][C%d][%ld][%s:%d] %s: \t" fmt "\n", xPortGetCoreID(), millis(), __FILE__, __LINE__, __func__, ##__VA_ARGS__)
#else
#define debugI(fmt, ...)
#endif
// #if DEBUG_SERIAL_PACKET >= 1
#if 1
#define debugW(fmt, ...) Serial.printf("[W][C%d][%ld][%s:%d] %s: \t" fmt "\n", xPortGetCoreID(), millis(), __FILE__, __LINE__, __func__, ##__VA_ARGS__)
#else
#define debugW(fmt, ...)
#endif
// #if DEBUG_SERIAL_PACKET >= 0
#if 1
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

void LoRaClients::newPktFromClient(Packet pkt, int8_t snr)
{
    debugV("Received new pkt with snr %d", snr);
    if (!isClientIDKnown(pkt.getSourceID()) && (pkt.getSourceID() != GATEWAY_ID)) /* if the client is not known */
    {
        debugD("Unknown client ID");
        if (pkt.getDestID() == _host._clientID)
        {
            /* we ask for its identification */
            uint8_t idPktPayload[1] = {ID_REQUIRED};
            Packet idPkt = Packet(1, idPktPayload);
            idPkt.setPiority(Packet::PRIORITY_HIGH);
            idPkt.setType(Packet::ID);
            idPkt.setQoS(Packet::ONE_PACKET_AT_MOST);
            idPkt.setProtocolVersion(_host._protocolVersion);
            idPkt.setSourceID(_host._clientID);
            idPkt.setDestID(pkt.getSourceID());
            _txQueue.addPacket(idPkt);
        }
    }
    else /* else, the client is known / msg is broadcast */
    {
        setClientLastSeen(pkt.getSourceID(), millis());
        setClientSNR(pkt.getSourceID(), snr);

        switch (pkt.getType())
        {
        case Packet::PING:
        case Packet::OTA:
        case Packet::DATA:
        {
            debugD("Received DATA pkt");
            if (pkt.getDestID() == _host._clientID ||
                pkt.getDestID() == BROADCAST_ID)
            {
                std::vector<LoRaClient>::iterator it;
                for (it = _clients.begin(); it != _clients.end(); ++it)
                {
                    if (pkt.getSourceID() == it->_clientID)
                    {
                        it->_rxQueue.addPacket(pkt);
                        break;
                    }
                }
            }
            break;
        }
        case Packet::ACK:
            debugD("received ACK pkt");
            if (pkt.getDestID() == _host._clientID ||
                pkt.getDestID() == BROADCAST_ID)
            {
                if (pkt.getDataSize() < 1)
                {
                    debugE("Received ACK with no payload!");
                    break;
                }
                else
                {
                    _txQueue.removePkt(pkt.getSourceID(), pkt.getData()[0]);
                    break;
                }
            }
            break;
        case Packet::ID:
            debugD("received ID pkt");
            if (pkt.getDataSize() > 0)
            {
                switch (pkt.getData()[0])
                {
                case DISCOVER:
                    debugD("ID pkt -> DISCOVER");
                    if (pkt.getDestID() == _host._clientID)
                    {
                        if (pkt.getDataSize() <= 6)
                        {
                            debugE("wrong payload for pkt ID DISCOVER");
                            break;
                        }
                        else
                        {
                            int id = attribID(&(pkt.getData()[1]));
                            if (id == -1)
                            {
                                debugE("Could not attribute a new client ID, ignoring request!");
                            }
                            else
                            {
                                LoRaClient newClient;
                                memcpy(newClient._macAddress, &(pkt.getData()[1]), 6);
                                newClient._lastSeenTimestamp = millis();
                                newClient._clientID = id;
                                newClient._snr = snr;
                                newClient._protocolVersion = pkt.getProtocolVersion();
                                newClient._rxQueue.setRcvCallback(_rcvCallback);
                                addClient(newClient);

                                uint8_t attribPayload[14] = {ATTRIBUTION};
                                memcpy(attribPayload + 1, newClient._macAddress, 6);
                                attribPayload[7] = id;
                                memcpy(attribPayload + 8, _host._macAddress, 6);
                                Packet attrib = Packet(14, attribPayload);
                                attrib.setPiority(Packet::PRIORITY_HIGH);
                                attrib.setType(Packet::ID);
                                attrib.setQoS(Packet::ONE_PACKET_AT_MOST);
                                attrib.setProtocolVersion(_host._protocolVersion);
                                attrib.setSourceID(GATEWAY_ID);
                                attrib.setDestID(BROADCAST_ID);
                                _txQueue.addPacket(attrib);
                            }
                        }
                    }
                    break;
                case ATTRIBUTION:
                    debugD("ID pkt -> ATTRIBUTION");
                    if (pkt.getDataSize() <= 13)
                    {
                        debugE("wrong payload for pkt ID ATTRIBUTION");
                        break;
                    }
                    else
                    {
                        if (memcmp(_host._macAddress, &(pkt.getData()[1]), 6) == 0) /* if macAddress is ours */
                        {
                            _host._clientID = pkt.getData()[7]; /* save the attributed id */

                            LoRaClient newClient;
                            memcpy(newClient._macAddress, &(pkt.getData()[8]), 6);
                            newClient._lastSeenTimestamp = millis();
                            newClient._clientID = pkt.getSourceID();
                            newClient._snr = snr;
                            newClient._protocolVersion = pkt.getProtocolVersion();
                            newClient._rxQueue.setRcvCallback(_rcvCallback);
                            addClient(newClient);
                        }
                    }
                    break;
                case ID_REQUIRED:
                {
                    debugD("ID pkt -> ID_REQUIRED");
                    if (pkt.getDestID() == _host._clientID)
                    {
                        _host._clientID = DEFAULT_ID;
                        uint8_t idPktPayload[7] = {DISCOVER};
                        memcpy(idPktPayload + 1, _host._macAddress, 6);
                        Packet idPkt = Packet(7, idPktPayload);
                        idPkt.setPiority(Packet::PRIORITY_HIGH);
                        idPkt.setType(Packet::ID);
                        idPkt.setQoS(Packet::ONE_PACKET_AT_MOST);
                        idPkt.setProtocolVersion(_host._protocolVersion);
                        idPkt.setSourceID(BROADCAST_ID);
                        idPkt.setDestID(pkt.getSourceID());
                        _txQueue.addPacket(idPkt);
                    }
                    break;
                }
                default:
                    debugE("Unknown op code for ID packet!");
                    break;
                }
            }
            else
            {
                debugE("A packet of type ID was received with no data!");
            }
            break;
        default:
            break;
        }
    }
}

int LoRaClients::getID(uint8_t macAddress[6])
{
    if (macAddress == nullptr)
        return -1;

    // Check if macAddress matches the _host
    if (memcmp(macAddress, _host._macAddress, 6) == 0)
    {
        return _host._clientID;
    }

    // Check if the macAddress matches the one of our known clients
    std::vector<LoRaClient>::iterator it;
    for (it = _clients.begin(); it != _clients.end(); ++it)
    {
        if (memcmp(macAddress, it->_macAddress, 6) == 0)
        {
            return it->_clientID;
        }
    }
    // No match, return -1
    return -1;
}

uint8_t *LoRaClients::getMac(uint8_t clientID)
{
    // Check if clientID matches the _host
    if (clientID == _host._clientID)
    {
        return _host._macAddress;
    }

    // Check if the clientID matches the one of our known clients
    std::vector<LoRaClient>::iterator it;
    for (it = _clients.begin(); it != _clients.end(); ++it)
    {
        if (clientID == it->_clientID)
        {
            return it->_macAddress;
        }
    }
    // No match, return nullptr
    return nullptr;
}

int8_t LoRaClients::getSNR(uint8_t clientID)
{
    // Check if clientID matches the _host
    if (clientID == _host._clientID)
    {
        return _host._snr;
    }

    // Check if the clientID matches the one of our known clients
    std::vector<LoRaClient>::iterator it;
    for (it = _clients.begin(); it != _clients.end(); ++it)
    {
        if (clientID == it->_clientID)
        {
            return it->_snr;
        }
    }
    // No match, return min value
    return -128;
}

Packet::PROTOCOL_VERSION LoRaClients::getProtocol(uint8_t clientID)
{
    // Check if clientID matches the _host
    if (clientID == _host._clientID)
    {
        return _host._protocolVersion;
    }

    // Check if the clientID matches the one of our known clients
    std::vector<LoRaClient>::iterator it;
    for (it = _clients.begin(); it != _clients.end(); ++it)
    {
        if (clientID == it->_clientID)
        {
            return it->_protocolVersion;
        }
    }
    // No match, return min value
    return Packet::VERSION_UNK;
}

uint32_t LoRaClients::lastSeenTimestamp(uint8_t clientID)
{
    // Check if clientID matches the _host
    if (clientID == _host._clientID)
    {
        return _host._lastSeenTimestamp;
    }

    // Check if the clientID matches the one of our known clients
    std::vector<LoRaClient>::iterator it;
    for (it = _clients.begin(); it != _clients.end(); ++it)
    {
        if (clientID == it->_clientID)
        {
            return it->_lastSeenTimestamp;
        }
    }
    // No match, return min value
    return 0;
}

String LoRaClients::getClientListAsJSON()
{
    String json = "[";
    bool atLeastOneClient = false;

    std::vector<LoRaClient>::iterator it;
    for (it = _clients.begin(); it != _clients.end(); ++it)
    {
        if (atLeastOneClient)
            json += ",";
        json += "{\"clientID\":\"";
        json += it->_clientID;
        json += "\",";
        json += "\"lastSeenTs\":\"";
        json += it->_lastSeenTimestamp;
        json += "\",";
        json += "\"snr\":\"";
        json += it->_snr;
        json += "\",";
        json += "\"protocol\":\"";
        json += it->_protocolVersion;
        json += "\",";
        json += "\"mac\":\"";
        char mac[18];
        sprintf(mac, "%02x:%02x:%02x:%02x:%02x:%02x",
                it->_macAddress[0], it->_macAddress[1],
                it->_macAddress[2], it->_macAddress[3],
                it->_macAddress[4], it->_macAddress[5]);
        json += mac;
        json += "\"}";
        atLeastOneClient = true;
    }

    json += "]";
    return json;
}

bool LoRaClients::isClientIDKnown(uint8_t clientID)
{
    if (clientID == BROADCAST_ID)
        return true;

    std::vector<LoRaClient>::iterator it;
    for (it = _clients.begin(); it != _clients.end(); ++it)
    {
        if (clientID == it->_clientID)
        {
            return true;
        }
    }
    return false;
}

bool LoRaClients::isClientMacKnown(uint8_t *macAddress)
{
    std::vector<LoRaClient>::iterator it;
    for (it = _clients.begin(); it != _clients.end(); ++it)
    {
        if (memcmp(macAddress, it->_macAddress, 6) == 0)
        {
            return true;
        }
    }
    return false;
}

int LoRaClients::attribID(uint8_t *macAddress)
{
    int id = getID(macAddress);
    if (id != -1)
        return id;
    id = 1;
    for (id = 1; id <= MAX_CLIENT_ID; ++id)
    {
        if (!isClientIDKnown(id) && (id != _host._clientID))
            break;
    }
    if (id > MAX_CLIENT_ID)
    {
        debugE("No client ID available");
        return -1;
    }

    return id;
}

bool LoRaClients::setHostMac(uint8_t *macAddress)
{
    if (macAddress == nullptr)
        return false;
    memcpy(_host._macAddress, macAddress, 6);
    return true;
}

bool LoRaClients::setClientLastSeen(uint8_t clientID, uint32_t ts)
{
    std::vector<LoRaClient>::iterator it;
    for (it = _clients.begin(); it != _clients.end(); ++it)
    {
        if (clientID == it->_clientID)
        {
            it->_lastSeenTimestamp = ts;
            return true;
        }
    }
    return false;
}

bool LoRaClients::setClientSNR(uint8_t clientID, int8_t snr)
{
    std::vector<LoRaClient>::iterator it;
    for (it = _clients.begin(); it != _clients.end(); ++it)
    {
        if (clientID == it->_clientID)
        {
            it->_snr = snr;
            return true;
        }
    }
    return false;
}

void LoRaClients::setRcvCallback(void (*rcvCallback)(uint8_t *payload, size_t size, Packet::PACKET_TYPE pktType))
{
    _rcvCallback = rcvCallback;
    std::vector<LoRaClient>::iterator it;
    for (it = _clients.begin(); it != _clients.end(); ++it)
    {
        it->_rxQueue.setRcvCallback(rcvCallback);
    }
}

bool LoRaClients::isHostIDset()
{
    return _host._clientID != DEFAULT_ID;
}

bool LoRaClients::setHostID(uint8_t id)
{
    if (id <= MAX_CLIENT_ID)
    {
        _host._clientID = id;
        return true;
    }
    return false;
}

bool LoRaClients::addClient(LoRaClient client)
{
    // check if client already exists and update it if it does
    std::vector<LoRaClient>::iterator it;
    for (it = _clients.begin(); it != _clients.end(); ++it)
    {
        if (memcmp(client._macAddress, it->_macAddress, 6) == 0)
        {
            it->_snr = client._snr;
            it->_clientID = client._clientID;
            it->_lastSeenTimestamp = client._lastSeenTimestamp;
            it->_protocolVersion = client._protocolVersion;
            it->_rxQueue = client._rxQueue;
            return true;
        }
    }
    // if client didn't already exist, create it
    _clients.push_back(client);
    return true;
}