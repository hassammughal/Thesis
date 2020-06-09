/*
 * discovery-application.h
 *
 *  Created on: May 17, 2020
 *      Author: Hassam
 */


#ifndef DISCOVERYAPPLICATION_H
#define DISCOVERYAPPLICATION_H

#include "ns3/applications-module.h"
#include "ns3/discovery-packet-header.h"

namespace ns3 {
class DiscoveryApplication : public Application
{
public:
    static TypeId GetTypeId();
    DiscoveryApplication ();
    virtual ~DiscoveryApplication();
    void ProcessDiscoveryPacket(DiscoveryPacketHeader header);
    void SendReplyPacketW(Ptr<Node> currNode, Ipv4Address source, Ipv4Address dest, uint16_t nextLoc, uint16_t nextTime, double currProSpeed);
    void SendReplyPacketWD(Ptr<Node> currNode,  Ipv4Address source, Ipv4Address dest, uint16_t nextLoc, uint16_t nextTime, double currProSpeed);
    void Setup (Address address1, Address address2, Time servicePeriod, uint16_t p1, uint16_t p2);
    Ptr<Node> currNode;
private:
    virtual void StartApplication (void);
    virtual void StopApplication (void);
    void ScheduleTx (void);
    //uint64_t GetDataRate(Ptr<Packet> pkt);
    void SendDiscoveryPacket (void);
    Ptr<Socket>     m_socket1;
    Ptr<Socket>     m_socket2;
    Address         m_peer1;
    Address         m_peer2;
    uint16_t        m_port1;
    uint16_t        m_port2;
    Ipv4Address     m_source1;
    Ipv4Address     m_source2;
    uint32_t        m_packetSize;
    uint32_t        m_nPackets;
    DataRate        m_dataRate;
    EventId         m_sendEvent;
    bool            m_running;
    uint32_t        m_packetsSent;
    uint16_t        m_pktSize;
    TypeId          m_tid;
    Time            m_period;
    TracedCallback<Ptr<const Packet> > m_txTrace;

    TracedCallback<Ptr<const Packet>, const Address &, const Address &> m_txTraceWithAddresses;
};

}

#endif // DISCOVERYAPPLICATION_H
