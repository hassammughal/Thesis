/*
 * discovery-application.cc
 *
 *  Created on: May 17, 2020
 *      Author: Hassam
 */

#include "ns3/udp-socket-factory.h"
#include "ns3/packet-socket-address.h"
#include "ns3/discovery-packet-header.h"
#include "ns3/socket.h"
#include "ns3/ipv4-address.h"
#include "discovery-application.h"
#include "ns3/ipv4.h"
#include "ns3/markovchain-mobility-model.h"
#include "ns3/reply-packet-header.h"
#include "ns3/log.h"

namespace ns3{
NS_LOG_COMPONENT_DEFINE ("DiscoveryApplication");

NS_OBJECT_ENSURE_REGISTERED(DiscoveryApplication);

TypeId
DiscoveryApplication::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::DiscoveryApplication")
    .SetParent<Application> ()
    .SetGroupName("Applications")
    .AddConstructor<DiscoveryApplication> ()
    .AddAttribute ("PacketSize", "The size of packets sent in on state",
                   UintegerValue (512),
                   MakeUintegerAccessor (&DiscoveryApplication::m_pktSize),
                   MakeUintegerChecker<uint32_t> (1))
    .AddAttribute ("Protocol", "The type of protocol to use. This should be "
                   "a subclass of ns3::SocketFactory",
                   TypeIdValue (UdpSocketFactory::GetTypeId ()),
                   MakeTypeIdAccessor (&DiscoveryApplication::m_tid),
                   // This should check for SocketFactory as a parent
                   MakeTypeIdChecker ())
    .AddTraceSource ("Tx", "A new packet is created and is sent",
                     MakeTraceSourceAccessor (&DiscoveryApplication::m_txTrace),
                     "ns3::Packet::TracedCallback")
    .AddTraceSource ("TxWithAddresses", "A new packet is created and is sent",
                     MakeTraceSourceAccessor (&DiscoveryApplication::m_txTraceWithAddresses),
                     "ns3::Packet::TwoAddressTracedCallback")
  ;
  return tid;
}



DiscoveryApplication::DiscoveryApplication ()
: m_socket1 (0),
  m_socket2(0),
  m_peer1 (),
  m_peer2(),
  m_packetSize (100),
  m_nPackets (1000000),
  m_dataRate (0),
  m_sendEvent (),
  m_running (false),
  m_packetsSent (0)
{
}
DiscoveryApplication::~DiscoveryApplication()
{
    m_socket1 = 0;
    m_socket2 = 0;

}

void
DiscoveryApplication::ProcessDiscoveryPacket(DiscoveryPacketHeader header)
{

}

void
DiscoveryApplication::SendReplyPacketW(Ptr<Node> currNode, Ipv4Address source, Ipv4Address dest, uint16_t nextLoc, uint16_t nextTime, double currProSpeed)
{
    uint32_t packetSize = 100;
    TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");
    ReplyPacketHeader header;
    header.SetSource(source);
    header.SetDestination(dest);
    header.SetNextLocation(nextLoc);
    header.SetNextTimeInterval(nextTime);
    header.SetCurrProSpeed(currProSpeed);
    Ptr<Socket> replySocket = Socket::CreateSocket (currNode, tid);
    replySocket->Bind();
    replySocket->Connect(InetSocketAddress(dest, 9));
    Ptr<Packet> packet = Create<Packet> (packetSize);
    packet->AddHeader(header);
    replySocket->Send(packet);
    replySocket->Close();
    NS_LOG_DEBUG("Sending Reply Packet W");

}

void
DiscoveryApplication::SendReplyPacketWD(Ptr<Node> currNode, Ipv4Address source, Ipv4Address dest, uint16_t nextLoc, uint16_t nextTime, double currProSpeed)
{
    uint32_t packetSize = 100;
    TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");
    ReplyPacketHeader header;
    header.SetSource(source);
    header.SetDestination(dest);
    header.SetNextLocation(nextLoc);
    header.SetNextTimeInterval(nextTime);
    header.SetCurrProSpeed(currProSpeed);
    Ptr<Socket> replySocket = Socket::CreateSocket (currNode, tid);
    replySocket->Bind();
    replySocket->Connect(InetSocketAddress(dest, 80));
    Ptr<Packet> packet = Create<Packet> (packetSize);
    packet->AddHeader(header);
    replySocket->Send(packet);
    replySocket->Close();
    NS_LOG_DEBUG("Sending Reply Packet WD");
}

void
DiscoveryApplication::Setup (Address address1, Address address2, Time servicePeriod,uint16_t p1, uint16_t p2)
{
    m_peer1 = address1;
    m_peer2 = address2;
    //m_peer1 = InetSocketAddress(Ipv4Address("255.255.255.255"), p1);
   // m_peer2 = InetSocketAddress(Ipv4Address("255.255.255.255"), p2);
    m_period = servicePeriod;
    m_source1 = GetNode()->GetObject<Ipv4>()->GetAddress(1,0).GetLocal();
    m_source2 = GetNode()->GetObject<Ipv4>()->GetAddress(2,0).GetLocal();
    m_tid = TypeId::LookupByName("ns3::UdpSocketFactory");
    m_port1 = p1;
    m_port2 = p2;
    currNode = GetNode();
//    m_packetSize = packetSize;
//    m_nPackets = nPackets;
//    m_dataRate = dataRate;
}
void
DiscoveryApplication::StartApplication (void)
{

    m_running = true;
    m_packetsSent = 0;
    if (!m_socket1)
         {
           m_socket1 = Socket::CreateSocket (GetNode (), m_tid);
           if (InetSocketAddress::IsMatchingType (m_peer1) ||
                    PacketSocketAddress::IsMatchingType (m_peer1))
             {
               if (m_socket1->Bind () == -1)
                 {
                   NS_FATAL_ERROR ("Failed to bind socket");
                 }
             }
      m_socket1->SetAllowBroadcast (true);
      m_socket1->Connect (m_peer1);

    }

    if (!m_socket2)
         {
           m_socket2 = Socket::CreateSocket (GetNode (), m_tid);
           if (InetSocketAddress::IsMatchingType (m_peer2) ||
                    PacketSocketAddress::IsMatchingType (m_peer2))
             {
               if (m_socket2->Bind () == -1)
                 {
                   NS_FATAL_ERROR ("Failed to bind socket");
                 }
             }
     m_socket2->SetAllowBroadcast (true);
     m_socket2->Connect (m_peer2);

    }
//    m_socket->Bind ();
//    m_socket->Connect (m_peer);
    //SendDiscoveryPacket ();
    ScheduleTx();

}
void
DiscoveryApplication::StopApplication (void)
{
    m_running = false;
    if (m_sendEvent.IsRunning ())
    {
        Simulator::Cancel (m_sendEvent);
    }
    if (m_socket1)
    {
        m_socket1->Close ();
    }

    if (m_socket2)
    {
        m_socket2->Close ();
    }
}
void
DiscoveryApplication::SendDiscoveryPacket (void)
{
    DiscoveryPacketHeader header1;
    DiscoveryPacketHeader header2;

    header1.SetSource(m_source1);
    header2.SetSource(m_source2);
    NS_LOG_DEBUG("Next Location being Set in the header: " << GetNode()->GetObject<MarkovChainMobilityModel>()->GetNextLocation());
    header1.SetNextLocation(GetNode()->GetObject<MarkovChainMobilityModel>()->GetNextLocation());
    header2.SetNextLocation(GetNode()->GetObject<MarkovChainMobilityModel>()->GetNextLocation());

    Ptr<Packet> packet1 = Create<Packet> (m_packetSize);
    Ptr<Packet> packet2 = Create<Packet> (m_packetSize);

    packet1->AddHeader(header1);
    packet2->AddHeader(header2);
    int a = packet1->GetSize();
    NS_LOG_DEBUG("Packet Size: " << a);
    //GetDataRate(packet);
    m_socket1->Send(packet1);
    m_socket2->Send(packet2);

    m_txTrace (packet1);
    if (InetSocketAddress::IsMatchingType (m_peer1))
      {
        NS_LOG_INFO ("At time " << Simulator::Now ().GetSeconds ()
                     << "s discovery application sent "
                     <<  packet1->GetSize () << " bytes to "
                     << InetSocketAddress::ConvertFrom(m_peer1).GetIpv4 ()
                     << " port " << InetSocketAddress::ConvertFrom (m_peer1).GetPort ()
                     <<"from "<<m_source1);
        m_txTraceWithAddresses (packet1, m_source1, InetSocketAddress::ConvertFrom (m_peer1));
      }


    m_txTrace (packet2);
    if (InetSocketAddress::IsMatchingType (m_peer2))
      {
        NS_LOG_INFO ("At time " << Simulator::Now ().GetSeconds ()
                     << "s discovery application sent "
                     <<  packet2->GetSize () << " bytes to "
                     << InetSocketAddress::ConvertFrom(m_peer2).GetIpv4 ()
                     << " port " << InetSocketAddress::ConvertFrom (m_peer2).GetPort ()
                     <<"from "<<m_source2);
        m_txTraceWithAddresses (packet2, m_source2, InetSocketAddress::ConvertFrom (m_peer2));
      }

    if (++m_packetsSent < m_nPackets)
    {
        ScheduleTx ();
    }
}
void
DiscoveryApplication::ScheduleTx (void)
{
    if (m_running)
    {
        //Time tNext (Seconds (m_packetSize * 8 / static_cast<double> (m_dataRate.GetBitRate ())));
        Ptr<UniformRandomVariable> x = CreateObject<UniformRandomVariable>();
        x->SetAttribute("Min",DoubleValue(-1));
        x->SetAttribute("Max",DoubleValue(1));
        double value = x->GetValue();
        m_sendEvent = Simulator::Schedule (m_period + Seconds(value), &DiscoveryApplication::SendDiscoveryPacket, this);
    }
}



}
