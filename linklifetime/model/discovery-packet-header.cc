/*
 * discovery-packet-header.cc
 *
 *  Created on: May 17, 2020
 *      Author: hassam
 */

#include "ns3/address-utils.h"
#include "ns3/packet.h"
#include "discovery-packet-header.h"

namespace ns3{
NS_OBJECT_ENSURE_REGISTERED(DiscoveryPacketHeader);

DiscoveryPacketHeader::DiscoveryPacketHeader(){

}

TypeId
DiscoveryPacketHeader::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::Internet::DiscoveryPacketHeader")
    .SetParent<Header> ()
    .SetGroupName ("DiscoveryPacketHeader")
    .AddConstructor<DiscoveryPacketHeader> ();
  return tid;
}

TypeId
DiscoveryPacketHeader::GetInstanceTypeId () const
{
  return GetTypeId ();
}

uint32_t
DiscoveryPacketHeader::GetSerializedSize () const
{
  return 12;
}

void
DiscoveryPacketHeader::Serialize (Buffer::Iterator i) const
{
  WriteTo (i, m_source);
  WriteTo (i, m_destination);
  i.WriteHtonU32 (m_nextLocation);

}

uint32_t
DiscoveryPacketHeader::Deserialize (Buffer::Iterator start)
{
  Buffer::Iterator i = start;

  ReadFrom (i, m_source);
  ReadFrom (i, m_destination);
  m_nextLocation = i.ReadNtohU32 ();

  uint32_t dist = i.GetDistanceFrom (start);
  NS_ASSERT (dist == GetSerializedSize ());
  return dist;
}

void
DiscoveryPacketHeader::Print (std::ostream &os) const
{
  os << "Source Address: " << m_source
     << " Destination Address: " << m_destination
     << " Next Location: " << m_nextLocation;
}


}

