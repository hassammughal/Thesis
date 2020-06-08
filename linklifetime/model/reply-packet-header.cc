/*
 * reply-packet-header.cc
 *
 *  Created on: May 17, 2020
 *      Author: hassam
 */

#include "ns3/address-utils.h"
#include "ns3/packet.h"
#include "ns3/double.h"
#include "reply-packet-header.h"

namespace ns3{
NS_OBJECT_ENSURE_REGISTERED(ReplyPacketHeader);

ReplyPacketHeader::ReplyPacketHeader(){

}

TypeId
ReplyPacketHeader::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::Internet::ReplyPacketHeader")
    .SetParent<Header> ()
    .SetGroupName ("ReplyPacketHeader")
    .AddConstructor<ReplyPacketHeader> ();
  return tid;
}

TypeId
ReplyPacketHeader::GetInstanceTypeId () const
{
  return GetTypeId ();
}

uint32_t
ReplyPacketHeader::GetSerializedSize () const
{
  return 20;
}

void
ReplyPacketHeader::Serialize (Buffer::Iterator i) const
{
  WriteTo (i, m_source);
  WriteTo (i, m_destination);
  i.WriteHtonU16 (m_nextLocation);
  i.WriteHtonU16 (m_nextTimeInterval);
//  i.WriteHtonU64 (m_currProSpeed);
  i.Write((uint8_t*)&m_currProSpeed,sizeof(m_currProSpeed));
}

uint32_t
ReplyPacketHeader::Deserialize (Buffer::Iterator start)
{
  Buffer::Iterator i = start;

  ReadFrom (i, m_source);
  ReadFrom (i, m_destination);
  m_nextLocation = i.ReadNtohU16 ();
  m_nextTimeInterval = i.ReadNtohU16();
//  m_currProSpeed = i.ReadNtohU64();
  i.Read((uint8_t*)&m_currProSpeed,sizeof(m_currProSpeed));

  uint32_t dist = i.GetDistanceFrom (start);
  NS_ASSERT (dist == GetSerializedSize ());
  return dist;
}

void
ReplyPacketHeader::Print (std::ostream &os) const
{
  os << "Source Address: " << m_source << ", Destination Address: " << m_destination  << ", Next Location: " << m_nextLocation << ", Next Time Interval: "
		  << m_nextTimeInterval << ", Processor Speed: " << m_currProSpeed;
}
}


