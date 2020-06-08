/*
 * discovery-packet-header.h
 *
 *  Created on: May 17, 2020
 *      Author: hassam
 */

#ifndef DISCOVERY_PACKET_HEADER_H_
#define DISCOVERY_PACKET_HEADER_H_
#include "ns3/header.h"
#include "ns3/ipv4-address.h"

namespace ns3 {
/**
 * \ingroup internet
 *
 * \brief Packet header for Network Discovery
 */
class DiscoveryPacketHeader : public Header
{
public:
	DiscoveryPacketHeader();
	/**
	 * \param source the source of this packet
	 */
	void SetSource (Ipv4Address source){
		m_source = source;
	}
	/**
	 * \param destination the destination of this packet.
	 */
	void SetDestination (Ipv4Address destination){
		m_destination = destination;
	}
	/**
	* \param nextLocation the nextLocation of this packet's sender.
	*/
	void SetNextLocation(uint16_t nextLocation){
		m_nextLocation = nextLocation;
	}
	/**
	 * \returns the source address of this packet
	 */
	Ipv4Address GetSource () const{
		return m_source;
	}
	/**
	 * \returns the destination address of this packet
	 */
	Ipv4Address GetDestination () const{
		return m_destination;
	}
	/**
	* \returns the next destination of this packet's sender
	*/
	uint16_t GetNextLocation() const{
		return m_nextLocation;
	}

	  static TypeId GetTypeId (void);
	  virtual TypeId GetInstanceTypeId (void) const;
	  virtual void Print (std::ostream &os) const;
	  virtual uint32_t GetSerializedSize (void) const;
	  virtual void Serialize (Buffer::Iterator start) const;
	  virtual uint32_t Deserialize (Buffer::Iterator start);

private:
	  Ipv4Address m_source; //!< source address
	  Ipv4Address m_destination; //!< destination address
	  uint32_t m_nextLocation; // Next most probable location

};
}




#endif /* DISCOVERY_PACKET_HEADER_H_ */
