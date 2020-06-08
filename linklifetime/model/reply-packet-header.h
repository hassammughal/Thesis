/*
 * reply-packet-header.h
 *
 *  Created on: May 17, 2020
 *      Author: hassam
 */

#ifndef REPLY_PACKET_HEADER_H_
#define REPLY_PACKET_HEADER_H_

#include "ns3/header.h"
#include "ns3/ipv4-address.h"

namespace ns3 {
/**
 * \ingroup internet
 *
 * \brief Packet header for Network Discovery
 */
class ReplyPacketHeader : public Header
{
public:
	ReplyPacketHeader();
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
	* \param timeInterval the next most probable time interval of this packet's sender.
	*/
	void SetNextTimeInterval(uint16_t nextTimeInterval){
		m_nextTimeInterval = nextTimeInterval;
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
	/**
	* \returns the next time interval of this packet's sender
	*/
	uint16_t GetNextTimeInterval() const{
		return m_nextTimeInterval;
	}

	/**
	* \returns the current processing speed of the packet's sender
	*/
	double GetCurrProSpeed() const {
		return m_currProSpeed;
	}

	void SetCurrProSpeed(double currProSpeed) {
		m_currProSpeed = currProSpeed;
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
	  uint16_t m_headerSize; //!< IP header size
	  uint16_t m_nextLocation; // Next most probable location
	  uint16_t m_nextTimeInterval; //Next most probable time interval
	  double m_currProSpeed;

};
}



#endif /* REPLY_PACKET_HEADER_H_ */
