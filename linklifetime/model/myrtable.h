/*
 * rtable.h
 *
 *  Created on: Mar 9, 2020
 *      Author: hassam
 */

#ifndef RTABLE_H
#define RTABLE_H

#include "ns3/ipv4.h"
#include "ns3/timer.h"
#include "ns3/net-device.h"
#include "ns3/output-stream-wrapper.h"
#include <map>
#include <ns3/vector.h>
#include <vector>

namespace ns3 {

/**
 * \brief Routing table entry
 */
class RTableEntry
{
private:
	Ipv4Address m_myAddress, m_destAddress;
	double m_timeConnected;
	Time m_timePktRcvd, m_timeFirstPktRcvd;
	Vector m_myLocation, m_neighborNodeLocation;
	int m_nextLoc, m_nextTime;
	double m_linkLifeTime;
	double m_currProSpeed;

public:
	RTableEntry(Ipv4Address myAddress = Ipv4Address(), Ipv4Address destAddress = Ipv4Address(),	double timeConnected = 0.0,Time timePktRcvd = Simulator::Now (),
			Time timeFirstPktRcvd = Simulator::Now (), Vector myLocation = {0.0, 0.0, 0.0},	Vector neighborNodeLocation = {0.0, 0.0, 0.0}, int nLoc = -1 ,
			int nTime = -1, double linkLifeTime = 0.0, double currProSpeed = 0.0);

	~RTableEntry();

	const Vector& getMyLocation() const {
		return m_myLocation;
	}

	void setMyLocation(const Vector &myLocation) {
		m_myLocation = myLocation;
	}

	const Vector& getNeighborNodeLocation() const {
		return m_neighborNodeLocation;
	}

	void setNeighborNodeLocation(const Vector &neighborNodeLocation) {
		m_neighborNodeLocation = neighborNodeLocation;
	}

	Ipv4Address getDestAddress() const {
		return m_destAddress;
	}

	void setDestAddress(Ipv4Address destAddress) {
		m_destAddress = destAddress;
	}

	Ipv4Address getMyAddress() const {
		return m_myAddress;
	}

	void setMyAddress(Ipv4Address myAddress) {
		m_myAddress = myAddress;
	}

	double getTimeConnected() const {
		return m_timeConnected;
	}

	void setTimeConnected(double timeConnected) {
		m_timeConnected = timeConnected;
	}

	const Time& getTimePktRcvd() const {
		return m_timePktRcvd;
	}

	void setTimePktRcvd(const Time &timePktRcvd) {
		m_timePktRcvd = timePktRcvd;
	}

	const Time& getTimeFirstPktRcvd() const {
		return m_timeFirstPktRcvd;
	}

	void setTimeFirstPktRcvd(const Time &timeFirstPktRcvd) {
		m_timeFirstPktRcvd = timeFirstPktRcvd;
	}

	void setNextLocation(int nLoc)
	{
		m_nextLoc = nLoc;
	}

	void setNextTime(int nTime)
	{
		m_nextTime = nTime;
	}

	int getNextLoc() const
	{
		return m_nextLoc;
	}

	int getNextTime() const
	{
		return m_nextTime;
	}

	double getLinkLifeTime() const {
		return m_linkLifeTime;
	}

	void setLinkLifeTime(double linkLifeTime) {
		m_linkLifeTime = linkLifeTime;
	}


	double getCurrProSpeed() const {
		return m_currProSpeed;
	}

	void setCurrProSpeed(double currProSpeed) {
		m_currProSpeed = currProSpeed;
	}

    void Print (Ptr<OutputStreamWrapper> stream) const;


};

class RTable{
private:
	/// an entry in the routing table.
    std::multimap<Ipv4Address, RTableEntry> m_ipv4AddressEntry;

public:

	/// c-tor
	RTable ();

	~RTable();
	/**
	 * Add routing table entry if it doesn't yet exist in routing table
	 * \param r routing table entry
	 * \return true in success
	 */
	bool
	AddRoute (RTableEntry & r);
	/**
	 * Delete routing table entry with destination address dst, if it exists.
	 * \param dst destination address
	 * \return true on success
	 */
	bool
	DeleteRoute (Ipv4Address dst);
	/**
	 * Lookup routing table entry with destination address dst
	 * \param dst destination address
	 * \param rt entry with destination address dst, if exists
	 * \return true on success
	 */
	bool
    LookupRoute (Ipv4Address dst, Ipv4Address src, RTableEntry & rt);

	/**
	 * Updating the routing Table with routing table entry rt
	 * \param rt routing table entry
	 * \return true on success
	 */
	bool
	Update (RTableEntry & rt);

	/**
	 * Lookup list of all addresses in the routing table
	 * \param allRoutes is the list that will hold all these addresses present in the nodes routing table
	 */

	void
	GetListOfAllRoutes ();

	void GetListofAllRoutes(Ipv4Address srcAddress);

	std::map<Ipv4Address, RTableEntry> GetAllRoutesWithIP(Ipv4Address address);

	void GetInActiveRoutes();
	/// Delete all entries from routing table
	void
	Clear ()
	{
		m_ipv4AddressEntry.clear ();
	}
	/**
	 * Delete all outdated entries if Lifetime is expired
	 * \param removedAddresses is the list of addresses to purge
	 */
	void
	Purge (std::map<Ipv4Address, RTableEntry> & removedAddresses);
	/**
	 * Print routing table
	 * \param stream the output stream
	 */
	void
    Print (Ptr<OutputStreamWrapper> stream, Ipv4Address nodeIP) const;
	void PrintRoutingTableAllAt (Time printTime, Ptr<OutputStreamWrapper> stream);
	void PrintRoutingTableAt (Time printTime, Ptr<Node> node, Ptr<OutputStreamWrapper> stream);

	/**
	 * Provides the number of routes present in that nodes routing table.
	 * \returns the number of routes
	 */
	uint32_t
	RTableSize ();
};
}


#endif /* RTABLE_H */
