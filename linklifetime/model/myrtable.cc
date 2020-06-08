/*
 * rtable.cc
 *
 *  Created on: Mar 9, 2020
 *      Author: ns3
 */

#include <stdio.h>
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/ipv4.h"
#include "ns3/node.h"
#include "ns3/net-device.h"
#include "ns3/simulator.h"
#include <iomanip>
#include <vector>
#include <ns3/vector.h>
#include "myrtable.h"
#include "ns3/log.h"


namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("RTable");

RTableEntry::RTableEntry(Ipv4Address myAddress, Ipv4Address destAddress, double timeConnected, Time timePktRcvd, Time timeFirstPktRcvd, Vector myLocation,
        Vector neighborNodeLocation,int nLoc, int nTime, double linkLifeTime, double currProSpeed):
		m_myAddress (myAddress),
		m_destAddress (destAddress),
		m_timeConnected (timeConnected),
		m_timePktRcvd (timePktRcvd),
		m_timeFirstPktRcvd (timeFirstPktRcvd),
		m_myLocation (myLocation),
        m_neighborNodeLocation (neighborNodeLocation),
        m_nextLoc(nLoc),
        m_nextTime(nTime),
		m_linkLifeTime(linkLifeTime),
		m_currProSpeed(currProSpeed)
{
}

RTableEntry::~RTableEntry ()
{
}

RTable::~RTable()
{
	NS_LOG_DEBUG("RTable Destructor is called!");
}

RTable::RTable ()
{
	NS_LOG_DEBUG("RTable Constructor is called!");
}

bool
RTable::LookupRoute (Ipv4Address id, Ipv4Address myID, RTableEntry & rt) //looks up route in routing table for the destination address in header of the packet
{																	//returns true, if the route is available, else returns false

//  if (m_ipv4AddressEntry.empty ())
//    {
//	  NS_LOG_DEBUG("IPv4AddressEntry is empty");
//      return false;
//    }

    typedef std::multimap<Ipv4Address, RTableEntry>::iterator MMapIt;
    std::pair<MMapIt, MMapIt> result = m_ipv4AddressEntry.equal_range(id);
  //  if (i == m_ipv4AddressEntry.end ())
  //    {
  //      return false;
  //    }
    for(MMapIt it = result.first; it!=result.second;it++)
    {
        if(it->second.getMyAddress() == myID)
    {
        rt = it->second;
        return true;
    }
    }

        return false;


//    std::map<Ipv4Address, RTableEntry>::const_iterator i = m_ipv4AddressEntry.find (id);
//  if (i == m_ipv4AddressEntry.end ())
//    {
//	  NS_LOG_DEBUG("IPv4AddressEntry is empty");
//      return false;
//    }
//  RTableEntry temp = i->second;
//  if(temp.getMyAddress() == myID)
//    {
//      rt = i->second;
//      return true;
//    }
//  else
//      return false;
}

void
RTable::GetListOfAllRoutes ()
{
/*
  for (std::map<Ipv4Address, RTableEntry>::iterator i = m_ipv4AddressEntry.begin (); i != m_ipv4AddressEntry.end (); ++i)
    {
      if (i->second.getDestAddress() != Ipv4Address ("127.0.0.1") && i->second.getDestAddress() != Ipv4Address ("102.102.102.102"))
        {
          allRoutes.insert (
            std::make_pair (i->first,i->second));
        }
    }
 */
	NS_LOG_DEBUG("============================Printing Table======================================");
	for (auto const map_entry : m_ipv4AddressEntry)
	{
		NS_LOG_DEBUG ("Source Address: " << map_entry.second.getMyAddress() << ", Destination Address: " << map_entry.second.getDestAddress() << ", TimeConnected: "
			<< map_entry.second.getTimeConnected() << "s, Time Last Packet Received At: " << map_entry.second.getTimePktRcvd().GetSeconds() <<
			"s, Time First Packet Received At: " << map_entry.second.getTimeFirstPktRcvd().GetSeconds() << "s, My Current Location: "<< map_entry.second.getMyLocation()
			<< ", Neighbor Node's Current Location: " << map_entry.second.getNeighborNodeLocation() <<", Neighbor Next Location: " << map_entry.second.getNextLoc()
			<< ", Neighbor Next Time Interval: " << map_entry.second.getNextTime() << ", Link LifeTime: " << map_entry.second.getLinkLifeTime()
			<< "s, Current Processing Speed: " << map_entry.second.getCurrProSpeed() << "GHz\n");
	}
	NS_LOG_DEBUG("============================Table Printing Ended======================================\n");
}

void RTable::GetListofAllRoutes(Ipv4Address myAddress){
	NS_LOG_DEBUG("============================Printing Table for " << myAddress << "======================================");
	for (auto const map_entry : m_ipv4AddressEntry)
	{
		if(map_entry.second.getMyAddress() == myAddress)
			NS_LOG_DEBUG ("Source Address: " << map_entry.second.getMyAddress() << ", Destination Address: " << map_entry.second.getDestAddress()
				<< ", TimeConnected: " << map_entry.second.getTimeConnected() << "s, Time Last Packet Received At: " << map_entry.second.getTimePktRcvd().GetSeconds()
				<< "s, Time First Packet Received At: " <<	map_entry.second.getTimeFirstPktRcvd().GetSeconds() << "s, My Current Location: " <<
				map_entry.second.getMyLocation() << ", Neighbor Node's Current Location: " << map_entry.second.getNeighborNodeLocation() << ", Neighbor Next Location: "
				<< map_entry.second.getNextLoc() << ", Neighbor Next Time Interval: " << map_entry.second.getNextTime() << ", Link LifeTime: "
				<< map_entry.second.getLinkLifeTime() << "s, Current Processing Speed: " << map_entry.second.getCurrProSpeed() << "GHz\n");
		}
        NS_LOG_DEBUG("============================Table Printing Ended======================================\n");
}

std::map<Ipv4Address, RTableEntry> RTable::GetAllRoutesWithIP(Ipv4Address address)
{
    std::map<Ipv4Address, RTableEntry> entries;
    for (auto const map_entry : m_ipv4AddressEntry)
    {
        if(map_entry.second.getMyAddress() == address)
//            NS_LOG_DEBUG ("Source Address: " << map_entry.second.getMyAddress() << ", Destination Address: " << map_entry.second.getDestAddress() << ", TimeConnected: "
//                    << map_entry.second.getTimeConnected() << "s, Time Last Packet Received At: " << map_entry.second.getTimePktRcvd().GetSeconds() <<
//                    "s, Time First Packet Received At: " << map_entry.second.getTimeFirstPktRcvd().GetSeconds() << "s, My Current Location: " << map_entry.second.getMyLocation()
//                    << ", Neighbor Node's Current Location: " << map_entry.second.getNeighborNodeLocation() <<"\n");
        entries.insert(std::pair<Ipv4Address, RTableEntry>(map_entry.first,map_entry.second));
    }
    return entries;
}

void RTable::GetInActiveRoutes(){
	NS_LOG_DEBUG("============================ Printing Table Every Second To check InActive Routes ======================================");
	for (auto const map_entry : m_ipv4AddressEntry)
	{
		double currTime = Simulator::Now().GetSeconds();
		double lastRcvdTime = map_entry.second.getTimePktRcvd().GetSeconds();
		double inActiveTime = currTime - lastRcvdTime;
		if(inActiveTime > 5.0)
			NS_LOG_DEBUG ("Source/My Address: " << map_entry.second.getMyAddress() << ", Destination Address: " << map_entry.second.getDestAddress() << ", TimeConnected: "
				<< map_entry.second.getTimeConnected() << "s, Time Last Packet Received At: " << map_entry.second.getTimePktRcvd().GetSeconds() <<
				"s, Time First Packet Received At: " << map_entry.second.getTimeFirstPktRcvd().GetSeconds() << "s, My Current Location: "
				<< map_entry.second.getMyLocation()	<< ", Neighbor Node's Current Location: " << map_entry.second.getNeighborNodeLocation() <<
				", Neighbor Next Location: " << map_entry.second.getNextLoc() << ", Neighbor Next Time Interval: " << map_entry.second.getNextTime() <<
				", Link LifeTime: " << map_entry.second.getLinkLifeTime() << "s, Current Processing Speed: " <<
				map_entry.second.getCurrProSpeed() << "GHz\n");
	}
	NS_LOG_DEBUG("============================Table Printing Ended======================================\n");
}

bool
RTable::DeleteRoute (Ipv4Address dst)
{
  if (m_ipv4AddressEntry.erase (dst) != 0)
    {
       NS_LOG_DEBUG("Route erased");
      return true;
    }
  return false;
}

uint32_t
RTable::RTableSize ()
{
  return m_ipv4AddressEntry.size ();
}

bool
RTable::AddRoute (RTableEntry & rt)
{
  m_ipv4AddressEntry.insert (std::make_pair (rt.getDestAddress(), rt));
  return true;
}

bool
RTable::Update (RTableEntry & rt)
{

  typedef std::multimap<Ipv4Address, RTableEntry>::iterator MMapIt;
  std::pair<MMapIt, MMapIt> result = m_ipv4AddressEntry.equal_range(rt.getDestAddress());
//  if (i == m_ipv4AddressEntry.end ())
//    {
//      return false;
//    }
  for(MMapIt it = result.first; it!=result.second;it++)
  {
      if(it->second.getMyAddress() == rt.getMyAddress())
  {
      it->second = rt;
      return true;
  }
  }

      return false;

}

void
RTableEntry::Print (Ptr<OutputStreamWrapper> stream) const
{
  *stream->GetStream () << std::setiosflags (std::ios::fixed) << m_myAddress << "\t" << m_destAddress << "\t" << std::setw (3) << std::setprecision (2)
                 <<  m_myLocation  << "\t" << std::setw (3) << std::setprecision (2) << m_neighborNodeLocation << "\t" << std::setw (3) << std::setprecision (2)
				<< m_timeConnected << "s" << "\t\t" << std::setw (2)  << m_nextLoc << "\t" << std::setw (2) << m_nextTime << "\t  " << m_linkLifeTime << "s\t"
				<< std::setprecision (2) << m_timePktRcvd.GetSeconds () << "s\t" << std::setw (2) << "\t" << std::setprecision (2) << m_currProSpeed << "GHz\n";
}

void
RTable::Print (Ptr<OutputStreamWrapper> stream, Ipv4Address nodeIP) const
{
  *stream->GetStream () << "\nRouting table\n" << "MyAddress\tDestination\tMyCurrLoc\t\tDestCurrLoc\t\tTimeConctd  NextLoc  NextIntrvl LinkLifeTime  TimePktRcvd"
		  "\tCurrProSpeed\n";
  for (std::map<Ipv4Address, RTableEntry>::const_iterator i = m_ipv4AddressEntry.begin (); i
       != m_ipv4AddressEntry.end (); ++i)
    {
      if(i->second.getMyAddress() == nodeIP)
         i->second.Print (stream);
    }
  *stream->GetStream () << "\n";
}

//void
//RTable::PrintRoutingTableAllAt (Time printTime, Ptr<OutputStreamWrapper> stream, Time::Unit unit)
//{
//  for (uint32_t i = 0; i < NodeList::GetNNodes (); i++)
//    {
//      Ptr<Node> node = NodeList::GetNode (i);
//      Simulator::Schedule (printTime, &RTable::Print, node, stream, unit);
//    }
//}
//
//void
//RTable::PrintRoutingTableAt (Time printTime, Ptr<Node> node, Ptr<OutputStreamWrapper> stream)
//{
//  Simulator::Schedule (printTime, &RTable::Print, node, stream);
//}
}

