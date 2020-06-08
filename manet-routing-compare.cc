/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2011 University of Kansas
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Justin Rohrer <rohrej@ittc.ku.edu>
 *
 * James P.G. Sterbenz <jpgs@ittc.ku.edu>, director
 * ResiliNets Research Group  http://wiki.ittc.ku.edu/resilinets
 * Information and Telecommunication Technology Center (ITTC)
 * and Department of Electrical Engineering and Computer Science
 * The University of Kansas Lawrence, KS USA.
 *
 * Work supported in part by NSF FIND (Future Internet Design) Program
 * under grant CNS-0626918 (Postmodern Internet Architecture),
 * NSF grant CNS-1050226 (Multilayer Network Resilience Analysis and Experimentation on GENI),
 * US Department of Defense (DoD), and ITTC at The University of Kansas.
 */

/*
 * This example program allows one to run ns-3 DSDV, AODV, or OLSR under
 * a typical random waypoint mobility model.
 *
 * By default, the simulation runs for 200 simulated seconds, of which
 * the first 50 are used for start-up time.  The number of nodes is 50.
 * Nodes move according to RandomWaypointMobilityModel with a speed of
 * 20 m/s and no pause time within a 300x1500 m region.  The WiFi is
 * in ad hoc mode with a 2 Mb/s rate (802.11b) and a Friis loss model.
 * The transmit power is set to 7.5 dBm.
 *
 * It is possible to change the mobility and density of the network by
 * directly modifying the speed and the number of nodes.  It is also
 * possible to change the characteristics of the network by changing
 * the transmit power (as power increases, the impact of mobility
 * decreases and the effective density increases).
 *
 * By default, OLSR is used, but specifying a value of 2 for the protocol
 * will cause AODV to be used, and specifying a value of 3 will cause
 * DSDV to be used.
 *
 * By default, there are 10 source/sink data pairs sending UDP data
 * at an application rate of 2.048 Kb/s each.    This is typically done
 * at a rate of 4 64-byte packets per second.  Application data is
 * started at a random time between 50 and 51 seconds and continues
 * to the end of the simulation.
 *
 * The program outputs a few items:
 * - packet receptions are notified to stdout such as:
 *   <timestamp> <node-id> received one packet from <src-address>
 * - each second, the data reception statistics are tabulated and output
 *   to a comma-separated value (csv) file
 * - some tracing and flow monitor configuration that used to work is
 *   left commented inline in the program
 */

#include <fstream>
#include <iostream>
#include <thread>
#include <chrono>
#include <string>
#include <cmath>
#include <ctime>
#include <queue>
#include <utility>
#include <functional>
#include <random>
#include <algorithm>
#include <iterator>
#include <vector>
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/aodv-module.h"
#include "ns3/olsr-module.h"
#include "ns3/dsdv-module.h"
#include "ns3/dsr-module.h"
#include "ns3/applications-module.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/data-rate.h"
#include "ns3/wifi-module.h"
#include "ns3/ipv4.h"
#include "ns3/discovery-packet-header.h"
#include "ns3/reply-packet-header.h"
#include "ns3/discovery-application.h"
#include "ns3/markovchain-mobility-model.h"
#include "ns3/myrtable.h"


using namespace ns3;
using namespace dsr;


NS_LOG_COMPONENT_DEFINE ("ManetRoutingCompare");

struct TaskDetails
{
    Time assignTime;
    bool success;
    Time dataTransferStart;
    Time dataTransferCompleted;
};

class RoutingExperiment
{
public:
	RoutingExperiment ();
	void Run (int nSinks, double txp, std::string CSVfileName);
	//static void SetMACParam (ns3::NetDeviceContainer & devices,
	//                                 int slotDistance);
	std::string CommandSetup (int argc, char **argv);

private:
	Ptr<Socket> SetupPacketReceive (Ipv4Address addr, Ptr<Node> node);
	Ptr<Socket> SetupPacketReceiveWD (Ipv4Address addr, Ptr<Node> node);
	Ptr<Socket> SetupDiscoveryReceive (Ipv4Address addr, Ptr<Node> node);
	Ptr<Socket> SetupDiscoveryReceiveWD (Ipv4Address addr, Ptr<Node> node);
	Ptr<Socket> SetupReplyReceive (Ipv4Address addr, Ptr<Node> node);
	Ptr<Socket> SetupReplyReceiveWD (Ipv4Address addr, Ptr<Node> node);
	void ReceivePacket (Ptr<Socket> socket);
	void ReceivePacketWD (Ptr<Socket> socket);
	void ReceiveDiscovery (Ptr<Socket> socket);
	void ReceiveDiscoveryWD (Ptr<Socket> socket);
	void ReceiveReply (Ptr<Socket> socket);
	void ReceiveReplyWD (Ptr<Socket> socket);
	void CheckIfTaskCompleted(int sourceID, Ipv4Address dest, double dataSize);
	void CheckThroughput (uint16_t i);
	void RxWD (std::string context, Ptr <const Packet> packet, uint16_t channelFreqMhz,  WifiTxVector txVector, MpduInfo aMpdu, SignalNoiseDbm signalNoise);
	void Rx (std::string context, Ptr <const Packet> packet, uint16_t channelFreqMhz,  WifiTxVector txVector, MpduInfo aMpdu, SignalNoiseDbm signalNoise);
	void Tx (std::string context, Ptr <const Packet> packet, uint16_t channelFreqMhz,  WifiTxVector txVector, MpduInfo aMpdu);
	void TxWD (std::string context, Ptr <const Packet> packet, uint16_t channelFreqMhz,  WifiTxVector txVector, MpduInfo aMpdu);
	Ipv4Address GetDestAddress(Ptr<const Packet> p);
	Ipv4Address GetSourceAddress(Ptr<const Packet> p);
	void txApp(Ptr<const Packet> pkt, const Address &src, const Address & des);
	void txAppWD(Ptr<const Packet> pkt, const Address &src, const Address & des);
	void txDiscApp(Ptr<const Packet> pkt);
	void txDiscAppWD(Ptr<const Packet> pkt);
	void PrintDropWD();
	void PrintDrop();
	void MacTxDrop(Ptr<const Packet> p);
	void MacTxDropWD(Ptr<const Packet> p);
	void PhyTxDrop(Ptr<const Packet> p);
	void PhyTxDropWD(Ptr<const Packet> p);
	void PhyRxDropWD(Ptr<const Packet> p, ns3::WifiPhyRxfailureReason reason);
	void PhyRxDrop(Ptr<const Packet> p, ns3::WifiPhyRxfailureReason reason);
	std::vector<std::string> Explode(const std::string& str, const char& ch);
	void LinkLifeTimer();
	void CourseChange (std::string context, Ptr<const MobilityModel> model);
	void AllocateAndSend(int nodeID, double tDataSize, double tDeadLine, bool maxProcSpeed);
	void StartTaskGeneration();
	void GenerateTasks();
	double TimeIntervalToTime(uint16_t interval);
	void InstallApplicationsW (Ptr<Node> source, Ipv4Address des, double simTime, double dataRate);
	void InstallApplicationsWD (Ptr<Node> source, Ipv4Address des, double simTime, double dataRate);
	void LocationDetector(uint16_t myLoc, uint16_t neighLoc, Ipv4Address myAddress, Ipv4Address src_ip);
	template <typename T> void PopulateQueue(T &user_task_queue);
	template<typename T> void PrintQueue(T& q);
	double ProSpeedGen (double minRange, double maxRange);
	void PrintRoutingTableW (Ptr<OutputStreamWrapper> stream, Ptr<Socket> socket, uint16_t id) const;
	void PrintRoutingTableWD (Ptr<OutputStreamWrapper> stream, Ptr<Socket> socket, uint16_t id) const;
	void PrintRoutingTable();

	static constexpr uint16_t nNodes = 5;
	uint32_t tasksAssigned[nNodes];
	uint32_t tasksFailed[nNodes];
	NodeContainer adhocNodes;
	uint32_t port;
	uint32_t portWD;
	uint32_t bytesTotalDisc[nNodes] = {};
	uint32_t packetsReceivedDisc[nNodes] = {};
	uint32_t bytesTotalWDDisc[nNodes] = {};
	uint32_t packetsReceivedWDDisc[nNodes] = {};
	uint32_t bytesTotalApp[nNodes] = {};
	uint32_t packetsReceivedApp[nNodes] = {};
	uint32_t bytesTotalWDApp[nNodes] = {};
	uint32_t packetsReceivedWDApp[nNodes] = {};
	std::string m_CSVfileName;
	uint32_t m_nSinks;
	std::string m_protocolName;
	std::string rtRcvW, rtDiscW,rtRcvWD, rtDiscWD;
	double m_txp;
	bool m_traceMobility;
	uint32_t m_protocol;
	uint32_t currentSeqNo[nNodes] = {};
	bool m_firstTime[nNodes] = {true,true,true,true,true};
	long double delay[nNodes] = {};
	long double rcv[nNodes] = {};
	long double sqhd[nNodes] = {};
	std::map<Ipv4Address, Ipv4Address> m_interfaceMap;
	uint32_t currentSeqNoWD[nNodes] = {};
	long double delayWD[nNodes] = {};
	long double rcvWD[nNodes] = {};
	long double sqhdWD[nNodes] = {};
	uint32_t m_nSources;
	uint32_t MacTxDropCount = 0;
	uint32_t PhyTxDropCount = 0;
	uint32_t PhyRxDropCount = 0;
	uint32_t MacTxDropCountWD = 0;
	uint32_t PhyTxDropCountWD = 0;
	uint32_t PhyRxDropCountWD = 0;
	std::map<Ipv4Address, uint64_t>appPktSend;
	std::map<Ipv4Address, uint64_t>appPktWDSend;
	std::map<Ipv4Address, uint64_t>appPktRec;
	std::map<Ipv4Address, uint64_t>appPktWDRec;
	uint64_t discPkt = 0;
	uint64_t discPktWD = 0;

	uint64_t counterTX[nNodes] = {};
	uint64_t counterRX[nNodes] = {};
	uint64_t counterAppTX[nNodes] = {};
	uint64_t counterAppRX[nNodes] = {};

	uint64_t counterTXWD[nNodes] = {};
	uint64_t counterRXWD[nNodes] = {};
	uint64_t counterAppTXWD[nNodes] = {};
	uint64_t counterAppRXWD[nNodes] = {};

	uint64_t m_txDataRate[nNodes] = {};
	uint64_t m_rxDataRate[nNodes] = {};
	uint64_t m_txDataRateWD[nNodes] = {};
	uint64_t m_rxDataRateWD[nNodes] = {};

	double m_throughputDisc[nNodes] = {};
	double m_throughputWDDisc[nNodes] = {};
	double m_throughputApp[nNodes] = {};
	double m_throughputWDApp[nNodes] = {};
	double mbsApp[nNodes] = {};
	double mbsDisc[nNodes] = {};
	double mbsWDDisc[nNodes] = {};
	double mbsWDApp[nNodes] = {};
	uint64_t m_timer = 0;
	double m_timeFirstPktAppSent[nNodes] = {};
	double m_timeFirstPktAppWDSent[nNodes] = {};
	double m_timeFirstPktDiscSent[nNodes] = {};
	double m_timeFirstPktDiscWDSent[nNodes] = {};
	bool m_firstTimeAppPktSent[nNodes] = {true, true, true, true, true};
	bool m_firstTimeDiscPktSent[nNodes] = {true, true, true, true, true};
	bool m_firstTimeAppWDPktSent[nNodes] = {true, true, true, true, true};
	bool m_firstTimeDiscWDPktSent[nNodes] = {true, true, true, true, true};
	RTable m_rTableWD, m_rTableW;
	Ptr<Socket> sink, sinkWD;
	Ptr<Socket> DiscoverySink, DiscoverySinkWD;
	Ptr<Socket> ReplySink, ReplySinkWD;
    int m_NodeId;
    std::vector<std::vector<TaskDetails>> allTasks;

};

class UserTask
{
public:
	UserTask() {
	}
	UserTask(std::uint16_t deadline, std::uint16_t task_size, std::uint16_t task_id): m_deadline(deadline), m_task_size(task_size), m_task_id(task_id){
	}
	void print_id(){
		std::cout<< "Running task: Task ID: " << m_task_id << ", Task Size: " << m_task_size <<"MB, Task Deadline: " << m_deadline << "s" <<std::endl;
	}

	std::uint16_t get_deadline(){
		return m_deadline;
	}
	std::uint16_t get_task_size(){
		return m_task_size;
	}
	std::uint16_t get_task_id(){
		return m_task_id;
	}

private:
	std::uint16_t m_deadline;
	std::uint16_t m_task_size;
	std::uint16_t m_task_id;
};


auto cmp_deadline = [](UserTask left, UserTask right) {
	return (left.get_deadline()) > (right.get_deadline()); };
using deadline_priority_queue = std::priority_queue<UserTask, std::vector<UserTask>, decltype(cmp_deadline)>;
deadline_priority_queue ordered_queue(cmp_deadline);

template<typename T> void RoutingExperiment::PrintQueue(T& q){
	while(!q.empty()) {
		auto task = q.top();
		task.print_id();
		q.pop();
	}
}
template <typename T>
void RoutingExperiment::PopulateQueue(T &user_task_queue){
	srand((int) time(NULL));
    uint32_t taskSizeArray[5] = {110, 300, 190, 750, 150};
    uint32_t taskDeadlineArray[5] = {10, 20, 15, 30, 12};
    int i = 0;
	while (i < 5){
        uint32_t ds = taskSizeArray[i];
        NS_LOG_DEBUG("Statically generated data size: " << ds << "MB, taskID: " << i);
        uint32_t dl = taskDeadlineArray[i];
        NS_LOG_DEBUG("Statically generated data transfer deadline: " << dl << "s, taskID: " << i);
		user_task_queue.push(UserTask(dl,ds,i));
		i++;
		std::this_thread::sleep_for(std::chrono::milliseconds(rand() % 2000 + 1));
	}
	NS_LOG_DEBUG("User Task Queue Size: " << user_task_queue.size());
}

double
RoutingExperiment::ProSpeedGen (double minRange, double maxRange)
{
	std::mt19937 rng;
	std::uniform_real_distribution < double >dist (minRange, maxRange);	//(min, max)
	rng.seed (std::random_device
			{
			} ());		//non-deterministic seed
	return dist (rng);
}


void RoutingExperiment::PrintRoutingTable()
{
	int j = 0;
	std::stringstream oss, oss1;
	oss << "Routing_TableW.routes";
	oss1 << "Routing_TableWD.routes";
	Ptr<OutputStreamWrapper> streamW= Create<OutputStreamWrapper>(oss.str(), std::ios::app);
	Ptr<OutputStreamWrapper> streamWD= Create<OutputStreamWrapper>(oss1.str(), std::ios::app);
	for (std::map<Ipv4Address, Ipv4Address>::const_iterator i = m_interfaceMap.begin (); i
	!= m_interfaceMap.end (); ++i)
	{
		*streamW->GetStream () << "Node: " << ++j
				<< ", Time: " << Simulator::Now().GetSeconds()
				<< ", Routing table" << std::endl;
		m_rTableW.Print (streamW, i->first);
		*streamW->GetStream () << std::endl;

		*streamWD->GetStream () << "Node: " << ++j
				<< ", Time: " << Simulator::Now().GetSeconds()
				<< ", Routing table" << std::endl;
		m_rTableWD.Print (streamWD, i->second);
		*streamWD->GetStream () << std::endl;

	}
	Simulator::Schedule(Seconds(10), &RoutingExperiment::PrintRoutingTable, this);
}

void
RoutingExperiment::CheckIfTaskCompleted(int sourceID, Ipv4Address dest, double dataSize)
{
	Ptr<Node> source = NodeList::GetNode(sourceID);
	Ipv4Address sourceIPW = source->GetObject<Ipv4>()->GetAddress(1,0).GetLocal();
	Ipv4Address sourceIPWD = source->GetObject<Ipv4>()->GetAddress(2,0).GetLocal();
	Ipv4Address destIPW = dest;
	Ipv4Address destIPWD = m_interfaceMap[dest];

	uint64_t packetsSent = appPktSend[sourceIPW] + appPktWDSend[sourceIPWD];
	uint64_t packetsRec = appPktRec[destIPW] + appPktWDRec[destIPWD];
	NS_LOG_DEBUG("Packets sent: "<< packetsSent);
	NS_LOG_DEBUG("Packets received: "<< packetsRec);
	appPktSend[sourceIPW] = 0;
	appPktWDSend[sourceIPWD] = 0;
	appPktRec[destIPW] = 0;
	appPktWDRec[destIPWD] = 0;
}

void
RoutingExperiment::InstallApplicationsW (Ptr<Node> source, Ipv4Address des, double simTime, double dataRate)
{
	std::string rate = std::to_string(dataRate)+"Mbps";
	NS_LOG_DEBUG("Sending the task through W");
	OnOffHelper onoff1 ("ns3::UdpSocketFactory", Address (InetSocketAddress (des, port + 1)));
	NS_LOG_DEBUG("Interface Address of WiFi: " << des << "and Port Number: " << port + 1);
	onoff1.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1.0]"));
	onoff1.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0.0]"));
	onoff1.SetAttribute ("DataRate", DataRateValue(DataRate(rate)));
	onoff1.SetAttribute ("PacketSize", StringValue("1024"));
	ApplicationContainer apps = onoff1.Install (source);
	apps.Start (Seconds(0));
	NS_LOG_DEBUG( "Wifi App Started" );
	NS_LOG_DEBUG("WiFi Interface APP Port Number: " << port);
	apps.Stop (Seconds(simTime));
	NS_LOG_DEBUG( "Wifi App Stopped at: " << simTime );
	int nApplications = source->GetNApplications();
	int nodeIndex = source->GetId();
	std::stringstream oss;
	oss << "/NodeList/" << nodeIndex <<"/ApplicationList/" << nApplications - 1 <<"/$ns3::OnOffApplication/TxWithAddresses";
	Config::ConnectWithoutContext(oss.str(), MakeCallback(&RoutingExperiment::txApp, this));
}

void RoutingExperiment::InstallApplicationsWD(Ptr<Node> source, Ipv4Address des, double simTime, double dataRate)
{
	std::string rate = std::to_string(dataRate)+"Mbps";
	NS_LOG_DEBUG("Sending the task through WD");
	OnOffHelper onoff1 ("ns3::UdpSocketFactory", Address (InetSocketAddress (des, portWD + 1)));
	NS_LOG_DEBUG("Interface Address of WiFi: " << des << " and Port Number: " << portWD + 1);
	onoff1.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1.0]"));
	onoff1.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0.0]"));
	onoff1.SetAttribute ("DataRate", StringValue (rate));
	onoff1.SetAttribute ("PacketSize", StringValue("1024"));
	ApplicationContainer apps = onoff1.Install (source);
	apps.Start (Seconds(0));
	NS_LOG_DEBUG( "Wifi App Started" );
	NS_LOG_DEBUG("WiFi Interface APP Port Number: " << portWD);
	apps.Stop (Seconds(simTime));
	NS_LOG_DEBUG( "Wifi App Stopped at: " << simTime );
	int nApplications = source->GetNApplications();
	int nodeIndex = source->GetId();
	std::stringstream oss;
	oss << "/NodeList/" << nodeIndex <<"/ApplicationList/" << nApplications - 1 <<"/$ns3::OnOffApplication/TxWithAddresses";
	Config::ConnectWithoutContext(oss.str(), MakeCallback(&RoutingExperiment::txAppWD, this));
}


RoutingExperiment::RoutingExperiment ()
: port (9),
  portWD (80),
  m_CSVfileName ("manet-routing.output.csv"),
  m_traceMobility (true),
  m_protocol (0),
  m_nSources(nNodes)// DSDV
{
    for(uint32_t i = 0;i<m_nSources;i++)
	{
		tasksAssigned[i] = 0;
		tasksFailed[i] = 0;
        std::vector<TaskDetails> temp;
        allTasks.push_back(temp);
	}
    m_NodeId = 0;
}

double
RoutingExperiment::TimeIntervalToTime(uint16_t interval)
{
	switch (interval)
	{
	case 0: return 10;
	case 1: return 30;
	case 2: return 60;
	default: return 10;
	}
}
void
RoutingExperiment::StartTaskGeneration()
{
	//	std::this_thread::sleep_for(std::chrono::milliseconds(rand() % 200 + 1));
	NS_LOG_DEBUG("Start Task Generation");
	PopulateQueue<deadline_priority_queue>(ordered_queue);
	GenerateTasks();
}

void
RoutingExperiment::GenerateTasks()
{
	NS_LOG_DEBUG("Generating Tasks");
	auto task = ordered_queue.top();
	double deadline = task.get_deadline();
	NS_LOG_DEBUG("Task ID: " << task.get_task_id() << ", deadline: " << deadline << "s");
	double dataSize = task.get_task_size();
	NS_LOG_DEBUG("DataSize: " << dataSize << "MB");
//	Ptr<UniformRandomVariable> x = CreateObject<UniformRandomVariable>();
//	x->SetAttribute("Min",DoubleValue(0));
//	x->SetAttribute("Max",DoubleValue(m_nSources-1));
//	int nodeID = x->GetInteger();
    int nodeID = m_NodeId++;
	NS_LOG_DEBUG("Node ID: " << nodeID << ", Data Size: " << dataSize << "MB, Deadline: " << deadline << "s");
	AllocateAndSend(nodeID, dataSize, deadline, false);
}

// lambda is evaluated at compile time using constexpr
//constexpr auto cmp_proSpeed =[] (RTableEntry left, RTableEntry right) {
//  return (left.getCurrProSpeed()) > (right.getCurrProSpeed());
//};
bool comparator(RTableEntry &left, RTableEntry &right)
{
	return (left.getCurrProSpeed() < right.getCurrProSpeed());
}

void
RoutingExperiment::AllocateAndSend(int nodeID, double tDataSize, double tDeadLine, bool maxProcSpeed)
{

    std::vector<RTableEntry> sortedRoutes;
    tasksAssigned[nodeID]++;
    TaskDetails thisTask;
    thisTask.assignTime = Simulator::Now();
	bool transferPossible = false;
	double speedW = m_txDataRate[nodeID]; //Mbps
	double speedWD = m_txDataRateWD[nodeID];
	double remainingData, finalData, totalData;
	double remainingTimeW = 0;
	double remainingTimeWD = 0;
	NS_LOG_DEBUG("Allocating the task");
	NS_LOG_DEBUG("DataRate W: " << m_txDataRate[nodeID] << "Mbps, DataRate WD: " << m_txDataRateWD[nodeID] << "Mbps");
	//	double proSpeed = std::max(cmp_proSpeed, header.GetCurrProSpeed());
	Ptr<Node> source = NodeList::GetNode(nodeID);
	Ipv4Address sourceIPW = source->GetObject<Ipv4>()->GetAddress(1,0).GetLocal();
	Ipv4Address sourceIPWD = source->GetObject<Ipv4>()->GetAddress(2,0).GetLocal();
	std::map<Ipv4Address, RTableEntry> allRoutesW = m_rTableW.GetAllRoutesWithIP(sourceIPW);
	std::map<Ipv4Address, RTableEntry> allRoutesWD = m_rTableWD.GetAllRoutesWithIP(sourceIPWD);
	for (auto const entry: allRoutesW) {
		sortedRoutes.push_back(entry.second);
	}
	if (maxProcSpeed == true)
	{
		std::sort(sortedRoutes.begin(), sortedRoutes.end(), &comparator);
	}
	uint16_t rtWSize = allRoutesW.size();
	uint16_t rtWDSize = allRoutesWD.size();
	double availableBWW = 0.0;
	double availableBWWD = 0.0;
	NS_LOG_DEBUG("Routing table W Size: " << rtWSize << ", Routing Table WD Size: " << rtWDSize);

	availableBWW = speedW -(mbsApp[nodeID] + mbsDisc[nodeID]);
	availableBWWD = speedWD - (mbsWDApp[nodeID] + mbsWDDisc[nodeID]);
	NS_LOG_DEBUG("Available BW on W: " << availableBWW << "Mbps, Available BW on WD: " << availableBWWD << "Mbps");

	if (availableBWWD > 0 && availableBWW > 0){
		//for (auto const map_entry : allRoutesW)
		for(uint16_t k = 0; k < sortedRoutes.size(); k++)
		{
			RTableEntry entry = sortedRoutes[k];
			NS_LOG_DEBUG("Map Entry First: " << entry.getDestAddress().GetAny() << ", Map Entry Second: " << entry.getMyAddress());
			NS_LOG_DEBUG("Inside AllRoutes W");
			double T_DT_W = (tDataSize/availableBWW);
			NS_LOG_DEBUG("Data Transfer Time W: " << T_DT_W);
			double T_DT_WD = (tDataSize/availableBWWD);
			NS_LOG_DEBUG("Data Transfer Time WD: " << T_DT_WD);
			double aW, aWD;

            Ipv4Address ipWD = m_interfaceMap[entry.getDestAddress()];

            if(rtWSize == 0){
            	if(T_DT_WD > tDeadLine){
            		NS_LOG_DEBUG("This task cannot be allocated ");
            		continue;
            	}
            }

            if(rtWDSize == 0){
            	if(T_DT_W > tDeadLine){
            		NS_LOG_DEBUG("This task cannot be allocated ");
            		continue;
            	}
            }


            if(maxProcSpeed == true)
			{
				aW = tDeadLine;
				aWD = tDeadLine;
			}
			else
			{
				aW = std::min(tDeadLine, entry.getLinkLifeTime());
				NS_LOG_DEBUG("Minimum value among deadline or link lifetime at W: " << aW);
                if(allRoutesWD.find(ipWD) == allRoutesWD.end())
                    continue;
                aWD = std::min(tDeadLine, allRoutesWD.at(ipWD).getLinkLifeTime());
				NS_LOG_DEBUG("Minimum value among deadline or link lifetime at WD: " << aWD);
			}

			double maxDataW = availableBWW * aW;
			double maxDataWD = availableBWWD * aWD;
			if (maxDataW > tDataSize)
				maxDataW = tDataSize;

			if (maxDataWD > tDataSize)
				maxDataWD = tDataSize;

			if(maxDataWD > maxDataW){
				remainingData = tDataSize - maxDataWD;
				remainingTimeW = remainingData*(1/availableBWW);
				finalData = maxDataWD;
				totalData = remainingData + maxDataWD;
			} else {
				remainingData = tDataSize - maxDataW;
				remainingTimeWD = remainingData*(1/availableBWWD);
				finalData = maxDataW;
				totalData = remainingData + maxDataW;
			}

			double remainingTime = remainingTimeW + remainingTimeWD;

			NS_LOG_DEBUG("IPWD: " << ipWD << ", Source IP: " << source->GetId());

			if(totalData < tDataSize || remainingTime > tDeadLine){
				NS_LOG_DEBUG("This data transfer is not possible");
				continue;
			}
			else {
				transferPossible = true;
                thisTask.success = true;
                thisTask.dataTransferStart = Simulator::Now();

                Simulator::Schedule(Seconds(tDeadLine), &RoutingExperiment::CheckIfTaskCompleted, this, nodeID, entry.getDestAddress(), tDataSize);
                if(maxDataWD >= maxDataW) {
                    if(T_DT_WD < tDeadLine){
						NS_LOG_DEBUG("Sending the maximum data " << finalData << " to node using Wi-Fi Direct for the time: " << T_DT_WD << "s");
						InstallApplicationsWD (source, ipWD, T_DT_WD, availableBWWD);
                        thisTask.dataTransferCompleted = Simulator::Now() + Seconds(T_DT_WD);
                        ordered_queue.pop();
					} else {
                        thisTask.dataTransferCompleted = Simulator::Now() + Seconds(std::max(aWD, remainingTimeW ));
						NS_LOG_DEBUG("Sending the maximum data " << finalData << " to node using Wi-Fi Direct for the time: " << aWD << "s");
						InstallApplicationsWD (source, ipWD, aWD, availableBWWD);
						ordered_queue.pop();
					}
					if(remainingData == 0){
						NS_LOG_DEBUG("No need to use another WCT");
					} else {
						NS_LOG_DEBUG("Sending the remaining data " << remainingData << " to node using Wi-Fi for the time: " << remainingTimeW << "s");
						InstallApplicationsW (source, entry.getDestAddress(), remainingTimeW, availableBWW);
						//ordered_queue.pop();
					}

				} else {
					if(T_DT_W < tDeadLine) {
						NS_LOG_DEBUG("Sending the maximum data " << finalData << " to node using Wi-Fi for the time: " << T_DT_W << "s");
                        thisTask.dataTransferCompleted = Simulator::Now() + Seconds(T_DT_W);
						InstallApplicationsW (source, entry.getDestAddress(), T_DT_W, availableBWW);
						ordered_queue.pop();
					} else {
						NS_LOG_DEBUG("Sending the maximum data " << finalData << " to node using Wi-Fi for the time: " << aW << "s" );
                        thisTask.dataTransferCompleted = Simulator::Now() + Seconds(std::max(aW, remainingTimeWD ));
						InstallApplicationsW (source, entry.getDestAddress(), aW, availableBWW);
						ordered_queue.pop();
					}
					if(remainingData == 0){
						NS_LOG_DEBUG( "No need to use another WCT" );
					} else {
						NS_LOG_DEBUG("Sending the remaining data " << remainingData << " to node using Wi-Fi Direct" << remainingTimeWD << "s" );
						InstallApplicationsWD (source,ipWD,remainingTimeWD, availableBWWD);
						//ordered_queue.pop();
					}
				}
			}
			if(transferPossible == true)
				break;
		}
	}

	if(transferPossible == false)
	{
		tasksFailed[nodeID]++;
        thisTask.success = false;
		ordered_queue.pop();
	}
    allTasks[nodeID].push_back(thisTask);
	if (ordered_queue.size() != 0)
	{
		Ptr<UniformRandomVariable> x = CreateObject<UniformRandomVariable>();
		x->SetAttribute("Min",DoubleValue(0));
        x->SetAttribute("Max",DoubleValue(10));
		int time = x->GetInteger();
		Simulator::Schedule(Seconds(time), &RoutingExperiment::GenerateTasks, this);
	}
}
std::vector<std::string> RoutingExperiment::Explode(const std::string& str, const char& ch){
	std::string next;
	std::vector<std::string> result;
	// For each character in the string
	for (std::string::const_iterator it = str.begin(); it != str.end(); it++) {
		// If we've hit the terminal character
		if (*it == ch) {
			// If we have some characters accumulated
			if (!next.empty()) {
				// Add them to the result vector
				result.push_back(next);
				next.clear();
			}
		} else {
			// Accumulate the next character into the sequence
			next += *it;
		}
	}
	if (!next.empty())
		result.push_back(next);
	return result;
}

Ipv4Address RoutingExperiment::GetSourceAddress(Ptr<const Packet> p)
{
	Ipv4Address src;
	// To get a header from Ptr<Packet> packet first, copy the packet
	Ptr<Packet> q = p->Copy();
	// Use indicator to search the packet
	PacketMetadata::ItemIterator metadataIterator = q->BeginItem();
	PacketMetadata::Item item;
	while (metadataIterator.HasNext())
	{
		item = metadataIterator.Next();
		NS_LOG_FUNCTION("item name: " << item.tid.GetName());
		// If we want to have a dsdv header
		if(item.tid.GetName() == "ns3::Ipv4Header")
		{
			Callback<ObjectBase *> constr = item.tid.GetConstructor();
			NS_ASSERT(!constr.IsNull());
			// Ptr<> and DynamicCast<> won't work here as all headers are from ObjectBase, not Object
			ObjectBase *instance = constr();
			NS_ASSERT(instance != 0);
			Ipv4Header* ipv4Header = dynamic_cast<Ipv4Header*>(instance);
			NS_ASSERT(ipv4Header != 0);
			ipv4Header->Deserialize(item.current);
			src = ipv4Header->GetSource();
			NS_LOG_DEBUG("Source IP ADDRESS FOUND from IPV4Header: " << src);
			break;
		}
		else if(item.tid.GetName() == "ns3::ArpHeader")
		{
			Callback<ObjectBase *> constr = item.tid.GetConstructor();
			NS_ASSERT(!constr.IsNull());
			// Ptr<> and DynamicCast<> won't work here as all headers are from ObjectBase, not Object
			ObjectBase *instance = constr();
			NS_ASSERT(instance != 0);
			ArpHeader* arpHeader = dynamic_cast<ArpHeader*>(instance);
			NS_ASSERT(arpHeader != 0);
			arpHeader->Deserialize(item.current);
			src = arpHeader->GetSourceIpv4Address();
			NS_LOG_DEBUG("Source IP ADDRESS FOUND from ARPHeader: " << src);
			break;
		}
		else
		{
			NS_LOG_DEBUG("NO SOURCE IP ADDRESS FOUND");
			//break;
		}
	}
	return src;
}

Ipv4Address RoutingExperiment::GetDestAddress(Ptr<const Packet> p)
{
	Ipv4Address dest;
	// To get a header from Ptr<Packet> packet first, copy the packet
	Ptr<Packet> q = p->Copy();
	// Use indicator to search the packet
	PacketMetadata::ItemIterator metadataIterator = q->BeginItem();
	PacketMetadata::Item item;
	while (metadataIterator.HasNext())
	{
		item = metadataIterator.Next();
		NS_LOG_FUNCTION("item name: " << item.tid.GetName());
		// If we want to have a dsdv header
		if(item.tid.GetName() == "ns3::Ipv4Header")
		{
			Callback<ObjectBase *> constr = item.tid.GetConstructor();
			NS_ASSERT(!constr.IsNull());
			// Ptr<> and DynamicCast<> won't work here as all headers are from ObjectBase, not Object
			ObjectBase *instance = constr();
			NS_ASSERT(instance != 0);
			Ipv4Header* ipv4Header = dynamic_cast<Ipv4Header*>(instance);
			NS_ASSERT(ipv4Header != 0);
			ipv4Header->Deserialize(item.current);
			dest = ipv4Header->GetDestination();
			NS_LOG_DEBUG("Destination IP ADDRESS FOUND from IPv4 Header: " << dest);
			break;
		}
		else if(item.tid.GetName() == "ns3::ArpHeader")
		{
			Callback<ObjectBase *> constr = item.tid.GetConstructor();
			NS_ASSERT(!constr.IsNull());
			// Ptr<> and DynamicCast<> won't work here as all headers are from ObjectBase, not Object
			ObjectBase *instance = constr();
			NS_ASSERT(instance != 0);
			ArpHeader* arpHeader = dynamic_cast<ArpHeader*>(instance);
			NS_ASSERT(arpHeader != 0);
			arpHeader->Deserialize(item.current);
			dest = arpHeader->GetDestinationIpv4Address();
			NS_LOG_DEBUG("Destination IP ADDRESS FOUND from ARPHeader: " << dest);
			break;
		}
		else
		{
			NS_LOG_DEBUG("NO Destination IP ADDRESS FOUND");
			//break;
		}
	}
	return dest;
}

void RoutingExperiment::txApp(Ptr<const Packet> pkt,  const Address &src, const Address & des){
	NS_LOG_DEBUG (Simulator::Now().GetSeconds() << "\t Transmitting Application Packet: ");
	//    InetSocketAddress dest = InetSocketAddress::ConvertFrom(des);
	//    appPktSend[dest.GetIpv4()]++;
}

void RoutingExperiment::txAppWD(Ptr<const Packet> pkt,  const Address &src, const Address & des){
	NS_LOG_DEBUG (Simulator::Now().GetSeconds() << "\t Transmitting Application Packet: ");
	//    InetSocketAddress dest = InetSocketAddress::ConvertFrom(des);
	//    appPktWDSend[dest.GetIpv4()]++;
}

void RoutingExperiment::txDiscApp(Ptr<const Packet> pkt){
	NS_LOG_DEBUG (Simulator::Now().GetSeconds() << "\t Transmitting Discovery Application Packet: " << discPkt);
	discPkt++;
}

void RoutingExperiment::txDiscAppWD(Ptr<const Packet> pkt){
	NS_LOG_DEBUG (Simulator::Now().GetSeconds() << "\t Transmitting Discovery Application Packet: " << discPktWD);
	discPktWD++;
}

static void
dataRate (uint64_t oldCurrRate, uint64_t newCurrRate)
{
	NS_LOG_DEBUG (Simulator::Now ().GetSeconds () << "\t Data Rate Previous: " << oldCurrRate << "\t Data Rate Now: " << newCurrRate);
}

void
RoutingExperiment::LocationDetector(uint16_t myLoc, uint16_t neighLoc, Ipv4Address myAddress, Ipv4Address src_ip)
{

	switch(myLoc){
	case 0:
		NS_LOG_DEBUG(myAddress << ", I am at L0");
		break;
	case 1:
		NS_LOG_DEBUG(myAddress << ", I am at L1");
		break;
	case 2:
		NS_LOG_DEBUG(myAddress << ", I am at L2");
		break;
	case 3:
		NS_LOG_DEBUG(myAddress << ", I am at L3");
		break;
	case 4:
		NS_LOG_DEBUG(myAddress << ", I am at L4");
		break;
	case 5:
		NS_LOG_DEBUG(myAddress << ", I am moving");
		break;
	default:
		NS_LOG_DEBUG(myAddress << ", I am at invalid Location");
	}

	switch(neighLoc){
	case 0:
		NS_LOG_DEBUG(src_ip << ", My neighbor is at L0");
		break;
	case 1:
		NS_LOG_DEBUG(src_ip << ", My neighbor is at L1");
		break;
	case 2:
		NS_LOG_DEBUG(src_ip << ", My neighbor is at L2");
		break;
	case 3:
		NS_LOG_DEBUG(src_ip << ", My neighbor is at L3");
		break;
	case 4:
		NS_LOG_DEBUG(src_ip << ", My neighbor is at L4");
		break;
	case 5:
		NS_LOG_DEBUG(src_ip << ", My neighbor is moving");
		break;
	default:
		NS_LOG_DEBUG(src_ip << ", My neighbor is at invalid Location");
	}

}


//Note: this is a promiscuous trace for all packet reception. This is also on physical layer, so packets still have WifiMacHeader
void
RoutingExperiment::Tx (std::string context, Ptr <const Packet> packet, uint16_t channelFreqMhz,  WifiTxVector txVector, MpduInfo aMpdu){
	//context will include info about the source of this event. Use string manipulation if you want to extract info.
	//std::cout << BOLD_CODE <<  context << END_CODE << std::endl;
	//Print the info.
	NS_LOG_DEBUG("TxW-------------------------------------------------------");
	NS_LOG_DEBUG("ContextW: " << context << "Packet size: "<<packet->GetSize());
	std::vector<std::string> result = Explode(context, '/');
	uint32_t nodeId = std::stoi(result[1]);
	Ipv4Address myAddress = NodeList::GetNode(nodeId)->GetObject<Ipv4>()->GetAddress(1,0).GetLocal();
	if(packet->GetSize() >= 500){
		counterAppTX[nodeId]++;
		appPktSend[myAddress]++;
		if(m_firstTimeAppPktSent[nodeId]){
			m_timeFirstPktAppSent[nodeId] = Simulator::Now().GetSeconds();
			m_firstTimeAppPktSent[nodeId] = false;
		}
	} else {
		counterTX[nodeId]++;
		if(m_firstTimeDiscPktSent[nodeId]){
			m_timeFirstPktDiscSent[nodeId] = Simulator::Now().GetSeconds();
			m_firstTimeDiscPktSent[nodeId] = false;
		}
	}
	NS_LOG_DEBUG(" Size = " << packet->GetSize()
			<< " Freq = "<<channelFreqMhz
			<< " Mode = " << txVector.GetMode()
			<< " TransmissionDataRate = " << txVector.GetMode().GetDataRate(txVector)
			<< " TX Counter of Node: " << nodeId << ": " << counterTX[nodeId] << "\t TX APP Counter: " << counterAppTX[nodeId]);
	m_txDataRate[nodeId] = txVector.GetMode().GetDataRate(txVector)/1000000;

	//We can also examine the WifiMacHeader
	WifiMacHeader hdr;
	if (packet->PeekHeader(hdr))
	{
		NS_LOG_DEBUG("\tDestination MAC : " << hdr.GetAddr1() << "\tSource MAC : " << hdr.GetAddr2());
	}
}

void
RoutingExperiment::TxWD (std::string context, Ptr <const Packet> packet, uint16_t channelFreqMhz,  WifiTxVector txVector,MpduInfo aMpdu){
	//context will include info about the source of this event. Use string manipulation if you want to extract info.
	//Print the info.
	NS_LOG_DEBUG("TxWD-------------------------------------------------------");
	NS_LOG_DEBUG("ContextWD: " << context << "Packet size: "<<packet->GetSize());
	std::vector<std::string> result = Explode(context, '/');
	uint32_t nodeId = std::stoi(result[1]);
	Ipv4Address myAddress = NodeList::GetNode(nodeId)->GetObject<Ipv4>()->GetAddress(1,0).GetLocal();
	if(packet->GetSize() >= 500){
		counterAppTXWD[nodeId]++;
		appPktSend[myAddress]++;
		if(m_firstTimeAppWDPktSent[nodeId]){
			m_timeFirstPktAppWDSent[nodeId] = Simulator::Now().GetSeconds();
			m_firstTimeAppWDPktSent[nodeId] = false;
		}
	} else {
		counterTXWD[nodeId]++;
		if(m_firstTimeDiscWDPktSent[nodeId]){
			m_timeFirstPktDiscWDSent[nodeId] = Simulator::Now().GetSeconds();
			m_firstTimeDiscWDPktSent[nodeId] = false;
		}
	}
	NS_LOG_DEBUG("\tSize = " << packet->GetSize()
			<< " Freq = "<<channelFreqMhz
			<< " Mode = " << txVector.GetMode()
			<< " TransmissionDataRate = " << txVector.GetMode().GetDataRate(txVector)
			<< " TX Counter of Node: " << nodeId <<": " << counterTXWD[nodeId] << "\t TX APP Counter: " << counterAppTXWD[nodeId]);
	m_txDataRateWD[nodeId] = txVector.GetMode().GetDataRate(txVector)/1000000;

	//We can also examine the WifiMacHeader
	WifiMacHeader hdr;
	if (packet->PeekHeader(hdr))
	{
		NS_LOG_DEBUG("\tDestination MAC : " << hdr.GetAddr1() << "\tSource MAC : " << hdr.GetAddr2());
	}
}

void
RoutingExperiment::RxWD (std::string context, Ptr <const Packet> packet, uint16_t channelFreqMhz,  WifiTxVector txVector, MpduInfo aMpdu, SignalNoiseDbm signalNoise){
	//context will include info about the source of this event. Use string manipulation if you want to extract info.

	NS_LOG_DEBUG("RxWD-------------------------------------------------------");
	NS_LOG_DEBUG("ContextWD: " << context << "Packet size: "<<packet->GetSize());
	std::vector<std::string> result = Explode(context, '/');
	uint32_t nodeId = std::stoi(result[1]);
	uint32_t iface = std::stoi(result[3]);
	Ptr<Node> node = NodeList::GetNode(nodeId);
	Ptr<NetDevice> dev = node->GetDevice(iface);
	Ptr<Ipv4> ip = node->GetObject<Ipv4>();
	uint32_t ifaceId = ip->GetInterfaceForDevice(dev);
	Ipv4InterfaceAddress addr = ip->GetAddress(ifaceId, 0);
	Ipv4Address myAddress = addr.GetLocal();
	Ipv4Address src_ip = GetSourceAddress(packet);
	Ipv4Address des_ip = GetDestAddress(packet);
	//  double timeConnected = 0.0;

	Vector neighborNodeLocation;

	int32_t nNodes = NodeList::GetNNodes ();

	for (int32_t i = 0; i < nNodes; ++i)
	{
		Ptr<Node> node = NodeList::GetNode (i);
		Ptr<Ipv4> ipv4 = node->GetObject<Ipv4> ();

		int32_t ifIndex = ipv4->GetInterfaceForAddress (src_ip);
		if (ifIndex != -1)
		{
			Ptr<MobilityModel> neighborNodeModel = node->GetObject<MobilityModel>();
			neighborNodeLocation = neighborNodeModel->GetPosition();
		}
	}

	Vector myLocation;
	Ptr<MobilityModel> myMobilityModel = node->GetObject<MobilityModel>();
	myLocation = myMobilityModel->GetPosition();
	NS_LOG_DEBUG("WD My Location: " << myLocation << ", Neighbor Node Location: " << neighborNodeLocation);
	double distance = std::sqrt((myLocation.x-neighborNodeLocation.x) * (myLocation.x-neighborNodeLocation.x)+ (myLocation.y-neighborNodeLocation.y)
			* (myLocation.y-neighborNodeLocation.y));
	NS_LOG_DEBUG("WD Distance among us is: " << distance);

	Ptr<MarkovChainMobilityModel> model = CreateObject<MarkovChainMobilityModel>();
	uint16_t myLoc = model->PositionToLocation(myLocation);
	uint16_t neighLoc = model->PositionToLocation(neighborNodeLocation);

	LocationDetector(myLoc, neighLoc, myAddress, src_ip);


	if(packet->GetSize() >= 500){
		counterAppRXWD[nodeId]++;
		//		NS_LOG_DEBUG("WIFI Direct Packet Received from: " << src_ip << ", Destination address in Packet: " << des_ip);
	} else {
		counterRXWD[nodeId]++;
	}

	NS_LOG_DEBUG(myAddress <<" WD Received Packet from the Source Address: " << src_ip << ", With the Destination address in Packet: " << des_ip
			<< ", Size = " << packet->GetSize()
			<< ", Freq = "<<channelFreqMhz
			<< ", Mode = " << txVector.GetMode()
			<< ", ReceptionDataRate = " << txVector.GetMode().GetDataRate(txVector)
			<< ", RX Counter: " << counterRXWD[nodeId] << ",\t RX APP Counter: " << counterAppRXWD[nodeId]);
	m_rxDataRateWD[nodeId] = txVector.GetMode().GetDataRate(txVector)/1000000;

	//We can also examine the WifiMacHeader
	WifiMacHeader hdr;
	if (packet->PeekHeader(hdr))
	{
		NS_LOG_DEBUG("\tDestination MAC : " << hdr.GetAddr1() << "\tSource MAC : " << hdr.GetAddr2());
	}
}

void
RoutingExperiment::Rx (std::string context, Ptr <const Packet> packet, uint16_t channelFreqMhz,  WifiTxVector txVector, MpduInfo aMpdu, SignalNoiseDbm signalNoise){
	//context will include info about the source of this event. Use string manipulation if you want to extract info.

	NS_LOG_DEBUG("Rx-------------------------------------------------------");
	NS_LOG_DEBUG("ContextW: " << context << "Packet size: "<<packet->GetSize());
	std::vector<std::string> result = Explode(context, '/');
	uint32_t nodeId = std::stoi(result[1]);
	uint32_t iface = std::stoi(result[3]);
	Ptr<Node> node = NodeList::GetNode(nodeId);
	Ptr<NetDevice> dev = node->GetDevice(iface);
	Ptr<Ipv4> ip = node->GetObject<Ipv4>();
	uint32_t ifaceId = ip->GetInterfaceForDevice(dev);
	Ipv4InterfaceAddress addr = ip->GetAddress(ifaceId, 0);
	Ipv4Address myAddress = addr.GetLocal();
	Ipv4Address src_ip = GetSourceAddress(packet);
	Ipv4Address des_ip = GetDestAddress(packet);

	Vector neighborNodeLocation;

	int32_t nNodes = NodeList::GetNNodes ();

	for (int32_t i = 0; i < nNodes; ++i)
	{
		Ptr<Node> node = NodeList::GetNode (i);
		Ptr<Ipv4> ipv4 = node->GetObject<Ipv4> ();

		int32_t ifIndex = ipv4->GetInterfaceForAddress (src_ip);
		if (ifIndex != -1)
		{
			Ptr<MobilityModel> neighborNodeModel = node->GetObject<MobilityModel>();
			neighborNodeLocation = neighborNodeModel->GetPosition();
		}
	}

	Vector myLocation;
	Ptr<MobilityModel> myMobilityModel = node->GetObject<MobilityModel>();
	myLocation = myMobilityModel->GetPosition();
	NS_LOG_DEBUG("W My Location: " << myLocation << ", Neighbor Node Location: " << neighborNodeLocation);
	double distance = std::sqrt((myLocation.x-neighborNodeLocation.x) * (myLocation.x-neighborNodeLocation.x)+ (myLocation.y-neighborNodeLocation.y)
			* (myLocation.y-neighborNodeLocation.y));
	NS_LOG_DEBUG("W Distance among us is: " << distance);

	Ptr<MarkovChainMobilityModel> model = CreateObject<MarkovChainMobilityModel>();
	uint16_t myLoc = model->PositionToLocation(myLocation);
	uint16_t neighLoc = model->PositionToLocation(neighborNodeLocation);

	LocationDetector(myLoc, neighLoc, myAddress, src_ip);

	if(packet->GetSize() >= 500){
		counterAppRX[nodeId]++;

	} else {
		counterRX[nodeId]++;
	}
	//
	NS_LOG_DEBUG(myAddress << " W Received Packet from Source Address: " << src_ip << ", With Destination address in Packet: " << des_ip
			<< ", Size = " << packet->GetSize()
			<< ", Freq = "<<channelFreqMhz
			<< ", Mode = " << txVector.GetMode()
			<< ", ReceptionDataRate = " << txVector.GetMode().GetDataRate(txVector)
			<< ", RX Counter: " << counterRX[nodeId] << ",\t RX APP Counter: " << counterAppRX[nodeId]);
	m_rxDataRate[nodeId] = txVector.GetMode().GetDataRate(txVector)/1000000;

	//We can also examine the WifiMacHeader
	WifiMacHeader hdr;
	if (packet->PeekHeader(hdr))
	{
		NS_LOG_DEBUG("\tDestination MAC : " << hdr.GetAddr1() << "\tSource MAC : " << hdr.GetAddr2());
	}
}

void
RoutingExperiment::ReceivePacket (Ptr<Socket> socket)
{
	Ptr<Packet> packet;
	Address senderAddress;
	//
	NS_LOG_DEBUG("Sender Address before while: " << senderAddress);
	while ((packet = socket->RecvFrom (senderAddress)))
	{
		Ptr<Node> node = socket->GetNode();
		uint16_t nodeID = node->GetId();
		SeqTsHeader seqTsx;
		packet->RemoveHeader (seqTsx);
		currentSeqNo[nodeID] = seqTsx.GetSeq ();
		//m_rxDataRate = GetDRate(packet);

		bytesTotalApp[nodeID] += packet->GetSize ();
		packetsReceivedApp[nodeID] += 1;
		NS_LOG_DEBUG("Current Seq No.: " << currentSeqNo[nodeID]);
		NS_LOG_DEBUG("Bytes total received: " << bytesTotalApp[nodeID] );
		NS_LOG_DEBUG("Packets total received: " << packetsReceivedApp[nodeID]);
		//NS_LOG_UNCOND (PrintReceivedPacket (socket, packet, senderAddress));
		rcv[nodeID] = Simulator::Now().GetSeconds();
		sqhd[nodeID] = seqTsx.GetTs().GetSeconds();
		NS_LOG_DEBUG("Seq No " << currentSeqNo[nodeID] << " Packet Transmitted at: " << sqhd[nodeID] << "s, Packet Received at: " << rcv[nodeID] << "s" );//Just to check seq number and Tx time

		//InetSocketAddress sender = InetSocketAddress::ConvertFrom(senderAddress);
		Ipv4Address myIP = node->GetObject<Ipv4>()->GetAddress(1,0).GetLocal();
		appPktRec[myIP]++;
		//if(rcv>sqhd)
		delay[nodeID] = rcv[nodeID] - sqhd[nodeID]; //delay calculation

		NS_LOG_DEBUG("Delay: " << delay[nodeID] << "s");

	}
}


void
RoutingExperiment::ReceivePacketWD (Ptr<Socket> socket)
{
	Ptr<Packet> packet;
	Address senderAddress;

	while ((packet = socket->RecvFrom (senderAddress)))
	{
		Ptr<Node> node = socket->GetNode();
		uint16_t nodeID = node->GetId();
		SeqTsHeader seqTsx;
		packet->RemoveHeader (seqTsx);
		currentSeqNoWD[nodeID] = seqTsx.GetSeq ();
		//m_rxDataRate = GetDRate(packet);

		bytesTotalWDApp[nodeID] += packet->GetSize ();
		packetsReceivedWDApp[nodeID] += 1;
		NS_LOG_DEBUG("Current Seq No.: " << currentSeqNo[nodeID]);
		NS_LOG_DEBUG("Bytes total received: " << bytesTotalWDApp[nodeID] );
		NS_LOG_DEBUG("Packets total received: " << packetsReceivedWDApp[nodeID]);
		//NS_LOG_UNCOND (PrintReceivedPacketWD (socket, packet, senderAddress));
		rcvWD[nodeID] = Simulator::Now().GetSeconds();
		sqhdWD[nodeID] = seqTsx.GetTs().GetSeconds();
		NS_LOG_DEBUG("WD Seq No " << currentSeqNoWD[nodeID] << " Packet Transmitted at: " << sqhdWD[nodeID] << "s, Packet Received at: " << rcvWD[nodeID] << "s" );//Just to check seq number and Tx time

		// if(rcvWD>sqhdWD)
		delayWD[nodeID] = rcvWD[nodeID] - sqhdWD[nodeID]; //delay calculation
		//InetSocketAddress sender = InetSocketAddress::ConvertFrom(senderAddress);
		Ipv4Address myIP = node->GetObject<Ipv4>()->GetAddress(2,0).GetLocal();
		appPktWDRec[myIP]++;
		NS_LOG_DEBUG("Delay: " << delayWD[nodeID] << "s");
	}

}

void
RoutingExperiment::PrintRoutingTableW (Ptr<OutputStreamWrapper> stream, Ptr<Socket> socket, uint16_t id) const	//prints the routing table stream.
{

	int j = 0;
	for (std::map<Ipv4Address, Ipv4Address>::const_iterator i = m_interfaceMap.begin (); i
	!= m_interfaceMap.end (); ++i)
	{
		*stream->GetStream () << "Node: " << ++j
				<< ", Time: " << Simulator::Now().GetSeconds()
				<< ", Routing table" << std::endl;

		m_rTableW.Print (stream, i->first);
		*stream->GetStream () << std::endl;
	}

	*stream->GetStream () << std::endl;
}

void
RoutingExperiment::PrintRoutingTableWD (Ptr<OutputStreamWrapper> stream,  Ptr<Socket> socket, uint16_t id) const	//prints the routing table stream.
{

	*stream->GetStream () << "Node: " << id
			<< ", Time: " << Simulator::Now().GetSeconds()
			<< ", Routing table" << std::endl;

	for (std::map<Ipv4Address, Ipv4Address>::const_iterator i = m_interfaceMap.begin (); i
	!= m_interfaceMap.end (); ++i)
	{
		m_rTableWD.Print (stream, i->second);
	}
	*stream->GetStream () << std::endl;
}


void
RoutingExperiment::ReceiveDiscovery(Ptr<Socket> socket)
{
	uint32_t context = Simulator::GetContext();
	NS_LOG_DEBUG("Context: " << context);
	Ptr<Packet> packet;
	Address senderAddress;
	Ptr<Node> thisNode = NodeList::GetNode(context);

	while ((packet = socket->RecvFrom (senderAddress)))
	{
		DiscoveryPacketHeader header;
		packet->RemoveHeader(header);
		Ptr<Node> node = socket->GetNode();
		uint16_t nodeID = node->GetId();
		bytesTotalDisc[nodeID] += packet->GetSize ();
		packetsReceivedDisc[nodeID] += 1;
		NS_LOG_DEBUG("Bytes total received: " << bytesTotalDisc[nodeID]);
		NS_LOG_DEBUG("Packets total received: " << packetsReceivedDisc[nodeID]);
		Ipv4Address src_ip = header.GetSource();
		Vector myLocation = thisNode->GetObject<MobilityModel>()->GetPosition();
		Vector neighborNodeLocation;
		Ipv4Address myAddress = thisNode->GetObject<Ipv4>()->GetAddress(1,0).GetLocal();
		int32_t nNodes = NodeList::GetNNodes ();

		for (int32_t i = 0; i < nNodes; ++i)
		{
			Ptr<Node> node = NodeList::GetNode (i);
			Ptr<Ipv4> ipv4 = node->GetObject<Ipv4> ();

			int32_t ifIndex = ipv4->GetInterfaceForAddress (src_ip);
			if (ifIndex != -1)
			{
				Ptr<MobilityModel> neighborNodeModel = node->GetObject<MobilityModel>();
				neighborNodeLocation = neighborNodeModel->GetPosition();
			}
		}

		Ptr<DiscoveryApplication> app = DynamicCast<DiscoveryApplication>( NodeList::GetNode(context)->GetApplication(0));
		//app->ProcessDiscoveryPacket(header);
		uint16_t nextLoc = thisNode->GetObject<MarkovChainMobilityModel>()->GetNextLocation();
		uint16_t nextTime = thisNode->GetObject<MarkovChainMobilityModel>()->GetNextTime();
		double proSpeed = ProSpeedGen(1.0,2.4);
		NS_LOG_DEBUG("W Next Location in Header: " << header.GetNextLocation() << ", My next Location: " << nextLoc);
		if (header.GetNextLocation() == nextLoc)
		{
			RTableEntry rTableEntryW;
            NS_LOG_DEBUG("Header source " << header.GetSource() << " MyAddress " << myAddress);
            bool permanentTableVerifier = m_rTableW.LookupRoute (header.GetSource(), myAddress, rTableEntryW);
			NS_LOG_DEBUG("Is the route inside the table? " << permanentTableVerifier);
			if (permanentTableVerifier == false)
			{
				app->SendReplyPacketW(thisNode, myAddress, src_ip, nextLoc, nextTime, proSpeed);
				NS_LOG_DEBUG ("Received New WRoute!");
				RTableEntry newEntry(/*My Address=*/ myAddress, /*destination address=*/ header.GetSource(), /*time connected=*/0.0, /*time packet received=*/
						Simulator::Now (), /*time First packet received=*/ Simulator::Now (), /*my location=*/ myLocation, /*neighbor location=*/ neighborNodeLocation,
						/*neighbor next location=*/header.GetNextLocation(), /*neighbor next interval=*/0, /*neighbor link lifetime=*/0.0, /*neighbor current
						 * processor speed=*/ 0.0);
				m_rTableW.AddRoute (newEntry);
				NS_LOG_DEBUG ("New WRoute added to Wtable!");
			}
			else
			{
				app->SendReplyPacketW(thisNode, myAddress, src_ip, nextLoc, nextTime, proSpeed);

				rTableEntryW.setDestAddress(src_ip);
				double timeConnected = Simulator::Now().GetSeconds() - rTableEntryW.getTimeFirstPktRcvd().GetSeconds();
				rTableEntryW.setTimeConnected(timeConnected);
				rTableEntryW.setTimePktRcvd(Simulator::Now());
				rTableEntryW.setMyLocation(myLocation);
				rTableEntryW.setNeighborNodeLocation(neighborNodeLocation);
				rTableEntryW.setNextLocation(nextLoc);
				m_rTableW.Update(rTableEntryW);
				NS_LOG_DEBUG ("Route Updated in WTable!");
			}
			NS_LOG_DEBUG("Values in the rTableW are: ");
			if(m_rTableW.RTableSize() > 0)
			{
				m_rTableW.GetListOfAllRoutes();
				m_rTableW.GetListofAllRoutes(myAddress);
				Ptr<OutputStreamWrapper> routingStream = Create<OutputStreamWrapper> ((rtDiscW + ".routes"), std::ios::app);
				PrintRoutingTableW(routingStream,socket, nodeID);
			}
		}
	}
}

void
RoutingExperiment::ReceiveDiscoveryWD(Ptr<Socket> socket)
{
	uint32_t context = Simulator::GetContext();
	NS_LOG_DEBUG("Context: " << context);
	Ptr<Packet> packet;
	Address senderAddress;
	Ptr<Node> thisNode = NodeList::GetNode(context);
	Ptr<Node> node = socket->GetNode();
	uint16_t nodeID = node->GetId();
	while ((packet = socket->RecvFrom (senderAddress)))
	{
		DiscoveryPacketHeader header;
		packet->RemoveHeader(header);
		bytesTotalWDDisc[nodeID] += packet->GetSize ();
		packetsReceivedWDDisc[nodeID] += 1;
		NS_LOG_DEBUG("Bytes total received: " << bytesTotalWDDisc[nodeID]);
		NS_LOG_DEBUG("Packets total received: " << packetsReceivedWDDisc[nodeID]);
		Ipv4Address src_ip = header.GetSource();
		Vector myLocation = thisNode->GetObject<MobilityModel>()->GetPosition();
		Vector neighborNodeLocation;
		Ipv4Address myAddress = thisNode->GetObject<Ipv4>()->GetAddress(2,0).GetLocal();
		int32_t nNodes = NodeList::GetNNodes ();

		for (int32_t i = 0; i < nNodes; ++i)
		{
			Ptr<Node> node = NodeList::GetNode (i);
			Ptr<Ipv4> ipv4 = node->GetObject<Ipv4> ();

			int32_t ifIndex = ipv4->GetInterfaceForAddress (src_ip);
			if (ifIndex != -1)
			{
				Ptr<MobilityModel> neighborNodeModel = node->GetObject<MobilityModel>();
				neighborNodeLocation = neighborNodeModel->GetPosition();
			}
		}

		Ptr<DiscoveryApplication> app = DynamicCast<DiscoveryApplication>( NodeList::GetNode(context)->GetApplication(0));
		//app->ProcessDiscoveryPacket(header);
		uint16_t nextLoc = thisNode->GetObject<MarkovChainMobilityModel>()->GetNextLocation();
		uint16_t nextTime = thisNode->GetObject<MarkovChainMobilityModel>()->GetNextTime();
		double proSpeed = ProSpeedGen(1.0,2.4);
		NS_LOG_DEBUG("WD Next Location in Header: " << header.GetNextLocation() << ", My next Location: " << nextLoc);
		if (header.GetNextLocation() == nextLoc)
		{
			RTableEntry rTableEntryWD;
            NS_LOG_DEBUG("Header source " << header.GetSource() << " MyAddress " << myAddress);
            bool permanentTableVerifier = m_rTableWD.LookupRoute (header.GetSource(), myAddress,rTableEntryWD);
			NS_LOG_DEBUG("Is the route inside the WDtable? " << permanentTableVerifier);
			if (permanentTableVerifier == false)
			{
				app->SendReplyPacketWD(thisNode, myAddress, src_ip, nextLoc, nextTime, proSpeed);
				NS_LOG_DEBUG ("Received New WDRoute!");
				RTableEntry newEntry(/*My Address=*/ myAddress, /*destination address=*/ header.GetSource(), /*time connected=*/0.0, /*time packet received=*/
						Simulator::Now (), /*time First packet received=*/ Simulator::Now (), /*my location=*/ myLocation, /*neighbor location=*/ neighborNodeLocation,
						/*neighbor next location=*/header.GetNextLocation(), /*neighbor next interval=*/0, /*neighbor link lifetime=*/0.0,/*neighbor current processor
						 * speed=*/ 0.0);
				m_rTableWD.AddRoute (newEntry);
				NS_LOG_DEBUG ("New WDRoute added to WDtable!");
			}
			else
			{
				app->SendReplyPacketWD(thisNode, myAddress, src_ip, nextLoc, nextTime, proSpeed);

				rTableEntryWD.setDestAddress(src_ip);
				double timeConnected = Simulator::Now().GetSeconds() - rTableEntryWD.getTimeFirstPktRcvd().GetSeconds();
				rTableEntryWD.setTimeConnected(timeConnected);
				rTableEntryWD.setTimePktRcvd(Simulator::Now());
				rTableEntryWD.setMyLocation(myLocation);
				rTableEntryWD.setNeighborNodeLocation(neighborNodeLocation);
				rTableEntryWD.setNextLocation(nextLoc);
				m_rTableWD.Update(rTableEntryWD);
				NS_LOG_DEBUG ("Route Updated in WDTable!");
			}
			NS_LOG_DEBUG("Values in the rTableWD are: ");
			if(m_rTableWD.RTableSize() > 0)
			{
				m_rTableWD.GetListOfAllRoutes();
				m_rTableWD.GetListofAllRoutes(myAddress);
				Ptr<OutputStreamWrapper> routingStream = Create<OutputStreamWrapper> ((rtDiscWD + ".routes"), std::ios::app);
				PrintRoutingTableWD(routingStream,socket, nodeID);
			}

		}
	}

}

void
RoutingExperiment::ReceiveReply(Ptr<Socket> socket)
{

	uint32_t context = Simulator::GetContext();
	NS_LOG_DEBUG("Context: " << context);
	Ptr<Packet> packet;
	Address senderAddress;
	Ptr<Node> thisNode = NodeList::GetNode(context);
	Ptr<Node> node = socket->GetNode();
	uint16_t nodeID = node->GetId();
	while ((packet = socket->RecvFrom (senderAddress)))
	{
		ReplyPacketHeader header;
		packet->RemoveHeader(header);
		bytesTotalDisc[nodeID] += packet->GetSize ();
		packetsReceivedDisc[nodeID] += 1;
		NS_LOG_DEBUG("Bytes total received: " << bytesTotalDisc[nodeID]);
		NS_LOG_DEBUG("Packets total received: " << packetsReceivedDisc[nodeID]);
		Ipv4Address src_ip = header.GetSource();
		Vector myLocation = thisNode->GetObject<MobilityModel>()->GetPosition();
		Vector neighborNodeLocation;
		Ipv4Address myAddress = thisNode->GetObject<Ipv4>()->GetAddress(1,0).GetLocal();
		//uint16_t senderNode;
		int32_t nNodes = NodeList::GetNNodes ();

		for (int32_t i = 0; i < nNodes; ++i)
		{
			Ptr<Node> node = NodeList::GetNode (i);
			Ptr<Ipv4> ipv4 = node->GetObject<Ipv4> ();

			int32_t ifIndex = ipv4->GetInterfaceForAddress (src_ip);
			if (ifIndex != -1)
			{
				//senderNode = i;
				Ptr<MobilityModel> neighborNodeModel = node->GetObject<MobilityModel>();
				neighborNodeLocation = neighborNodeModel->GetPosition();
			}
		}
		NS_LOG_DEBUG("Next Time interval in Header: " << header.GetNextTimeInterval() << ", My Next Time Interval: " << thisNode->GetObject<MarkovChainMobilityModel>()->GetNextTime());
		double llt = std::min(TimeIntervalToTime(header.GetNextTimeInterval()), TimeIntervalToTime(thisNode->GetObject<MarkovChainMobilityModel>()->GetNextTime()));
		NS_LOG_DEBUG("Link LifetimeW: " << llt);
		RTableEntry rTableEntryW;

        NS_LOG_DEBUG("Header source " << src_ip << " MyAddress " << myAddress);
        bool permanentTableVerifier = m_rTableW.LookupRoute (src_ip, myAddress, rTableEntryW);
		NS_LOG_DEBUG("Is the route inside the RTableW? " << permanentTableVerifier);
		if (permanentTableVerifier == false)
		{
			NS_LOG_DEBUG ("Received New WRoute!");
			RTableEntry newEntry(/*My Address=*/ myAddress, /*destination address=*/ header.GetSource(), /*time connected=*/0.0, /*time packet received=*/
					Simulator::Now (), /*time First packet received=*/ Simulator::Now (), /*my location=*/ myLocation, /*neighbor location=*/ neighborNodeLocation,
					/*neighbor next location=*/header.GetNextLocation(),/*neighbor next time Interval=*/header.GetNextTimeInterval(),/*neighbor link lifetime=*/
					llt, /*neighbor current processor speed=*/ header.GetCurrProSpeed());

			m_rTableW.AddRoute (newEntry);
			NS_LOG_DEBUG ("New WRoute added to Wtable!");
		}
		else
		{
			uint16_t nextLoc = rTableEntryW.getNextLoc();
			uint16_t nextTime = rTableEntryW.getNextTime();
			//	double currProSpeed = rTableEntryW.getCurrProSpeed();
			if (header.GetNextLocation() == nextLoc)
			{
				if (header.GetNextTimeInterval() == nextTime)
				{
					//if(header.GetCurrProSpeed() == currProSpeed){
                    double timeConnected = Simulator::Now().GetSeconds() - rTableEntryW.getTimeFirstPktRcvd().GetSeconds();
                    rTableEntryW.setTimeConnected(timeConnected);
                    rTableEntryW.setTimePktRcvd(Simulator::Now());
                    rTableEntryW.setLinkLifeTime(llt);
					rTableEntryW.setCurrProSpeed(header.GetCurrProSpeed());
					m_rTableW.Update(rTableEntryW);
					NS_LOG_DEBUG("Routing Table W updated");
					NS_LOG_DEBUG("No need to update the Routing Table W as the values haven't changed yet!");
					//}
				}
				else
				{
                    double timeConnected = Simulator::Now().GetSeconds() - rTableEntryW.getTimeFirstPktRcvd().GetSeconds();
                    rTableEntryW.setTimeConnected(timeConnected);
                    rTableEntryW.setTimePktRcvd(Simulator::Now());
                    rTableEntryW.setNextTime(header.GetNextTimeInterval());
					rTableEntryW.setLinkLifeTime(llt);
					rTableEntryW.setCurrProSpeed(header.GetCurrProSpeed());
					m_rTableW.Update(rTableEntryW);
					NS_LOG_DEBUG("Routing Table W updated");
				}

			}
			else
			{
                double timeConnected = Simulator::Now().GetSeconds() - rTableEntryW.getTimeFirstPktRcvd().GetSeconds();
                rTableEntryW.setTimeConnected(timeConnected);
                rTableEntryW.setTimePktRcvd(Simulator::Now());
                rTableEntryW.setNextTime(header.GetNextTimeInterval());
				rTableEntryW.setNextLocation(header.GetNextLocation());
				rTableEntryW.setLinkLifeTime(llt);
				rTableEntryW.setCurrProSpeed(header.GetCurrProSpeed());
				m_rTableW.Update(rTableEntryW);
				NS_LOG_DEBUG("Routing Table W updated");
			}

		}
		NS_LOG_DEBUG("Values in the rTableW are: ");
		if(m_rTableW.RTableSize() > 0)
		{
			m_rTableW.GetListOfAllRoutes();
			m_rTableW.GetListofAllRoutes(myAddress);
			Ptr<OutputStreamWrapper> routingStream = Create<OutputStreamWrapper> ((rtRcvW + ".routes"), std::ios::app);
			PrintRoutingTableW(routingStream,socket, nodeID);
		}

	}
}
void
RoutingExperiment::ReceiveReplyWD(Ptr<Socket> socket)
{
	uint32_t context = Simulator::GetContext();
	NS_LOG_DEBUG("Context: " << context);
	Ptr<Packet> packet;
	Address senderAddress;
	Ptr<Node> thisNode = NodeList::GetNode(context);
	Vector myLocation = thisNode->GetObject<MobilityModel>()->GetPosition();
	Vector neighborNodeLocation;
	Ptr<Node> node = socket->GetNode();
	uint16_t nodeID = node->GetId();
	while ((packet = socket->RecvFrom (senderAddress)))
	{
		ReplyPacketHeader header;
		packet->RemoveHeader(header);
		bytesTotalWDDisc[nodeID] += packet->GetSize ();
		packetsReceivedWDDisc[nodeID] += 1;
		NS_LOG_DEBUG("Bytes total received: " << bytesTotalWDDisc[nodeID]);
		NS_LOG_DEBUG("Packets total received: " << packetsReceivedWDDisc[nodeID]);
		Ipv4Address src_ip = header.GetSource();
		//        uint16_t senderNode;
		int32_t nNodes = NodeList::GetNNodes ();
		Ipv4Address myAddress = thisNode->GetObject<Ipv4>()->GetAddress(2,0).GetLocal();

		for (int32_t i = 0; i < nNodes; ++i)
		{
			Ptr<Node> node = NodeList::GetNode (i);
			Ptr<Ipv4> ipv4 = node->GetObject<Ipv4> ();

			int32_t ifIndex = ipv4->GetInterfaceForAddress (src_ip);
			if (ifIndex != -1)
			{
				//senderNode = i;
				Ptr<MobilityModel> neighborNodeModel = node->GetObject<MobilityModel>();
				neighborNodeLocation = neighborNodeModel->GetPosition();
			}
		}

		NS_LOG_DEBUG("Next Time interval in Header: " << header.GetNextTimeInterval() << ", My Next Time Interval: " << thisNode->GetObject<MarkovChainMobilityModel>()->GetNextTime());
		double llt = std::min(TimeIntervalToTime(header.GetNextTimeInterval()), TimeIntervalToTime(thisNode->GetObject<MarkovChainMobilityModel>()->GetNextTime()));
		NS_LOG_DEBUG("Link LifetimeWD: " << llt);
		RTableEntry rTableEntryWD;
       NS_LOG_DEBUG("Header source " << src_ip << " MyAddress " << myAddress);
        bool permanentTableVerifier = m_rTableWD.LookupRoute (src_ip, myAddress, rTableEntryWD);
		NS_LOG_DEBUG("Is the route inside the RtableWD? " << permanentTableVerifier);
		if (permanentTableVerifier == false)
		{
			NS_LOG_DEBUG ("Received New WDRoute!");
			RTableEntry newEntry(/*My Address=*/ myAddress, /*destination address=*/ header.GetSource(), /*time connected=*/0.0, /*time packet received=*/
					Simulator::Now (), /*time First packet received=*/ Simulator::Now (), /*my location=*/ myLocation, /*neighbor location=*/ neighborNodeLocation,
					/*neighbor next location=*/header.GetNextLocation(),/*neighbor next time Interval=*/header.GetNextTimeInterval(),/*neighbor link lifetime=*/
					llt,/*neighbor current processor speed=*/ header.GetCurrProSpeed());

			m_rTableWD.AddRoute (newEntry);
			NS_LOG_DEBUG ("New WDRoute added to WDtable!");
		}
		else
		{
			uint16_t nextLoc = rTableEntryWD.getNextLoc();
			uint16_t nextTime = rTableEntryWD.getNextTime();
			//double proSpeed = rTableEntryWD.getCurrProSpeed();
			if (header.GetNextLocation() == nextLoc)
			{
				if (header.GetNextTimeInterval() == nextTime)
				{
                    double timeConnected = Simulator::Now().GetSeconds() - rTableEntryWD.getTimeFirstPktRcvd().GetSeconds();
                    rTableEntryWD.setTimeConnected(timeConnected);
                    rTableEntryWD.setTimePktRcvd(Simulator::Now());
                    rTableEntryWD.setLinkLifeTime(llt);
					rTableEntryWD.setCurrProSpeed(header.GetCurrProSpeed());
					m_rTableWD.Update(rTableEntryWD);
					NS_LOG_DEBUG("Routing Table WD updated");
					//					if(header.GetCurrProSpeed() == proSpeed){
					//
					//					}
					NS_LOG_DEBUG("No need to update the Routing Table WD as the values haven't changed yet!");
				}
				else
				{
                    double timeConnected = Simulator::Now().GetSeconds() - rTableEntryWD.getTimeFirstPktRcvd().GetSeconds();
                    rTableEntryWD.setTimeConnected(timeConnected);
                    rTableEntryWD.setTimePktRcvd(Simulator::Now());
                    rTableEntryWD.setNextTime(header.GetNextTimeInterval());
					rTableEntryWD.setLinkLifeTime(llt);
					rTableEntryWD.setCurrProSpeed(header.GetCurrProSpeed());
					m_rTableWD.Update(rTableEntryWD);
					NS_LOG_DEBUG("Routing Table WD updated");
				}

			}
			else
			{
				rTableEntryWD.setNextTime(header.GetNextTimeInterval());
				rTableEntryWD.setNextLocation(header.GetNextLocation());
				rTableEntryWD.setLinkLifeTime(llt);
				rTableEntryWD.setCurrProSpeed(header.GetCurrProSpeed());
				m_rTableW.Update(rTableEntryWD);
				NS_LOG_DEBUG("Routing Table WD updated");
			}

		}
		NS_LOG_DEBUG("Values in the rTableWD are: ");
		if(m_rTableWD.RTableSize() > 0)
		{
			m_rTableWD.GetListOfAllRoutes();
			m_rTableWD.GetListofAllRoutes(myAddress);
			Ptr<OutputStreamWrapper> routingStream = Create<OutputStreamWrapper> ((rtRcvWD + ".routes"), std::ios::app);
			PrintRoutingTableWD(routingStream,socket, nodeID);
		}

	}
}

void
RoutingExperiment::CheckThroughput (uint16_t i)
{
	mbsDisc[i] = (bytesTotalDisc[i] * 8.0) / 1000 / 1000;
	bytesTotalDisc[i] = 0;

	mbsWDDisc[i] = (bytesTotalWDDisc[i] * 8.0) / 1000 / 1000;
	bytesTotalWDDisc[i] = 0;

	mbsApp[i] = (bytesTotalApp[i] * 8.0) / 1000 / 1000;
	bytesTotalApp[i] = 0;

	mbsWDApp[i] = (bytesTotalWDApp[i] * 8.0) / 1000 / 1000;
	bytesTotalWDApp[i] = 0;

	double throughputW = mbsDisc[i]/(Simulator::Now().GetSeconds() - m_timeFirstPktDiscSent[i]);

	double throughputWApp = mbsApp[i]/(Simulator::Now().GetSeconds() - m_timeFirstPktAppSent[i]);

	double throughputWD = mbsWDDisc[i]/(Simulator::Now().GetSeconds() - m_timeFirstPktDiscWDSent[i]);

	double throughputWDApp = mbsWDApp[i]/(Simulator::Now().GetSeconds() - m_timeFirstPktAppWDSent[i]);

	NS_LOG_DEBUG("Performance Metrics: mbsDisc: " << mbsDisc[i] << ", mbsWDDisc: " << mbsWDDisc[i] << ", mbsApp: " << mbsApp[i] << ", mbsWDApp: " << mbsWDApp[i]
		 << ", throughputW: " << throughputW << ", throughputWApp: " << throughputWApp << ", throughputWD: " << throughputWD << ", throughputWDApp: " << throughputWDApp);

	std::ofstream out (std::to_string(i) + m_CSVfileName.c_str (), std::ios::app);
	out << (Simulator::Now ()).GetSeconds () << ","
			<< tasksAssigned[i] << ","
			<< tasksFailed[i] << ","
			<< mbsDisc[i] << ","
			<< mbsWDDisc[i] << ","
			<< mbsApp[i] << ","
			<< mbsWDApp[i] << ","
			<< throughputW << ","
			<< throughputWD << ","
			<< throughputWApp << ","
			<< throughputWDApp << ","
			<< packetsReceivedDisc[i] << ","
			<< packetsReceivedWDDisc[i] << ","
			<< packetsReceivedApp[i] << ","
			<< packetsReceivedWDApp[i] << ","
			<< currentSeqNo[i] << ","
			<< currentSeqNoWD[i] << ","
			<< m_txDataRate[i] << ","
			<< m_txDataRateWD[i] << ","
			<< m_rxDataRate[i] << ","
			<< m_rxDataRateWD[i] << ","
			<< delay[i] << ","
			<< delayWD[i] << ","
			<< counterAppTX[i] << ","
			<< counterAppTXWD[i] << ","
			<< counterAppRX[i] << ","
			<< counterAppRXWD[i] << ","
			<< counterTX[i] << ","
			<< counterTXWD[i] << ","
			<< counterRX[i] << ","
			<< counterRXWD[i] << ""
			<< std::endl;

	out.close ();

	packetsReceivedDisc[i] = 0;
	packetsReceivedWDDisc[i] = 0;
	packetsReceivedApp[i] = 0;
	packetsReceivedWDApp[i] = 0;
	currentSeqNo[i] = 0;
	counterTX[i] = 0;
	counterRX[i] = 0;
	counterAppTX[i] = 0;
	counterAppRX[i] = 0;
	rcv[i] = 0.0;
	sqhd[i] = 0.0;
	delay[i] = 0.0;
	rcvWD[i] = 0.0;
	sqhdWD[i] = 0.0;
	delayWD[i] = 0.0;
	currentSeqNoWD[i] = 0;
	counterTXWD[i] = 0;
	counterRXWD[i] = 0;
	counterAppTXWD[i] = 0;
	counterAppRXWD[i] = 0;
	PhyTxDropCount = 0;
	PhyRxDropCount = 0;
	PhyTxDropCount = 0;
	PhyRxDropCount = 0;
	PhyTxDropCountWD = 0;
	PhyRxDropCountWD = 0;
	discPkt = 0;
	discPktWD = 0;

	Simulator::Schedule (Seconds (1.0), &RoutingExperiment::CheckThroughput, this, i);
}

Ptr<Socket>
RoutingExperiment::SetupPacketReceive (Ipv4Address addr, Ptr<Node> node)
{
	TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
	Ptr<Socket> sink = Socket::CreateSocket (node, tid);
	InetSocketAddress local = InetSocketAddress (addr, port + 1);
	NS_LOG_DEBUG("Local Address W: " << local.GetIpv4());
	sink->Bind (local);
	sink->SetRecvCallback (MakeCallback (&RoutingExperiment::ReceivePacket, this));

	return sink;
}

Ptr<Socket>
RoutingExperiment::SetupPacketReceiveWD (Ipv4Address addr, Ptr<Node> node)
{
	TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
	Ptr<Socket> sink = Socket::CreateSocket (node, tid);
	InetSocketAddress local = InetSocketAddress (addr, portWD + 1);
	NS_LOG_DEBUG("Local Address WD: " << local.GetIpv4());
	sink->Bind (local);
	sink->SetRecvCallback (MakeCallback (&RoutingExperiment::ReceivePacketWD, this));

	return sink;
}

Ptr<Socket>
RoutingExperiment:: SetupDiscoveryReceive (Ipv4Address addr, Ptr<Node> node)
{
	TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
	Ptr<Socket> sink = Socket::CreateSocket (node, tid);
	InetSocketAddress local = InetSocketAddress (addr, port);
	NS_LOG_DEBUG("Local Address W: " << local.GetIpv4());
	sink->Bind (local);
	sink->SetRecvCallback (MakeCallback (&RoutingExperiment::ReceiveDiscovery, this));

	return sink;
}

Ptr<Socket>
RoutingExperiment::SetupDiscoveryReceiveWD (Ipv4Address addr, Ptr<Node> node)
{
	TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
	Ptr<Socket> sink = Socket::CreateSocket (node, tid);
	InetSocketAddress local = InetSocketAddress (addr, portWD);
	NS_LOG_DEBUG("Local Address WD: " << local.GetIpv4());
	sink->Bind (local);
	sink->SetRecvCallback (MakeCallback (&RoutingExperiment::ReceiveDiscoveryWD, this));

	return sink;
}

Ptr<Socket>
RoutingExperiment::SetupReplyReceive (Ipv4Address addr, Ptr<Node> node)
{
	TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
	Ptr<Socket> sink = Socket::CreateSocket (node, tid);
	InetSocketAddress local = InetSocketAddress (addr, port);
	NS_LOG_DEBUG("Local Address WD: " << local.GetIpv4());
	sink->Bind (local);
	sink->SetRecvCallback (MakeCallback (&RoutingExperiment::ReceiveReply, this));

	return sink;
}

Ptr<Socket>
RoutingExperiment::SetupReplyReceiveWD (Ipv4Address addr, Ptr<Node> node)
{
	TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
	Ptr<Socket> sink = Socket::CreateSocket (node, tid);
	InetSocketAddress local = InetSocketAddress (addr, portWD);
	NS_LOG_DEBUG("Local Address WD: " << local.GetIpv4());
	sink->Bind (local);
	sink->SetRecvCallback (MakeCallback (&RoutingExperiment::ReceiveReplyWD, this));

	return sink;
}

std::string
RoutingExperiment::CommandSetup (int argc, char **argv)
{
	CommandLine cmd;
	cmd.AddValue ("CSVfileName", "The name of the CSV output file name", m_CSVfileName);
	cmd.AddValue ("traceMobility", "Enable mobility tracing", m_traceMobility);
	cmd.AddValue ("protocol", "1=OLSR;2=AODV;3=DSDV;4=DSR", m_protocol);
	cmd.Parse (argc, argv);
	return m_CSVfileName;
}

void
RoutingExperiment::MacTxDrop(Ptr<const Packet> p)
{
	NS_LOG_INFO("Packet Drop");
	MacTxDropCount++;
}

void
RoutingExperiment::PhyTxDrop(Ptr<const Packet> p)
{
	NS_LOG_INFO("Packet Drop");
	PhyTxDropCount++;
}
void
RoutingExperiment::PhyRxDrop(Ptr<const Packet> p, ns3::WifiPhyRxfailureReason reason)
{
	NS_LOG_INFO("Packet Drop and Reason is: " << reason);
	PhyRxDropCount++;
}

void
RoutingExperiment::PrintDrop()
{
	NS_LOG_DEBUG( Simulator::Now().GetSeconds() << "\t MAC TX Drop: " << MacTxDropCount << "\t PHY TX Drop: "<< PhyTxDropCount << "\t PHY RX Drop: " << PhyRxDropCount << "\n");
	Simulator::Schedule(Seconds(5.0), &RoutingExperiment::PrintDrop,this);
}

void
RoutingExperiment::PrintDropWD()
{
	NS_LOG_DEBUG( Simulator::Now().GetSeconds() << "\t WDMAC TX Drop: " << MacTxDropCountWD << "\t WDPHY TX Drop: "<< PhyTxDropCountWD << "\t WDPHY RX Drop: " << PhyRxDropCountWD << "\n");
	Simulator::Schedule(Seconds(5.0), &RoutingExperiment::PrintDropWD, this);
}

void
RoutingExperiment::MacTxDropWD(Ptr<const Packet> p)
{
	NS_LOG_INFO("Packet Drop WD");
	MacTxDropCountWD++;
}

void
RoutingExperiment::PhyTxDropWD(Ptr<const Packet> p)
{
	NS_LOG_INFO("Packet Drop WD");
	PhyTxDropCountWD++;
}
void
RoutingExperiment::PhyRxDropWD(Ptr<const Packet> p, ns3::WifiPhyRxfailureReason reason)
{
	NS_LOG_INFO("Packet Drop WD and Reason is: " << reason);
	PhyRxDropCountWD++;
}



void
RoutingExperiment::LinkLifeTimer()
{
	m_rTableW.GetInActiveRoutes();
	m_rTableWD.GetInActiveRoutes();
	Simulator::Schedule(Seconds(1.0), &RoutingExperiment::LinkLifeTimer, this);
}

void
RoutingExperiment::CourseChange (std::string context, Ptr<const MobilityModel> model)
{
	Vector position = model->GetPosition();
	NS_LOG_DEBUG (context << " x = " << position.x << ", y = " << position.y);
}

int
main (int argc, char *argv[])
{
	LogComponentEnable ("ManetRoutingCompare", LOG_LEVEL_DEBUG);
	RoutingExperiment experiment;
	uint32_t nSinks = 5;
	std::string CSVfileName = experiment.CommandSetup (argc,argv);

	//blank out the last output file and write the column headers
	for(uint16_t i = 0; i < nSinks; i++){
		std::ofstream out (std::to_string(i) + CSVfileName.c_str ());
		out << "SimulationSecond," <<
				"TasksAssigned," <<
				"TasksFailed," <<
				"ThroughputDisc," <<
				"ThroughputWDDisc," <<
				"ThroughputApp," <<
				"ThroughputWDApp," <<
				"ThroughputDiscovery," <<
				"ThroughputWDDiscovery," <<
				"ThroughputApplication," <<
				"ThroughputWDApplication," <<
				"PacketsReceivedDisc," <<
				"PacketsReceivedWDDisc," <<
				"PacketsReceivedApp," <<
				"PacketsReceivedWDApp," <<
				"TotalPacketsSent," <<
				"TotalPacketsSentWD," <<
				"TxDataRate," <<
				"TxDataRateWD," <<
				"RxDataRate," <<
				"RxDataRateWD," <<
				"Delay," <<
				"DelayWD," <<
				"NumOfTxAppPkts," <<
				"NumOfTxAppPktsWD," <<
				"NumOfRxAppPkts," <<
				"NumOfRxAppPktsWD," <<
				"TxPackets," <<
				"TxPacketsWD," <<
				"RxPackets," <<
				"RxPacketsWD" <<
				std::endl;
		out.close ();
	}


	double txp = 7.5;

	experiment.Run (nSinks, txp, CSVfileName);
}

void
RoutingExperiment::Run (int nSinks, double txp, std::string CSVfileName)
{
	Packet::EnablePrinting ();
	m_nSinks = nSinks;
	m_txp = txp;
	m_CSVfileName = CSVfileName;
	uint32_t nWifis = nSinks;
//	double m_dataStart = 0.01;
	double TotalTime = 250.0;
	std::string tr_name ("manet-routing-compare");
    rtRcvW = "rt-rcv-w";
    rtRcvWD = "rt-rcv-wd";
    rtDiscW = "rt-disc-w";
    rtDiscWD = "rt-disc-wd";
	int nodeSpeed = 30; //in m/s
	int nodePause = 0; //in s
	std::string rtslimit = "2200";
	m_protocolName = "protocol";

	Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue (rtslimit));
	adhocNodes.Create (nWifis);

	// setting up wifi phy and channel using helpers
	WifiHelper wifi, wifiD;
	wifi.SetStandard (WIFI_PHY_STANDARD_80211b);
	wifiD.SetStandard(WIFI_PHY_STANDARD_80211n_5GHZ);

	YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();
	YansWifiPhyHelper wifiPhyWD =  YansWifiPhyHelper::Default ();
	YansWifiChannelHelper wifiChannelW, wifiChannelWD;//, wifiChannelLTE;
	wifiChannelW.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
	wifiChannelW.AddPropagationLoss ("ns3::FriisPropagationLossModel");
	wifiPhy.SetChannel (wifiChannelW.Create ());

	wifiChannelWD.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
	wifiChannelWD.AddPropagationLoss ("ns3::FriisPropagationLossModel");
	wifiPhyWD.SetChannel (wifiChannelWD.Create ());
	// Add a mac and disable rate control
	WifiMacHelper wifiMac;
	wifiMac.SetType ("ns3::AdhocWifiMac");

	wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager", "DataMode", StringValue ("DsssRate11Mbps"), "ControlMode",
			StringValue ("DsssRate11Mbps"));
	wifiD.SetRemoteStationManager ("ns3::MinstrelHtWifiManager", "NonUnicastMode", StringValue ("ErpOfdmRate36Mbps"));

	wifiPhy.Set ("TxPowerStart",DoubleValue (txp));
	wifiPhy.Set ("TxPowerEnd", DoubleValue (txp));

	wifiPhyWD.Set ("TxPowerStart",DoubleValue (txp));
	wifiPhyWD.Set ("TxPowerEnd", DoubleValue (txp));

	NetDeviceContainer adhocDevices = wifi.Install (wifiPhy, wifiMac, adhocNodes);
	NetDeviceContainer adhocDevicesWD = wifiD.Install (wifiPhyWD, wifiMac, adhocNodes);

	MobilityHelper mobility;

	int64_t streamIndex = 0; // used to get consistent mobility across scenarios

	ObjectFactory pos;
	pos.SetTypeId ("ns3::RandomBoxPositionAllocator");
	pos.Set ("X", StringValue ("ns3::UniformRandomVariable[Min=0|Max=500.0]"));
	pos.Set ("Y", StringValue ("ns3::UniformRandomVariable[Min=0|Max=450.0]"));
	pos.Set ("Z", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=100.0]"));

	Ptr<PositionAllocator> taPositionAlloc = pos.Create ()->GetObject<PositionAllocator> ();
	streamIndex += taPositionAlloc->AssignStreams (streamIndex);

	std::stringstream ssSpeed;
	//ssSpeed << "ns3::UniformRandomVariable[Min=0.0|Max=" << nodeSpeed << "]";
	ssSpeed << "ns3::ConstantRandomVariable[Constant=" << nodeSpeed << "]";
	std::stringstream ssPause;
	ssPause << "ns3::ConstantRandomVariable[Constant=" << nodePause << "]";

	mobility.SetMobilityModel("ns3::MarkovChainMobilityModel",
			"BoundsOne",BoxValue (Box (0.0, 150.0, 0.0, 150.0, 0.0, 100.0)),
			"BoundsTwo",BoxValue (Box (50.0, 150.0, 200.0, 350.0, 0.0, 100.0)),
			"BoundsThree",BoxValue (Box (200.0, 300.0, 0.0, 200.0, 0.0, 100.0)),
			"BoundsFour",BoxValue (Box (220.0, 500.0, 300.0, 450.0, 0.0, 100.0)),
			"BoundsFive",BoxValue (Box (350.0, 500.0, 0.0, 250.0, 0.0, 100.0)),
			"Speed", StringValue (ssSpeed.str ()),
			"Pause", StringValue (ssPause.str ()),
			"PositionAllocator", PointerValue (taPositionAlloc));
	mobility.SetPositionAllocator (taPositionAlloc);
	mobility.Install (adhocNodes);
	streamIndex += mobility.AssignStreams (adhocNodes, streamIndex);
	NS_UNUSED (streamIndex); // From this point, streamIndex is unused);

	InternetStackHelper stack;
	stack.Install (adhocNodes);

	NS_LOG_INFO ("assigning ip address");

	Ipv4AddressHelper addressAdhoc, addressAdhocWD;
	addressAdhoc.SetBase ("10.1.1.0", "255.255.255.0");
	addressAdhocWD.SetBase ("10.1.2.0", "255.255.255.0");
	Ipv4InterfaceContainer adhocInterfaces, adhocInterfacesWD;
	adhocInterfaces = addressAdhoc.Assign (adhocDevices);
	adhocInterfacesWD = addressAdhocWD.Assign(adhocDevicesWD);

	for(uint16_t i=0;i < nWifis; i++)
	{
		Ptr<Node> node = NodeList::GetNode (i);
		Ipv4Address nodeAddress = node->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal ();
		Ipv4Address nodeAddressWD = node->GetObject<Ipv4> ()->GetAddress (2, 0).GetLocal ();
		m_interfaceMap.insert(std::pair<Ipv4Address, Ipv4Address>(nodeAddress, nodeAddressWD));
		appPktSend.insert(std::pair<Ipv4Address, uint64_t>(nodeAddress,0));
		appPktRec.insert(std::pair<Ipv4Address, uint64_t>(nodeAddress,0));
		appPktWDSend.insert(std::pair<Ipv4Address, uint64_t>(nodeAddressWD,0));
		appPktWDRec.insert(std::pair<Ipv4Address, uint64_t>(nodeAddressWD,0));
	}

	for(uint16_t i=0;i < nWifis; i++)
	{
		Ptr<Node> node = NodeList::GetNode (i);
		Ptr<DiscoveryApplication> app = Create<DiscoveryApplication> ();
		Ipv4Address nodeAddress = node->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal ();
		Ipv4Address nodeAddressWD = node->GetObject<Ipv4> ()->GetAddress (2, 0).GetLocal ();
		Ipv4Address broadCast1 = node->GetObject<Ipv4>()->GetAddress(1,0).GetBroadcast();
		Ipv4Address broadCast2 = node->GetObject<Ipv4>()->GetAddress(2,0).GetBroadcast();
		NS_LOG_DEBUG("Broadcast Address W: " << broadCast1);
		NS_LOG_DEBUG("Broadcast Address WD: " << broadCast2);
		node->AddApplication(app);
		app->Setup(InetSocketAddress(broadCast1, port), InetSocketAddress(broadCast2, portWD), Seconds(1),10, 81);
		DiscoverySink = SetupDiscoveryReceive (broadCast1, node);
		DiscoverySinkWD = SetupDiscoveryReceiveWD (broadCast2, node);
		ReplySink = SetupReplyReceive(nodeAddress, node);
		ReplySinkWD = SetupReplyReceiveWD(nodeAddressWD, node);
	}

	for (uint32_t i = 0; i < m_nSinks; i++ )
	{
		Ptr<Node> node = NodeList::GetNode (i);
		Ipv4Address nodeAddress = node->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal();
		Ipv4Address nodeAddressWD = node->GetObject<Ipv4> ()->GetAddress (2, 0).GetLocal ();
		sink = SetupPacketReceive (nodeAddress, node);
		sinkWD = SetupPacketReceiveWD (nodeAddressWD, node);
	}

	NS_LOG_INFO ("Configure Tracing.");
	//tr_name = tr_name + "_" + m_protocolName +"_" + nodes + "nodes_" + sNodeSpeed + "speed_" + sNodePause + "pause_" + sRate + "rate";

	AsciiTraceHelper ascii;
	Ptr<OutputStreamWrapper> osw = ascii.CreateFileStream ( (tr_name + ".tr").c_str());
	wifiPhy.EnableAsciiAll (osw);
	wifiPhy.EnablePcap("manet-routing-compare", adhocDevices);
	MobilityHelper::EnableAsciiAll (ascii.CreateFileStream (tr_name + ".mob"));

	Ptr<FlowMonitor> flowmon;
	FlowMonitorHelper flowmonHelper;
	flowmon = flowmonHelper.InstallAll ();

	NS_LOG_INFO ("Run Simulation.");

	for(uint16_t i = 0; i < 5; i++){
		NS_LOG_DEBUG("First Time: " << m_firstTime[i]);
		if(m_firstTime[i]){
			NS_LOG_DEBUG("Ignore checking throughput");
		}
		else{
			CheckThroughput (i);
			m_firstTime[i] = false;
		}
	}
	PrintRoutingTable();
	Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/$ns3::MinstrelHtWifiManager/Rate", MakeCallback (&dataRate));
	Config::ConnectWithoutContext("/NodeList/*/DeviceList/0/$ns3::WifiNetDevice/Mac/MacTxDrop", MakeCallback(&RoutingExperiment::MacTxDrop, this));
	Config::ConnectWithoutContext("/NodeList/*/DeviceList/0/$ns3::WifiNetDevice/Phy/PhyRxDrop", MakeCallback(&RoutingExperiment::PhyRxDrop, this));
	Config::ConnectWithoutContext("/NodeList/*/DeviceList/0/$ns3::WifiNetDevice/Phy/PhyTxDrop", MakeCallback(&RoutingExperiment::PhyTxDrop, this));
	Config::ConnectWithoutContext("/NodeList/*/DeviceList/1/$ns3::WifiNetDevice/Mac/MacTxDrop", MakeCallback(&RoutingExperiment::MacTxDropWD, this));
	Config::ConnectWithoutContext("/NodeList/*/DeviceList/1/$ns3::WifiNetDevice/Phy/PhyRxDrop", MakeCallback(&RoutingExperiment::PhyRxDropWD, this));
	Config::ConnectWithoutContext("/NodeList/*/DeviceList/1/$ns3::WifiNetDevice/Phy/PhyTxDrop", MakeCallback(&RoutingExperiment::PhyTxDropWD, this));
	Config::ConnectWithoutContext("/NodeList/*/ApplicationList/0/$ns3::DiscoveryApplication/Tx", MakeCallback(&RoutingExperiment::txDiscApp, this));
	Config::ConnectWithoutContext("/NodeList/*/ApplicationList/0/$ns3::DiscoveryApplication/Tx", MakeCallback(&RoutingExperiment::txDiscAppWD, this));
	Config::Connect("/NodeList/*/DeviceList/0/$ns3::WifiNetDevice/Phy/MonitorSnifferTx", MakeCallback(&RoutingExperiment::Tx, this));
	Config::Connect("/NodeList/*/DeviceList/0/$ns3::WifiNetDevice/Phy/MonitorSnifferRx", MakeCallback(&RoutingExperiment::Rx, this));
	Config::Connect("/NodeList/*/DeviceList/1/$ns3::WifiNetDevice/Phy/MonitorSnifferTx", MakeCallback(&RoutingExperiment::TxWD, this));
	Config::Connect("/NodeList/*/DeviceList/1/$ns3::WifiNetDevice/Phy/MonitorSnifferRx", MakeCallback(&RoutingExperiment::RxWD, this));
	Config::Connect("/NodeList/*/$ns3::MobilityModel/CourseChange", MakeCallback (&RoutingExperiment::CourseChange, this));
	Simulator::Schedule(Seconds(5.0), &RoutingExperiment::PrintDrop, this);
	Simulator::Schedule(Seconds(5.0), &RoutingExperiment::PrintDropWD, this);
	Simulator::Schedule(Seconds(1.0), &RoutingExperiment::LinkLifeTimer, this);
	Simulator::Schedule(Seconds(10.0), &RoutingExperiment::StartTaskGeneration, this);
	Simulator::Schedule(Seconds(1.0), &RoutingExperiment::CheckThroughput, this,0);
	Simulator::Schedule(Seconds(1.0), &RoutingExperiment::CheckThroughput, this,1);
	Simulator::Schedule(Seconds(1.0), &RoutingExperiment::CheckThroughput, this,2);
	Simulator::Schedule(Seconds(1.0), &RoutingExperiment::CheckThroughput, this,3);
	Simulator::Schedule(Seconds(1.0), &RoutingExperiment::CheckThroughput, this,4);
	Simulator::Stop (Seconds (TotalTime));


	Simulator::Run ();

	// Print per flow statistics
	flowmon->CheckForLostPackets ();
	Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmonHelper.GetClassifier ());
	std::map<FlowId, FlowMonitor::FlowStats> stats = flowmon->GetFlowStats ();

//	for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator iter = stats.begin (); iter != stats.end (); ++iter)
//	{
//		Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (iter->first);

//		NS_LOG_UNCOND( "  Flow " << iter->first << " (" << t.sourceAddress << " -> " << t.destinationAddress << ")\n");
//		NS_LOG_UNCOND( "  Tx Bytes:   " << iter->second.txBytes << "\n");
//		NS_LOG_UNCOND( "  Rx Bytes:   " << iter->second.rxBytes << "\n");
//		NS_LOG_UNCOND( "  Throughput: " << iter->second.rxBytes * 8.0 / (TotalTime - m_dataStart) / 1000 / 1000 << " Mbps\n");
//		NS_LOG_UNCOND( "  Rx Segments: " << iter->second.rxPackets << "\n");
//		NS_LOG_UNCOND( "  Lost packets: " << iter->second.lostPackets << "\n");
//		NS_LOG_UNCOND( "  Packet Delivery Fraction: " << (float) iter->second.rxBytes / iter->second.txBytes * 100 << " %\n");
//		//  NS_LOG_UNCOND( "  Average End-to-end Delay: " << iter->second.delaySum / iter->second.rxPackets / 1000 << " us\n");
//		NS_LOG_UNCOND( "  Throughput: " << iter->second.rxBytes * 8.0 / (iter->second.timeLastRxPacket.GetSeconds()-iter->second.timeFirstTxPacket.GetSeconds()) / 1024 / 1024 << " Mbps");

//	}
	for(int i=0;i<5;i++)
	{
		NS_LOG_DEBUG("Tasks assigned to node: "<< i <<" : "<<tasksAssigned[i]);
		NS_LOG_DEBUG("Tasks failed by node: "<< i <<" : "<<tasksFailed[i]);
	}

    NS_LOG_DEBUG("Printing task completion details");
    int j = 0;

    for(std::vector<TaskDetails> node : allTasks)
    {    NS_LOG_DEBUG("-------------------------------------------");
	NS_LOG_DEBUG("Node Size: " << node.size());
        NS_LOG_DEBUG("Printing task completion details for node "<<j);
        for(TaskDetails task : node)
        {

            NS_LOG_DEBUG("Task assign time: "<< task.assignTime.GetSeconds()<<"s");
            if (task.success == false){
            	NS_LOG_DEBUG("Task assigned sucessfully: No");
            }
            else
            {
                NS_LOG_DEBUG("Task assigned sucessfully: Yes");
                NS_LOG_DEBUG("Data transfer started at: "<< task.dataTransferStart.GetSeconds()<<"s");
                NS_LOG_DEBUG("Task completed at: "<< task.dataTransferCompleted.GetSeconds() <<"s\n");
            }
        }
        j++;
    }

	flowmon->SerializeToXmlFile ((tr_name + ".flowmon").c_str(), true, true);

	Simulator::Destroy ();
}
