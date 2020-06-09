/*
 * markovchain-mobility-model.cc
 *
 *  Created on: Apr 17, 2020
 *      Author: hassam
 */
#include "ns3/markovchain-mobility-model.h"
#include "ns3/enum.h"
#include "ns3/double.h"
#include "ns3/string.h"
#include "ns3/pointer.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include "ns3/node.h"
#include <cmath>
#include <ns3/vector.h>
#include <vector>
#include <iostream>
#include <fstream>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("MarkovChain");

NS_OBJECT_ENSURE_REGISTERED (MarkovChainMobilityModel);

TypeId
MarkovChainMobilityModel::GetTypeId (void)
{
        static TypeId tid = TypeId ("ns3::MarkovChainMobilityModel").SetParent<MobilityModel> ()
           .SetGroupName ("Mobility")
           .AddConstructor<MarkovChainMobilityModel> ()
           .AddAttribute ("BoundsOne", "Bounds of the Location one's area to cruise.",
                          BoxValue (Box (0.0, 200.0, 0.0, 200.0, 0.0, 100.0)),
                          MakeBoxAccessor (&MarkovChainMobilityModel::m_l1),
                          MakeBoxChecker ())
           .AddAttribute ("BoundsTwo", "Bounds of the Location two's area to cruise.",
                          BoxValue (Box (150.0, 475.0, 300.0, 600.0, 0.0, 100.0)),
                          MakeBoxAccessor (&MarkovChainMobilityModel::m_l2),
                          MakeBoxChecker ())
           .AddAttribute ("BoundsThree", "Bounds of the Location three's area to cruise.",
                          BoxValue (Box (300.0, 600.0, 0.0, 200.0, 0.0, 100.0)),
                          MakeBoxAccessor (&MarkovChainMobilityModel::m_l3),
                          MakeBoxChecker ())
           .AddAttribute ("BoundsFour", "Bounds of the Location four's area to cruise.",
                          BoxValue (Box (525.0, 800.0, 300.0, 600.0, 0.0, 100.0)),
                          MakeBoxAccessor (&MarkovChainMobilityModel::m_l4),
                          MakeBoxChecker ())
           .AddAttribute ("BoundsFive", "Bounds of the Location five's area to cruise.",
                          BoxValue (Box (700.0, 1000.0, 0.0, 200.0, 0.0, 100.0)),
                          MakeBoxAccessor (&MarkovChainMobilityModel::m_l5),
                          MakeBoxChecker ())
           .AddAttribute ("TimeInterval", "Change current direction and speed after moving for this time interval.",
                          TimeValue (Seconds (1.0)),
                          MakeTimeAccessor (&MarkovChainMobilityModel::m_timeInterval),
                          MakeTimeChecker ())
           .AddAttribute ("Speed", "A random variable used to pick the speed (m/s).",
                          StringValue ("ns3::UniformRandomVariable[Min=1.0|Max=15.0]"),
                          MakePointerAccessor (&MarkovChainMobilityModel::m_speed),
                          MakePointerChecker<RandomVariableStream> ())
           .AddAttribute ("Pause", "A random variable used to pick the pause of a random waypoint model.",
                          StringValue ("ns3::ConstantRandomVariable[Constant=1.0]"),
                          MakePointerAccessor (&MarkovChainMobilityModel::m_pause),
                          MakePointerChecker<RandomVariableStream> ())
           .AddAttribute ("PositionAllocator", "The position model used to pick a destination point.",
                          PointerValue (),
                          MakePointerAccessor (&MarkovChainMobilityModel::m_position),
                          MakePointerChecker<PositionAllocator> ());
        return tid;
}

Vector MarkovChainMobilityModel::GetCenter(Box in)
{
    return Vector((in.xMax + in.xMin)/2,(in.yMax + in.yMin)/2, 100.0/2);
}

void
MarkovChainMobilityModel::DoInitialize (void)
{
	//m_default = true;
	m_firstTime = true;
	m_countArrival = 0;
    DoInitializePrivate ();
    MobilityModel::DoInitialize ();

    for(uint16_t i = 0; i < 5; i++){
        for(uint16_t j = 0; j < 5; j++){
            m_firstArrival[j] = true;
            m_elem.m_firstUpd[j] = true;
            m_elem.m_firstTimeUpd[j] = true;
            m_elem.m_locMatrix[i][j].m_locationCounter = 100;
            m_elem.m_locMatrix[i][j].m_locationProbability = 0.20;
            m_elem.m_locMatrix[i][j].m_totalNumOfVisits = 100;
            for(uint16_t k = 0; k < 3; k++){
                for(uint16_t l = 0; l < 3; l++){
                    if(k == l){
                        m_elem.m_locMatrix[i][j].m_timeIntervalMatrix[k][l].m_timeIntervalProbability = 0.34;
                        m_elem.m_locMatrix[i][j].m_timeIntervalMatrix[k][l].m_timeIntervalCounter = 100;
                        m_elem.m_locMatrix[i][j].m_timeIntervalMatrix[k][l].m_totalTimeIntervals = 100;
                    }
                    else{
                        m_elem.m_locMatrix[i][j].m_timeIntervalMatrix[k][l].m_timeIntervalProbability = 0.33;
                        m_elem.m_locMatrix[i][j].m_timeIntervalMatrix[k][l].m_timeIntervalCounter = 100;
                        m_elem.m_locMatrix[i][j].m_timeIntervalMatrix[k][l].m_totalTimeIntervals = 100;
                    }
                }
            }
        }
    }

    m_currLocation = 0;
    m_timePrevLoc = -1;
    m_timeCurrLoc = 0;
    NS_LOG_DEBUG("Previous Location Coordinates: " << m_helper.GetCurrentPosition());
    uint16_t prevloc = PositionToLocation(m_helper.GetCurrentPosition());
    if(prevloc != 5){
        m_prevLocation = prevloc;
    }
    else{
        prevloc = LocationIdentifier(m_helper.GetCurrentPosition());
        m_prevLocation = prevloc;
    }

    NS_LOG_DEBUG("Previous Location: " << m_prevLocation);
    m_prevArrivalTime[m_prevLocation] = Simulator::Now().GetSeconds();
    NS_LOG_DEBUG("Arrival Time at PreviousLocation in Initialize Method: " << m_prevArrivalTime[m_prevLocation]);

    for (int i =0; i< 5; i++)
        for (int j =0; j< 5; j++)
            m_prevTime[i][j] = 0;

    m_destLoc.insert({0, GetCenter(m_l1)});
    m_destLoc.insert({1, GetCenter(m_l2)});
    m_destLoc.insert({2, GetCenter(m_l3)});
    m_destLoc.insert({3, GetCenter(m_l4)});
    m_destLoc.insert({4, GetCenter(m_l5)});
    CheckTimeInterval();
    m_locFileName = "locMatricesResults.txt";
    m_timeFileName = "timeMatricesResults.txt";
    m_timeSpentFileName = "timeSpentMatricesResults.txt";

}

void
MarkovChainMobilityModel::DoInitializePrivate (void)
{
    m_helper.Update ();
    m_helper.Pause ();
    Time pause;
    NS_LOG_DEBUG("Count Arrival: " << m_countArrival);

    if(m_firstTime){
    	pause = Seconds(m_pause->GetValue());
    	NS_LOG_DEBUG("First Time");
    	m_firstTime = false;
    } else {
    	if(m_countArrival > 2){
    		NS_LOG_DEBUG("If Count Arrival: " << m_countArrival);
    		pause = Seconds(TimeIntervaltoTime(m_interval));
    	}
    	else{
    		NS_LOG_DEBUG("Else Count Arrival: " << m_countArrival);
    		pause = Seconds(m_pause->GetValue());
    	}
    }

    m_countArrival++;
    m_event = Simulator::Schedule (pause, &MarkovChainMobilityModel::BeginWalk, this);
    NotifyCourseChange ();
}

void MatrixElement::SetLocationProbability(uint16_t currLoc, uint16_t prevLoc){

    NS_LOG_DEBUG("Setting Location Probability");
    m_locMatrix[prevLoc][currLoc].m_locationProbability = (double) m_locMatrix[prevLoc][currLoc].m_locationCounter/m_locMatrix[prevLoc][currLoc].m_totalNumOfVisits;
}

void MatrixElement::SetTimeIntervalProbability(uint16_t currLoc, uint16_t prevLoc, uint16_t currTimeInt, uint16_t prevTimeInt){

        NS_LOG_DEBUG("Setting Time Interval Probability");
        NS_LOG_DEBUG("Time Interval Counter: " << m_locMatrix[prevLoc][currLoc].m_timeIntervalMatrix[prevTimeInt][currTimeInt].m_timeIntervalCounter
                << ", Total Time Intervals: " << m_locMatrix[prevLoc][currLoc].m_timeIntervalMatrix[prevTimeInt][currTimeInt].m_totalTimeIntervals);
        m_locMatrix[prevLoc][currLoc].m_timeIntervalMatrix[prevTimeInt][currTimeInt].m_timeIntervalProbability
                                = (double) m_locMatrix[prevLoc][currLoc].m_timeIntervalMatrix[prevTimeInt][currTimeInt].m_timeIntervalCounter /
                                m_locMatrix[prevLoc][currLoc].m_timeIntervalMatrix[prevTimeInt][currTimeInt].m_totalTimeIntervals;
}

void
MarkovChainMobilityModel::CheckTimeInterval(){
    m_timeCurrLoc = PositionToLocation(m_helper.GetCurrentPosition());
    if(m_timePrevLoc == m_timeCurrLoc || (m_timePrevLoc == 5 || m_timeCurrLoc == 5) ){
        NS_LOG_DEBUG("No need to update the time interval matrix");
        NS_LOG_DEBUG("Time Spent: " << m_timeSpent[m_timePrevLoc][m_timeCurrLoc] << "s");
    }
    else{

        NS_LOG_DEBUG("Arrival Time: " << m_arrivalTime[m_timeCurrLoc].GetSeconds());
        Ptr<Node> node = GetObject<Node>();
        uint16_t id = node->GetId();

        if(m_timePrevLoc != -1)
        {
            int time = Simulator::Now().GetSeconds() - m_arrivalTime[m_timeCurrLoc].GetSeconds();
            m_timeSpentMatriceResults.open (m_timeSpentFileName.c_str(), std::ios::app);
            m_timeSpent[m_timePrevLoc][m_timeCurrLoc] = time;
            m_currTime[m_timePrevLoc][m_timeCurrLoc] = TimeToTimeInterval(m_timeSpent[m_timePrevLoc][m_timeCurrLoc]);
            NS_LOG_DEBUG("Time Spent: " << m_timeSpent[m_timePrevLoc][m_timeCurrLoc] << "s");
            NS_LOG_DEBUG("Current Time: " << m_currTime[m_timePrevLoc][m_timeCurrLoc] << "s");
            NS_LOG_DEBUG("Printing timeSpent matrix");
            m_timeSpentMatriceResults << "\n\n" << Simulator::Now ().GetSeconds () <<", Node " << id << ": Time Spent Matrix\n ";
            for(int i=0; i < 5; i++){
                for(int j=0; j < 5; j++){
                   NS_LOG_DEBUG(" [" << i << "][" << j <<"] " << m_timeSpent[i][j]<<"s ");
                   m_timeSpentMatriceResults << " " << m_timeSpent[i][j] << " ";
                }
                NS_LOG_DEBUG(" ");
                m_timeSpentMatriceResults << " ";
            }
            m_timeSpentMatriceResults.close();
        }
        m_arrivalTime[m_timeCurrLoc] = Simulator::Now();
        m_timePrevLoc = m_timeCurrLoc;
    }
    Simulator::Schedule(Seconds(1.0), &MarkovChainMobilityModel::CheckTimeInterval, this);
}

uint16_t
MarkovChainMobilityModel::GetNextTime() const
{
    return m_interval;
}

uint16_t
MarkovChainMobilityModel::GetNextLocation() const
{
    return m_destination;
}

void MatrixElement::UpdateLocationProbability(uint16_t currLoc, uint16_t prevLoc, Ptr<MarkovChainMobilityModel> caller){
    NS_LOG_DEBUG("Location Probability Update Method is called");
    double rowSum = 0;
    double finalRowSum = 0;
    double sub = 0;
    double div = 0;
    double numOfVisits = 0;

    NS_LOG_DEBUG("Total Number of Visits before updating at current location: " << m_locMatrix[prevLoc][currLoc].m_totalNumOfVisits);
    //If this is the first time update method is called, then add the initial total number of visit at each location i.e. 100+100+100+100+100 = 500
    //if(m_firstUpd[currLoc] == true){
        for(uint16_t i = 0; i < 5; ++i){
            numOfVisits += m_locMatrix[prevLoc][i].m_locationCounter;
        }

        for(uint16_t j = 0; j< 5; j++){
            m_locMatrix[prevLoc][j].m_totalNumOfVisits = numOfVisits;
        }
    //once added. No need to add again so turn the bool to false
//		m_firstUpd[currLoc] = false;
//	}
    NS_LOG_DEBUG("Total Number of Visits after updating at current location: " << m_locMatrix[prevLoc][currLoc].m_totalNumOfVisits);

    //Insert the updated probability
    SetLocationProbability(currLoc, prevLoc);

    //The sum of a row should always be equal to 1, so initially calculate the row sum
    for(uint16_t i = 0; i < 5; i++){
        rowSum += m_locMatrix[prevLoc][i].m_locationProbability;
    }
    NS_LOG_DEBUG("Row Sum: " << rowSum);

    //if the sum is less than or greater than 1 then, first subtract the row sum from 1, and later divide this value with the n-1 here n is 5, so dividing by 4
    if(rowSum > 1 || rowSum < 1){
        sub = 1 - rowSum;
        div = sub/4;
        for(uint16_t i = 0; i < 5; i++){
            //No need to add the value from the current updated element, so checking if the element is current or not, if not then adding the divided value
            if(i != currLoc){
                m_locMatrix[prevLoc][i].m_locationProbability = m_locMatrix[prevLoc][i].m_locationProbability + div;
            }
        }
    }
    //finally calculating the row sum again after adding the div, so that rowsum should be 1.
    for(uint16_t i = 0; i < 5; i++){
        finalRowSum += m_locMatrix[prevLoc][i].m_locationProbability;
    }
    NS_LOG_DEBUG("Final row sum: " << finalRowSum);

//	caller->CheckTimeInterval();
    m_locMatrix[prevLoc][currLoc].m_timeIntervalMatrix[caller->m_prevTime[prevLoc][currLoc]][caller->m_currTime[prevLoc][currLoc]].m_timeIntervalCounter++;
    NS_LOG_DEBUG("Time Interval Counter: " << m_locMatrix[prevLoc][currLoc].m_timeIntervalMatrix[caller->m_prevTime[prevLoc][currLoc]][caller->m_currTime[prevLoc][currLoc]]
                                                                        .m_timeIntervalCounter);
    UpdateTimeIntervalProbability(currLoc, prevLoc, caller->m_currTime[prevLoc][currLoc], caller->m_prevTime[prevLoc][currLoc]);
     caller->m_prevTime[prevLoc][currLoc] = caller->m_currTime[prevLoc][currLoc];
//    double temp[3];
//    for(uint16_t j = 0; j < 3; j++)
//        temp[j] =  m_locMatrix[prevLoc][currLoc].m_timeIntervalMatrix[caller->m_prevTime[prevLoc][currLoc]][caller->m_currTime[prevLoc][currLoc]].m_timeIntervalProbability;

//    m_interval = SelectTimeInterval(temp);


}

void MatrixElement::UpdateTimeIntervalProbability(uint16_t currLoc, uint16_t prevLoc, uint16_t currTimeInt, uint16_t prevTimeInt){
    NS_LOG_DEBUG("Time Interval Probability Update Method is called");
    NS_LOG_DEBUG("PrevTimeInt: " << prevTimeInt << ", currentTimeInt: " << currTimeInt);
    double rowSum = 0;
    double finalRowSum = 0;
    double sub = 0;
    double div = 0;
    double numOfTimeIntervals = 0;
    NS_LOG_DEBUG("Number of Time Intervals before updating at current location: " << m_locMatrix[prevLoc][currLoc].m_timeIntervalMatrix[prevTimeInt][currTimeInt]
                                                                                        .m_timeIntervalCounter);
    NS_LOG_DEBUG("Total Number of Time Intervals before updating at current location: " << m_locMatrix[prevLoc][currLoc].m_timeIntervalMatrix[prevTimeInt][currTimeInt]
                                                                                            .m_totalTimeIntervals);
    //If this is the first time update method is called, then add the initial total number of time intervals at each time interval i.e. 100+100+100 = 300
    //if(m_firstTimeUpd[currLoc] == true){
        for(uint16_t k = 0; k < 3; k++){
            numOfTimeIntervals += m_locMatrix[prevLoc][currLoc].m_timeIntervalMatrix[prevTimeInt][k].m_timeIntervalCounter;
        }
        for(uint16_t k = 0; k < 3; k++){
            m_locMatrix[prevLoc][currLoc].m_timeIntervalMatrix[prevTimeInt][k].m_totalTimeIntervals = numOfTimeIntervals;
        }
//		m_firstTimeUpd[currLoc] = false;
//	}
    NS_LOG_DEBUG("Total Number of Time Intervals after updating: " << numOfTimeIntervals);
    NS_LOG_DEBUG("Number of Time Intervals after updating at current location: " << m_locMatrix[prevLoc][currLoc].m_timeIntervalMatrix[prevTimeInt][currTimeInt]
                                                                                            .m_timeIntervalCounter);
    NS_LOG_DEBUG("Total Number of Time Intervals after updating at current location: " << m_locMatrix[prevLoc][currLoc].m_timeIntervalMatrix[prevTimeInt][currTimeInt]
                                                                                                .m_totalTimeIntervals);

    //Insert the updated probability
    SetTimeIntervalProbability(currLoc, prevLoc, currTimeInt, prevTimeInt);
    NS_LOG_DEBUG("Time interval probability updating: " << m_locMatrix[prevLoc][currLoc].m_timeIntervalMatrix[prevTimeInt][currTimeInt].m_timeIntervalProbability);
    //The sum of a row should always be equal to 1, so initially calculate the row sum

    for(uint16_t j = 0; j < 3; j++){
        rowSum += m_locMatrix[prevLoc][currLoc].m_timeIntervalMatrix[prevTimeInt][j].m_timeIntervalProbability;
    }
    NS_LOG_DEBUG("Time Row Sum: " << rowSum);

    //if the sum is less than or greater than 1 then, first subtract the row sum from 1, and later divide this value with the n-1 here n is 3, so dividing by 2
    if(rowSum > 1 || rowSum < 1){
        sub = 1 - rowSum;
        NS_LOG_DEBUG("Sub: " << sub);
        div = sub/2;
        NS_LOG_DEBUG("Div: " << div);
        //No need to add the value from the current updated element, so checking if the element is current or not, if not then adding the divided value
        for(uint16_t j = 0; j < 3; j++){
            if(j != currTimeInt){
                m_locMatrix[prevLoc][currLoc].m_timeIntervalMatrix[prevTimeInt][j].m_timeIntervalProbability = m_locMatrix[prevLoc][currLoc]
                .m_timeIntervalMatrix[prevTimeInt][j].m_timeIntervalProbability + div;
                NS_LOG_DEBUG("Time interval probability updating: " << m_locMatrix[prevLoc][currLoc].m_timeIntervalMatrix[prevTimeInt][j].m_timeIntervalProbability);
            }
        }
    }

    for(uint16_t j = 0; j < 3; j++){
        finalRowSum += m_locMatrix[prevLoc][currLoc].m_timeIntervalMatrix[prevTimeInt][j].m_timeIntervalProbability;
    }
    NS_LOG_DEBUG("Final Time Row Sum: " << finalRowSum);

}

void
MarkovChainMobilityModel::BeginWalk (void)
{

    //Getting current position
    m_helper.Update ();
    Vector m_current = m_helper.GetCurrentPosition ();
    NS_ASSERT_MSG (m_position, "No position allocator added before using this model");
    NS_LOG_DEBUG("Current Location Coordinates: " << m_current);
    NS_LOG_DEBUG("Previous Location Coordinates: " << m_prevLocation);
    //convert our current position to location number
    uint16_t currentLoc = PositionToLocation(m_current);
    if(currentLoc == 5){
        m_currLocation = LocationIdentifier(m_current);
    } else{
        m_currLocation = currentLoc;
    }

    NS_LOG_DEBUG("Current Location: " << m_currLocation << ", Previous Location: " << m_prevLocation);

    if(m_prevLocation == m_currLocation){
        NS_LOG_DEBUG("No Need to update the matrix");
    } else {

        //Incrementing the location counter or the current number of visit
        m_elem.m_locMatrix[m_prevLocation][m_currLocation].m_locationCounter++;

        NS_LOG_DEBUG("Location Counter value after incrementing at m_prevlocation: " << m_prevLocation << ", and m_currLocation: " << m_currLocation << ", is: " <<
                m_elem.m_locMatrix[m_prevLocation][m_currLocation].m_locationCounter);
        //update the probability matrix for journey from m_prevLocation to m_currLocation
        Ptr<MarkovChainMobilityModel> temp(this);
        m_elem.UpdateLocationProbability(m_currLocation, m_prevLocation, temp);
        Ptr<Node> node = GetObject<Node>();
        uint16_t id = node->GetId();

        m_locMatricesResults.open (m_locFileName.c_str(), std::ios::app);

        m_locMatricesResults << "\n\n" << Simulator::Now ().GetSeconds () <<", Node " << id << ": Location Transition Probability Matrix\n ";
        for(uint16_t i = 0; i < 5; i++){
            for(uint16_t j = 0; j < 5; j++){
                NS_LOG_DEBUG("Printing updated location matrix: ");
                NS_LOG_DEBUG(" [" << i << "][" << j <<"] " << m_elem.m_locMatrix[i][j].m_locationProbability << " ");
                m_locMatricesResults << " " << m_elem.m_locMatrix[i][j].m_locationProbability << " ";
            }
            NS_LOG_DEBUG("");
            m_locMatricesResults << " \n";
        }
        m_locMatricesResults.close();

        m_timeMatricesResults.open (m_timeFileName.c_str(), std::ios::app);
        m_timeMatricesResults << "\n\n" << Simulator::Now ().GetSeconds () << ", Node " << id << ": Time Interval Transition Probability Matrix\n ";
        for(uint16_t i = 0; i < 5; i++){
            for(uint16_t j = 0; j < 5; j++){
                for(uint16_t k = 0; k < 3; k++){
                    for(uint16_t l = 0; l < 3; l++){
                        NS_LOG_DEBUG("Printing updated time interval matrix: ");
                        NS_LOG_DEBUG(" [" << i << "][" << j <<"], " << " [" << k << "][" << l <<"] " << m_elem.m_locMatrix[i][j].m_timeIntervalMatrix[k][l]
                            .m_timeIntervalProbability << " ");
                        m_timeMatricesResults<< " " << m_elem.m_locMatrix[i][j].m_timeIntervalMatrix[k][l].m_timeIntervalProbability << " ";
                    }
                    NS_LOG_DEBUG(" ");
                    m_timeMatricesResults <<" \n";
                }
                m_timeMatricesResults <<" \n\n";
            }
        }
        m_timeMatricesResults.close();
    }

    //find the destination location
    if (m_currLocation == 5)
    {
        double temp[5] = {0.2,0.2,0.2,0.2,0.2};
        m_destination = SelectLocation(temp);

    }
    else {
//        double temp[5];
//        for(uint16_t j = 0; j < 5; j++)
//            temp[j] =  m_elem.m_locMatrix[m_currLocation][j].m_locationProbability;

    	Ptr<Node> node = GetObject<Node>();
    	        uint16_t id = node->GetId();
    	switch (id){
    		case 0:
    		{

    			uint16_t m_locations[] = {1,0,3,4,2,1};
    			m_destination = m_locations[i[0]];
    			i[0]++;

    			uint16_t m_times[] = {0,0,1,2,0,2};
    			m_interval = m_times[k[0]];
    			k[0]++;
    		}
    			break;
    		case 1:
    		{

    			uint16_t m_locations1[] = {1,0,3,4,2,1};
    			m_destination = m_locations1[i[1]];
    			i[1]++;

    			uint16_t m_times1[] = {0,0,1,2,0,2};
    			m_interval = m_times1[k[1]];
    			k[1]++;
    		}
    			break;
    		case 2:
    		{

    			uint16_t m_locations2[] = {4,3,2,1,0,0};
    			m_destination = m_locations2[i[2]];
    			i[2]++;

    			uint16_t m_times2[] = {2,1,0,0,1,2};
    			m_interval = m_times2[k[2]];
    			k[2]++;
    		}
    			break;
    		case 3:
    		{

    			uint16_t m_locations3[] = {4,3,2,1,0,0};
    			m_destination = m_locations3[i[3]];
    			i[3]++;

    			uint16_t m_times3[] = {2,1,0,0,1,2};
    			m_interval = m_times3[k[3]];
    			k[3]++;
    		}
    			break;
    		case 4:
    		{

    			uint16_t m_locations4[] = {1,0,3,4,2,1};
    			m_destination = m_locations4[i[4]];
    			i[4]++;

    			uint16_t m_times4[] = {0,1,2,1,0,0};
    			m_interval = m_times4[k[4]];
    			k[4]++;
    		}
    			break;
    		default:
    			NS_LOG_DEBUG("No node!");
    		}

       // m_destination = SelectLocation(temp);
//        for(uint16_t j = 0; j < 3; j++)
//            temp[j] =  m_elem.m_locMatrix[m_currLocation][m_destination].m_timeIntervalMatrix[m_prevTime[m_currLocation][m_destination]][j].m_timeIntervalProbability;
       // m_interval = SelectTimeInterval(temp);

        NS_LOG_DEBUG("The next most probable time interval is: " << m_interval);
    }



    NS_LOG_DEBUG("Destination Value: " << m_destination);
    Vector destination = m_destLoc[m_destination];
    double speed = m_speed->GetValue ();
    double dx = (destination.x - m_current.x);
    double dy = (destination.y - m_current.y);
    double dz = (destination.z - m_current.z);
    double k = speed / std::sqrt (dx*dx + dy*dy + dz*dz);
    m_helper.SetVelocity (Vector (k*dx, k*dy, k*dz));
    m_helper.Unpause ();

    Time travelDelay = Seconds (CalculateDistance (destination, m_current) / speed);
    NS_LOG_DEBUG("Travel Delay: " << travelDelay.GetSeconds() << "s");
    m_event.Cancel ();
    m_event = Simulator::Schedule (travelDelay, &MarkovChainMobilityModel::DoInitializePrivate, this);
    //Store the current location as previous location
    m_prevLocation = m_currLocation;
    NotifyCourseChange ();
}



int
MarkovChainMobilityModel::PositionToLocation (Vector pos) const
{
    if(m_l1.IsInside(pos)){
        NS_LOG_DEBUG("This node is at L0");
        return 0;
    } else if(m_l2.IsInside(pos)){
        NS_LOG_DEBUG("This node is at L1");
        return 1;
    } else if(m_l3.IsInside(pos)){
        NS_LOG_DEBUG("This node is at L2");
        return 2;
    } else if(m_l4.IsInside(pos)){
        NS_LOG_DEBUG("This node is at L3");
        return 3;
    } else if(m_l5.IsInside(pos)){
        NS_LOG_DEBUG("This node is at L4");
        return 4;
    } else {
        NS_LOG_DEBUG("This node is Not at any of the specified locations, hence it might be traveling to the next destination");
        return 5;
    }
}

int
MarkovChainMobilityModel::LocationIdentifier(Vector pos) const
{

    if(pos.x <= 175.0 && pos.y <= 175.0 && pos.z <= 100.0){
        NS_LOG_DEBUG("Node is moving at Location 0");
        return 0;
    } else if(pos.x <= 175.0 && pos.y <= 500.0 && pos.z <= 100.0){
        NS_LOG_DEBUG("Node is moving at Location 1");
        return 1;
    } else if(pos.x > 175.0 && pos.x <= 325.0 && pos.y <= 250.0 && pos.z <= 100.0){
        NS_LOG_DEBUG("Node is moving at Location 2");
        return 2;
    } else if(pos.x > 180.0 && pos.x <= 550.0 && pos.y > 250.0 && pos.y <= 500.0 && pos.z <= 100.0){
        NS_LOG_DEBUG("Node is moving at Location 3");
        return 3;
    } else if(pos.x >= 325.0 && pos.x <= 550.0 && pos.y > 250.0 && pos.y <= 275.0 && pos.z <= 100.0){
        NS_LOG_DEBUG("Node is moving at Location 4");
        return 4;
    } else{
        NS_LOG_DEBUG("Node is moving at none of these locations");
        return 5;
    }
}

int
MarkovChainMobilityModel::TimeToTimeInterval(double timeSpent){
    NS_LOG_DEBUG("Time Spent: " << timeSpent);
    if(timeSpent >= 0.0 && timeSpent <= 5.0){
        NS_LOG_DEBUG("Time spent is Short Interval");
        return 0;
    } else if(timeSpent > 5.0 && timeSpent <= 10.0){
        NS_LOG_DEBUG("Time spent is Medium Interval");
        return 1;
    } else if(timeSpent > 10.0) {
        NS_LOG_DEBUG("Time spent is Long Interval");
        return 2;
    } else {
        NS_LOG_DEBUG("Time spent is Long Interval");
        return 2;
    }
}

double
MarkovChainMobilityModel::TimeIntervaltoTime(uint16_t timeInterval){

	switch (timeInterval){
	case 0:
		return 5.0;
	case 1:
		return 10.0;
	case 2:
		return 30.0;
	default:
		return 5.0;
	}

}

uint32_t
MarkovChainMobilityModel::SelectLocation(double pdf[5] )
{
    double cdf[5];
    double sum = 0;
    for (int i = 0;i < 5;i++) {
        sum += pdf[i];
        cdf[i] = sum;
    }
    Ptr<UniformRandomVariable> x = CreateObject<UniformRandomVariable>();
    x->SetAttribute("Min",DoubleValue(0));
    x->SetAttribute("Max",DoubleValue(1));
    double value = x->GetValue();
    if (value < cdf[0])
        return 0;
    else if (value < cdf[1])
        return 1;
    else if (value < cdf[2])
        return 2;
    else if (value < cdf[3])
        return 3;
    else
        return 4;
}


uint32_t
MarkovChainMobilityModel::SelectTimeInterval(double pdf[3] )
{
    double cdf[3];
    double sum = 0;
    for (int i = 0;i < 3;i++) {
        sum += pdf[i];
        cdf[i] = sum;
    }
    Ptr<UniformRandomVariable> x = CreateObject<UniformRandomVariable>();
    x->SetAttribute("Min",DoubleValue(0));
    x->SetAttribute("Max",DoubleValue(1));
    double value = x->GetValue();
    if (value < cdf[0])
        return 0;
    else if (value < cdf[1])
        return 1;
    else
        return 2;
}

void
MarkovChainMobilityModel::DoDispose (void)
{
 // chain up
        MobilityModel::DoDispose ();
}
Vector
MarkovChainMobilityModel::DoGetPosition (void) const
{
        m_helper.Update ();
    return m_helper.GetCurrentPosition ();
}
void
MarkovChainMobilityModel::DoSetPosition (const Vector &position)
{
        Vector oldVelocity = m_helper.GetVelocity();
        m_helper.SetPosition (position);
        m_helper.SetVelocity(oldVelocity);
    Simulator::Remove (m_event);
    m_event = Simulator::ScheduleNow (&MarkovChainMobilityModel::DoInitializePrivate, this);
}
Vector
MarkovChainMobilityModel::DoGetVelocity (void) const
{
    return m_helper.GetVelocity ();
}
int64_t
MarkovChainMobilityModel::DoAssignStreams (int64_t stream)
{
    int64_t positionStreamsAllocated;
    m_speed->SetStream (stream);
    m_pause->SetStream (stream + 1);
    NS_ASSERT_MSG (m_position, "No position allocator added before using this model");
    positionStreamsAllocated = m_position->AssignStreams (stream + 2);
    return (2 + positionStreamsAllocated);
}

} // namespace ns3
