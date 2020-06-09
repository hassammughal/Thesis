/*
 * markovchain-mobility-model.h
 *
 *  Created on: Apr 17, 2020
 *      Author: hassam
 */
#ifndef SRC_MOBILITY_MODEL_MARKOVCHAIN_MOBILITY_MODEL_H_
#define SRC_MOBILITY_MODEL_MARKOVCHAIN_MOBILITY_MODEL_H_

#include "ns3/object.h"
#include "ns3/nstime.h"
#include "ns3/event-id.h"
#include "ns3/box.h"
#include "ns3/random-variable-stream.h"
#include "ns3/position-allocator.h"
#include "ns3/ptr.h"
#include "mobility-model.h"
#include "constant-velocity-helper.h"
#include <ns3/vector.h>
#include <vector>
#include <map>
#include <iostream>
#include <fstream>

namespace ns3 {
/**
 * \ingroup mobility
 * \brief Markov Chain based mobility model.
 *
 * Each instance moves with a constant speed and different direction chosen from the row of a transition matrix
 * until either a fixed distance has been walked or until a fixed amount
 * of time. If we hit one of the boundaries (specified by a rectangle),
 * of the model, update our transition matrices of location and time interval.
 * This model is often identified as a Probabilistic Version of Random Walk Model.
 */

class MarkovChainMobilityModel;

typedef struct
{
    double m_timeIntervalProbability;
    uint16_t m_timeIntervalCounter;
    uint16_t m_totalTimeIntervals;
} TimeIntervalMatrix;

//We'll use TimeIntervalMatrix type as a type of a member of the following struct.
typedef struct
{
    double m_locationProbability;
    uint16_t m_locationCounter;
    uint16_t m_totalNumOfVisits;
    TimeIntervalMatrix m_timeIntervalMatrix[3][3];
} LocationMatrix;

class MatrixElement{
public:
    void SetLocationProbability(uint16_t currLoc, uint16_t prevLoc);
    void SetTimeIntervalProbability(uint16_t currLoc, uint16_t prevLoc, uint16_t currTimeInt, uint16_t prevTimeInt);

    void UpdateLocationProbability(uint16_t currLoc, uint16_t prevLoc, Ptr<MarkovChainMobilityModel> caller);
    void UpdateTimeIntervalProbability(uint16_t currLoc, uint16_t prevLoc, uint16_t currTimeInt, uint16_t prevTimeInt);

    LocationMatrix m_locMatrix[5][5];


    bool m_firstUpd[5], m_firstTimeUpd[5];
    uint16_t  currTime;

private:

};

class MarkovChainMobilityModel : public MobilityModel
{
public:
    /**
     * Register this type with the TypeId system.
     * \return the object TypeId
     */
    static TypeId GetTypeId (void);
    int PositionToLocation (Vector pos) const;
    int LocationIdentifier (Vector pos) const;
    int TimeToTimeInterval (double timeSpent);
    double TimeIntervaltoTime (uint16_t timeInterval);
    void CheckTimeInterval();
    uint16_t GetNextLocation() const;
    uint16_t GetNextTime() const;

    uint16_t m_prevTime[5][5]; //track previous time interval
    uint16_t m_currTime[5][5]; //track current time interval

protected:
    virtual void DoInitialize (void);
private:

    /**
     * Get next position, begin moving towards it, schedule future pause event
     */
    void BeginWalk (void);
    /**
     * Perform initialization of the object before MobilityModel::DoInitialize ()
     */
    void DoInitializePrivate (void);
    virtual void DoDispose (void);
    virtual Vector DoGetPosition (void) const;
    virtual void DoSetPosition (const Vector &position);
    virtual Vector DoGetVelocity (void) const;
    virtual int64_t DoAssignStreams (int64_t);

    //Get Center coordinates of the box
    Vector GetCenter(Box in);

    ConstantVelocityHelper m_helper; //!< helper for this object
    EventId m_event; //!< stored event ID
    Ptr<PositionAllocator> m_position;
    Time m_timeInterval;
    //The next location the node will move towards
    uint32_t m_destination;

    Ptr<RandomVariableStream> m_speed, m_direction, m_pause; //!< Random variable for picking speed, direction and pause
    Box m_l1, m_l2, m_l3, m_l4, m_l5; //!< Bounds of the location

    //A vector for providing the CDF function a value to select randomly from the set of destinations
    std::map<int, Vector> m_destLoc;

    uint16_t m_prevLocation; //the previous location node came from
    uint16_t m_currLocation; //the current location is stored here to be used at the next location

    int m_timePrevLoc; //different variables to track time spent in a location
    int m_timeCurrLoc;

    //Boolean variable to check if the update is 1st time or not
    bool m_firstLoc[5], m_firstArrival[5], m_firstTime;
    uint16_t m_countArrival;
    //Object of MatrixElement class
    MatrixElement m_elem;
//    uint16_t m_locations[10];
//    uint16_t m_times[10];
    //Time-intervals
    double m_prevArrivalTime[5];
    double m_timeSpent[5][5];
	uint16_t i[5] = {0};
    uint16_t k[5] = {0};
    Time m_arrivalTime[5];
    //Time m_prevTime[5][5];
    Time m_mobilityTime;
    mutable double m_arrTime[5][5];
    mutable double m_tSpent[5][5];
    mutable uint16_t previousLocation, currentLocation;
    mutable Vector pos, currPos;
    std::ofstream m_locMatricesResults;
	std::ofstream m_timeMatricesResults;
	std::ofstream m_timeSpentMatriceResults;
	std::string m_locFileName;
	std::string m_timeFileName;
	std::string m_timeSpentFileName;
    //Events
    EventId m_entryEvent, m_leavingEvent;
    //no. of nodes
    uint32_t SelectLocation(double pdf[5]);
    uint32_t SelectTimeInterval(double pdf[3]);
    int m_nodes;
    uint16_t m_interval;

};



} // namespace ns3

#endif /* SRC_MOBILITY_MODEL_MARKOVCHAIN_MOBILITY_MODEL_H_ */
