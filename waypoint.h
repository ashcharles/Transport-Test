#ifndef WAYPOINTLIST_H
#define WAYPOINTLIST_H

#include <RapiStage>
//#include <RapiCore>
#include <list>
#include <string>

using namespace Rapi;

class CWaypointList
{
  public:
    CWaypointList( std::string filename );
    ~CWaypointList();
    void print();
    void update( CPose2d myPose );
    //CWaypoint2d * findWaypoint( std::string name);
    //void setCurrentWaypoint( std::string name );
    CWaypoint2d getWaypoint();
    //CWaypoint2d * getNextWaypoint();
    void populateStageWaypoints( std::vector
      <Stg::ModelPosition::Waypoint>& stgWaypoints);
  private:
    std::list<CWaypoint2d> mWaypoints;
    CWaypoint2d mCurrentWaypoint;
};


#endif //WAYPOINTLIST_H
