#include <stdlib.h>
#include <stdio.h>
#include "waypoint.h"

//----------------------------------------------------------------------------
CWaypointList::CWaypointList( std::string filename )
{
  FILE * file = fopen( filename.c_str(), "r" );
  if( file == NULL ) {
    fprintf( stderr, "Waypoint: Can't open file\n");
    exit( EXIT_FAILURE );
  }

  // parse file
  while( !feof( file ) ) {
    float x, y, yaw;
    char name [80];
    char line [256];
    fgets( line, 256, file );
    int args = sscanf( line, "%f,%f,%f,%s\n", &x, &y, &yaw, name );
    if ( feof( file) )
      break;
    if (args < 3)
      continue;
    yaw = D2R(yaw);
    CWaypoint2d waypoint( x, y, yaw );
    if ( args == 4 ) 
      waypoint.setLabel( name );
    mWaypoints.push_back( waypoint );
  }

  mCurrentWaypoint = mWaypoints.front();

  fclose( file );
}
//----------------------------------------------------------------------------
CWaypointList::~CWaypointList()
{
}
//----------------------------------------------------------------------------
void CWaypointList::print()
{
  std::list<CWaypoint2d>::iterator it;
  if( mWaypoints.empty() ) {
    printf( "Waypoint list is empty\n" );
    return;
  }

  for( it = mWaypoints.begin(); it != mWaypoints.end(); ++it ) {
    it->print();
  }
}
//----------------------------------------------------------------------------
void CWaypointList::update( CPose2d myPose )
{
  std::list<CWaypoint2d>::iterator it;
  if( mWaypoints.empty() ) {
    printf( "Waypoint list is empty\n" );
    return;
  }

  // find closest waypoint
  CWaypoint2d closestWaypoint;
  double distToNearestWaypoint = INFINITY;
  for( it = mWaypoints.begin(); it != mWaypoints.end(); ++it ) {
    double distToWaypoint = myPose.distance( it->getPose() );
    if( distToWaypoint < distToNearestWaypoint ) {
      distToNearestWaypoint = distToWaypoint;
      closestWaypoint = *it;
    }
  }

  // if we are at the waypoint, kick it off the list and move on
  if( (distToNearestWaypoint < 1.0) && (mWaypoints.size() > 1) ) {
    mWaypoints.pop_front();
    closestWaypoint = mWaypoints.front();
    printf( "I moved to the next waypoint:\n" );
    closestWaypoint.getPose().print();
  }
  mCurrentWaypoint = closestWaypoint;
}
//----------------------------------------------------------------------------
//CWaypoint2d * CWaypointList::findWaypoint( std::string name)
//{
//  std::list<CWaypoint2d>::iterator it;
//  for( it = mWaypoints.begin(); it != mWaypoints.end(); it++ ) {
//    if( name == it->getLabel() )
//      return &(*it);
//  }
//  return NULL;
//}
////----------------------------------------------------------------------------
//void CWaypointList::setCurrentWaypoint( std::string name )
//{
//  mCurrentWaypoint = findWaypoint( name );
//}
////----------------------------------------------------------------------------
CWaypoint2d CWaypointList::getWaypoint()
{
  CWaypoint2d currentWaypoint = mCurrentWaypoint;
  return currentWaypoint;
}
//------------------------------------------------------------------------------
void CWaypointList::populateStageWaypoints(
  std::vector<Stg::ModelPosition::Waypoint>& stgWaypoints )
{
  std::list<CWaypoint2d>::iterator it;
  for( it = mWaypoints.begin(); it != mWaypoints.end(); ++it ) {
    CPose2d pose = it->getPose();
    Stg::ModelPosition::Waypoint * stgWaypoint = new
      Stg::ModelPosition::Waypoint();
    stgWaypoint->pose.x = pose.mX;
    stgWaypoint->pose.y = pose.mY;
    stgWaypoint->pose.a = pose.mYaw;
    stgWaypoint->color = Stg::Color( 0, 1, 0 ); // green
    printf("Added waypoint ");
    pose.print();
    stgWaypoints.push_back( *stgWaypoint );
  }
}
//----------------------------------------------------------------------------
//int main( int argc, char * argv[] )
//{
//  if( argc < 2 ) {
//    fprintf( stderr, "First argument should be filename\n" );
//    exit( EXIT_FAILURE );
//  }
//  std::string filename( argv[1] );
//  WaypointList waylist( filename );
//  waylist.print();
//  CWaypoint2d find = waylist.findWaypoint( "" );
//  find.print();
//}
//----------------------------------------------------------------------------
