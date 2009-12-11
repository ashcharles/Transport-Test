/***************************************************************************
 * Project: ash-test (RAPI)                                                *
 * Author:  Ash Charles (jac27@sfu.ca)                                     *
 * $Id: chatterboxctrl.cpp,v 1.5 2009-09-01 20:04:06 gumstix Exp $
 ***************************************************************************
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 **************************************************************************/
#include "chatterboxctrl.h"

/** Number of flags to transport */
const int NUM_FLAGS = 10;

/** Array of State names for logging */
const std::string StateNames[] = { "Start", "Work", "Search", "Load",
                                   "Dump", "Pause", "Quit" };

//-----------------------------------------------------------------------------
/** Utility function to transform coordinate systems */
void transform( double angle, double x0, double y0, double yaw0, double &x1,
                double &y1, double &yaw1 )
{
  x1 = cos( angle ) * x0 + sin( angle ) * y0;
  y1 = -sin( angle ) * x0 + cos( angle ) * y0;
  yaw1 = yaw0;
}
//-----------------------------------------------------------------------------
CChatterboxCtrl::CChatterboxCtrl( ARobot* robot ) : ARobotCtrl( robot )
{
  // Initialize robot
  char hostname[20];
  gethostname( hostname, 20 );
  std::string name( hostname );
  mName = name.substr( 0, name.find( "." ) ); // e.g. 'walle.autolab'
  mState = START;
  mStateName = StateNames[mState];
  mIsLoaded = false;
  mFlags = 0;
  mElapsedStateTime = 0.0;
  mAccumulatedRunTime = 0.0;

  // get robot devices
#ifndef CHATTERBOX 
  PRT_STATUS( "Running simulation in Stage" );
  mRobot->findDevice( mDrivetrain, "drivetrain:0" );
  mRobot->findDevice( mLaser, "laser:0" );
  mRobot->findDevice( mFiducialDetector, "fiducial:0" );
  mRobot->findDevice( mIr, "ranger:0" );
#else
  PRT_STATUS( "Running on chatterbox" );
  mRobot->setUpdateInterval( 0.5 );
  mRobot->findDevice( mDrivetrain, "CB:drivetrain" );
  ((CCBDrivetrain2dof*) mDrivetrain)->setDefaultOIMode( CB_MODE_FULL );
  mRobot->findDevice( mLaser, "CB:laser" );
  mRobot->findDevice( mFiducialDetector, "CB:front_fiducial" );
  mRobot->findDevice( mIr, "CB:ir" );
#endif
  mRangeFinder = mIr;
  if( mLaser ) {
    PRT_STATUS( "Using a laser device" );
    mRangeFinder = mLaser;
  }

  // Setup navigation
  mPath = new CWaypointList( "source2sink.txt" );
  mObstacleAvoider = new CNd( 0.3, 0.3, 0.3, mName,
                              1 * mRangeFinder->getNumSamples() );
  mObstacleAvoider->setEpsilonDistance( 0.3 );
  mObstacleAvoider->setEpsilonAngle( M_PI );
  mObstacleAvoider->addRangeFinder( mRangeFinder );
  mOdo = mDrivetrain->getOdometry();
  mAngle = mOdo->getPose().mYaw;
  mOdo->setToZero(); // reset local odometry

  // Setup logging 
  char filename[40];
  sprintf( filename, "logfile_%s.log", mName.c_str() );
  mDataLogger = CDataLogger::getInstance( filename , OVERWRITE, "#" );
  mDataLogger->addVar( &mStateName, "State name" );
}
//-----------------------------------------------------------------------------
CChatterboxCtrl::~CChatterboxCtrl()
{
  mDrivetrain->stop();
  if ( mObstacleAvoider )
    delete mObstacleAvoider;
  if( mPath )
    delete mPath;
}
//-----------------------------------------------------------------------------
bool CChatterboxCtrl::isAtCargoBay()
{
  return isChargerDetected();
}
//-----------------------------------------------------------------------------
bool CChatterboxCtrl::isChargerDetected()
{
  unsigned char ir;
  if( mFiducialDetector->getNumReadings() == 0 )
    return false;
  ir = mFiducialDetector->mFiducialData[0].id;

#ifndef CHATTERBOX
  if( ir == 6 ) {
#else
  if( (ir == CB_RED_BUOY) ||
      (ir == CB_GREEN_BUOY) ||
      (ir == CB_FORCE_FIELD) ||
      (ir == CB_RED_GREEN_BUOY) ||
      (ir == CB_RED_BUOY_FORCE_FIELD) ||
      (ir == CB_GREEN_BUOY_FORCE_FIELD) ||
      (ir == CB_RED_GREEN_BUOY_FORCE_FIELD) ) {
#endif
    PRT_STATUS( "See charger" );
    return true;
  }
  return false;
}
//-----------------------------------------------------------------------------
tActionResult CChatterboxCtrl::actionWork()
{
  mPath->update( mCurrentPose );
  mObstacleAvoider->setGoal( mPath->getWaypoint().getPose() );
  mPath->getWaypoint().getPose().print();
  mDrivetrain->setVelocityCmd( mObstacleAvoider->getRecommendedVelocity() );
  return ( mPath->mFgAtEnd ? COMPLETED : IN_PROGRESS );
}
//-----------------------------------------------------------------------------
tActionResult CChatterboxCtrl::actionSearch()
{
  double turnTime = 10.0; // turn on the spot [s]
  double goalTime = 4.0; // try to reach new goal [s]

  double modTime = 0.1 * floor( 10 * fmod( mElapsedStateTime,
                                turnTime + goalTime ) );
  if( mIsStateChanged || (modTime == 0.0 ) ) {
    mDrivetrain->stop();
    mDrivetrain->setRotationalVelocityCmd( -2.0 * M_PI / turnTime );
  }
  else if( modTime == turnTime ) {
    CPose2d goal( (float(rand()) / RAND_MAX ), (float(rand()) / RAND_MAX ), 0.0 );
    mObstacleAvoider->setGoal( goal + mCurrentPose );
  }
  else if( modTime > turnTime ) {
    mDrivetrain->setVelocityCmd( mObstacleAvoider->getRecommendedVelocity() );
  }

  return ( isAtCargoBay() ? COMPLETED : IN_PROGRESS );
}
//-----------------------------------------------------------------------------
tActionResult CChatterboxCtrl::actionLoad()
{
  const double turnAngle = 0.5 * M_PI; // [rad]
  const double turnTime = 10.0; // [s]

  if ( mIsStateChanged ) {
    mDrivetrain->stop();
    mDrivetrain->setRotationalVelocityCmd( -turnAngle/turnTime );
  }

  if( mElapsedStateTime > turnTime ) {
    mOdo->setToZero();
    delete( mPath );
    if( mIsLoaded ) {
      PRT_MSG1( 0, "%d flags transported", ++mFlags );
      mIsLoaded = false;
      mAngle = 0.75 * M_PI;
      mPath = new CWaypointList( "source2sink.txt" );
    }
    else {
      PRT_STATUS( "Loading complete!" );
      mIsLoaded = true;
      mAngle = -0.25 * M_PI;
      mPath = new CWaypointList( "sink2source.txt" );
    }
    return COMPLETED;
  }

  return IN_PROGRESS;
}
//-----------------------------------------------------------------------------
tActionResult CChatterboxCtrl::actionDump()
{
  return actionLoad();
}
//-----------------------------------------------------------------------------
tActionResult CChatterboxCtrl::actionPause()
{
  mDrivetrain->stop();
  return IN_PROGRESS;
}
//-----------------------------------------------------------------------------
void CChatterboxCtrl::updateData( float dt )
{
  static tState prevTimestepState = mState;

  // household chores
  CPose2d pos = mOdo->getPose();
  transform( mAngle, pos.mX, pos.mY, pos.mYaw, mCurrentPose.mX,
             mCurrentPose.mY, mCurrentPose.mYaw );
  CVelocity2d vel = mDrivetrain->getVelocity();
  transform( mAngle, vel.mXDot, vel.mYDot, vel.mYawDot, mCurrentVelocity.mXDot,
             mCurrentVelocity.mYDot, mCurrentVelocity.mYawDot );
  mElapsedStateTime += dt;
  mAccumulatedRunTime += dt;
  mDataLogger->write( mAccumulatedRunTime );
  mObstacleAvoider->update( mAccumulatedRunTime, mCurrentPose ,
                            mCurrentVelocity );

//******************************** START FSM **********************************
  switch ( mState ) {
    case START:
      mState = WORK;
      break;

    case WORK:
      if ( actionWork() == COMPLETED )
        mState = SEARCH;
      break;

    case SEARCH:
      if ( actionSearch() == COMPLETED )
        mState = mIsLoaded ? DUMP : LOAD;
      break;

    case LOAD:
      if ( actionLoad() == COMPLETED )
        mState = WORK;
      break;

    case DUMP:
      if ( actionDump() == COMPLETED )
        mState = ( mFlags < NUM_FLAGS ) ? WORK : QUIT;
      break;

    case PAUSE: // state transitions done below
      actionPause();
      break;

    case QUIT:
      PRT_STATUS( "Quitting...");
      mRobot->quit();
      break;

    default:
      PRT_WARN1( "Unknown state #%d", mState );
      mState = START;
      break;
  }
//******************************** END FSM ************************************

  // advance states for next time step
  if ( prevTimestepState != mState ) {
    mIsStateChanged = true;
    mPrevState = prevTimestepState;
    mElapsedStateTime = 0.0;
    mStateName = StateNames[mState];
  }
  else {
    mIsStateChanged = false;
  }
  prevTimestepState = mState; 

  // check for any errors
  if ( rapiError->hasError() ) {
    rapiError->print();
  }
} // updateData
