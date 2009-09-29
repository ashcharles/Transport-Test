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

/** Time constant for voltage low pass filter [s] */
const float TAU_VOLTAGE_LPF = 5.0;
/** Battery voltage threshold to trigger charging [V] */
const float LOW_ENERGY_VOLTAGE_THRESHOLD = 14.5;
/** Battery voltage threshold of a 'full' battery [V] */
const float FULLY_CHARGED_VOLTAGE_THRESHOLD = 16.75;
/** Maximum allowable charging time [s] */
const float MAX_CHARGING_TIME = 600.0;
/** Array of State names for logging */
const std::string StateNames[] = {"Start", "Work", "Load", "Dump",
                                  "Pause", "Quit"
                                 };
//-----------------------------------------------------------------------------
CChatterboxCtrl::CChatterboxCtrl( ARobot* robot )
    : ARobotCtrl( robot )
{
  printf( "\n" );
  PRT_STATUS( "\nStage Example of Chatterbox: Follow Waypoints\n" );

  // get robot devices
  mRobot->findDevice( mDrivetrain, "drivetrain:0" );
  mRobot->findDevice( mPowerPack, "powerpack:0" );
  mRobot->findDevice( mIr, "ranger:0" );
  mRobot->findDevice( mTextDisplay, "textdisplay:0" );
  mRobot->findDevice( mFrontFiducial, "fiducial:0" );

  // Initialize robot
  char hostname[20];
  if ( gethostname( hostname, 20 ) != 0 ) { exit( EXIT_FAILURE ); }
  std::string name( hostname );
  mName = name.substr( 0, name.find( "autolab" ) );
  mState = START;
  mStateName = StateNames[mState];
  mIsLoaded = false;
  mVoltageLpf = mPowerPack->getVoltage();
  mDrivetrain->setTranslationalAccelerationLimit( CLimit( -INFINITY, 0.3 ) );
  mDrivetrain->setRotationalAccelerationLimit( CLimit( -INFINITY, INFINITY ) );

  // Setup navigation
  mObstacleAvoider = new CNd( 0.5, 0.5, 0.5 );
  assert( mObstacleAvoider );
  mObstacleAvoider->addRangeFinder( mIr );
  mObstacleAvoider->setEpsilonDistance( 0.1 );
  mObstacleAvoider->setEpsilonAngle( 10.0 );
  mOdo = mDrivetrain->getOdometry();
  mPreviousPose = mOdo->getPose();
  mPath = new CWaypointList( "waypoints.txt" );
  assert( mPath );
  mPath->populateStageWaypoints(
    ((CLooseStageDrivetrain2dof *) mDrivetrain)->getStageModel()->waypoints );

  // set up timers (in seconds)
  mElapsedStateTime = 0.0;
  mAccumulatedRunTime = 0.0;

  // Setup logging & rpc server
  char filename[40];
  sprintf( filename, "logfile_%s.log", mName.c_str() );
  mDataLogger = CDataLogger::getInstance( filename , OVERWRITE, "#" );
  mDataLogger->addVar( &mRobotPose, "Pose" );
  mDataLogger->addVar( &mVoltageLpf, "Filtered Voltage" );
  mDataLogger->addVar( &mStateName, "State name" );
}
//-----------------------------------------------------------------------------
CChatterboxCtrl::~CChatterboxCtrl()
{
  if ( mPath ) {
    delete mPath;
  }

  if ( mObstacleAvoider ) {
    delete mObstacleAvoider;
  }
}
//-----------------------------------------------------------------------------
bool CChatterboxCtrl::isChargingRequired()
{
  if ( mVoltageLpf < LOW_ENERGY_VOLTAGE_THRESHOLD ) {
    return true;
  }
  return false;
}
//-----------------------------------------------------------------------------
bool CChatterboxCtrl::isAtCargoBay()
{
  //TODO
  if ( false ) {
    return true;
  }
  return false;
}
//-----------------------------------------------------------------------------
bool CChatterboxCtrl::isChargerDetected()
{
  unsigned char ir;
  ir = mFrontFiducial->mFiducialData[0].id;

  return false;
}
//-----------------------------------------------------------------------------
tActionResult CChatterboxCtrl::actionWork()
{
//  CPose2d goal = CPose2d( mRobotPose.mX + 1.0, mRobotPose.mY,
//                          mRobotPose.mYaw + D2R(-10.0) ); // right wall follow
  mPath->update( mOdo->getPose() ); //TODO: this should be relative pose
  CWaypoint2d goal = mPath->getWaypoint();

  mObstacleAvoider->setGoal( goal.getPose() );
  mDrivetrain->setVelocityCmd( mObstacleAvoider->getRecommendedVelocity() );
  return IN_PROGRESS;
}
//-----------------------------------------------------------------------------
tActionResult CChatterboxCtrl::actionLoad()
{
  static CRgbColor color( 0, 0, 0 );
  static int loadCount = 0;

  unsigned char rate = 10;

  if ( mIsStateChanged ) {
    mDrivetrain->stop();
    mOdo->setToZero();
  }

  color.mGreen = ( color.mGreen < 110 ) ? color.mGreen + rate : 255;
  // We've filled an LED
  if ( color.mGreen >= 255 ) {
    loadCount = ( loadCount + 1 ) % 5;
    color = CRgbColor( 0, 0, 0 );
    // We're done
    if ( loadCount == 0 ) {
      PRT_STATUS( "Loading Complete!\n" );
      mIsLoaded = true;
      return COMPLETED;
    }
  }
  return IN_PROGRESS;
}
//-----------------------------------------------------------------------------
tActionResult CChatterboxCtrl::actionDump()
{
  static CRgbColor color( 0, 110, 0 );
  static int loadCount = 0;

  unsigned char rate = 10;

  if ( mIsStateChanged )
    mDrivetrain->stop();

  color.mGreen = ( color.mGreen > rate ) ? color.mGreen - rate : 0;
  // We've filled an LED
  if ( color.mGreen <= 0 ) {
    printf( "Next!\n" );
    loadCount = ( loadCount + 1 ) % 5;
    color = CRgbColor( 0, 110, 0 );
    // We're done
    if ( loadCount == 0 ) {
      PRT_STATUS( "Unloading complete!\n" );
      mIsLoaded = false;
      return COMPLETED;
    }
  }
  return IN_PROGRESS;
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
  mElapsedStateTime += dt;
  mAccumulatedRunTime += dt;
  mVoltageLpf = mVoltageLpf + ( mRobot->getUpdateInterval() / TAU_VOLTAGE_LPF )
                * ( mPowerPack->getVoltage() - mVoltageLpf );
  mRobotPose = mOdo->getPose() - mPreviousPose; // differential pose
  mPreviousPose = mOdo->getPose();
  mDataLogger->write( mAccumulatedRunTime );
  mObstacleAvoider->update( mAccumulatedRunTime,
                            mDrivetrain->getOdometry()->getPose(),
                            mDrivetrain->getVelocity() );

//******************************** START FSM **********************************
  switch ( mState ) {
    case START:
      mTextDisplay->setText( "Start" );
      mState = WORK;
      break;

    case WORK:
      mTextDisplay->setText( "Work" );
      actionWork();
      if ( isAtCargoBay() && mElapsedStateTime > 20.0 )
        mState = mIsLoaded ? DUMP : LOAD;
      break;

    case LOAD:
      mTextDisplay->setText( "Load" );
      if ( actionLoad() == COMPLETED )
        mState = WORK;
      break;

    case DUMP:
      mTextDisplay->setText( "Dump" );
      if ( actionDump() == COMPLETED )
        mState = WORK;
      break;

    case PAUSE: // state transitions done below
      mTextDisplay->setText( "Pause" );
      actionPause();
      break;

    case QUIT: // state transitions done below
      mTextDisplay->setText( "Quit" );
      mRobot->quit();
      break;

    default:
      PRT_WARN1( "Unknown FSM state %d", mState );
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
