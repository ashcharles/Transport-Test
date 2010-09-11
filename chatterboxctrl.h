/***************************************************************************
 * Project: ash-test (RAPI)                                                *
 * Author:  Ash Charles (jac27@sfu.ca)                                     *
 * $Id: chatterboxctrl.h,v 1.4 2009-09-01 00:52:26 gumstix Exp $
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
#ifndef CHATTERBOXCTRL_H
#define CHATTERBOXCTRL_H

//#define CHATTERBOX // defined if running on chatterbox

#include <stdio.h>
#include <unistd.h>
#include <string>
#include <RapiStage>
#include <RapiChatterbox>
#include "nd.h"
#include "waypoint.h"

using namespace Rapi;

/** Type definition for state of FSM */
typedef enum { START, WORK, SEARCH, LOAD, DUMP, PAUSE, QUIT } tState;
/** Type definition for iRobot Create buttons */ 
typedef enum { PLAY_BUTTON, FAST_FORWARD_BUTTON } tButton;
/** Type definition for action results */ 
typedef enum { COMPLETED, IN_PROGRESS } tActionResult;

/**
 * A template controller for chatterbox
 * @author Ash Charles <jac27@sfu.ca>
 */
class CChatterboxCtrl : public ARobotCtrl
{
  public:
    /**
     * Default constructor
     * @param robot this controller controls
     */
    CChatterboxCtrl ( ARobot* robot );
    /** Default destructor */
    ~CChatterboxCtrl();

  private:
    //------------- functions ------------//
    /** <EM>Run</EM> action */
    tActionResult actionWork();
    /** <EM>Loading</EM> action */
    tActionResult actionLoad();
    /** <EM>Dumping</EM> action */
    tActionResult actionDump();
    /** <EM>Pause</EM> action */
    tActionResult actionPause();
    /** <EM>Search</EM> action */
    tActionResult actionSearch();
    /**
     * Checks if we are at a source or sink
     * @return true if we are there
     */
    bool isAtCargoBay();
    /**
     * Checks if we see a charger 
     * @return true if we see charger
     */
    bool isChargerDetected();
    /**
     * Update controller for the current time step
     * @param dt time since last upate [s]
     */
    void updateData( float dt );
    //------------- devices ------------//
    /** Drivetrain */
    ADrivetrain2dof * mDrivetrain;
    /** Generic Range Finder */
    ARangeFinder * mRangeFinder;
    /** Infrared sensors */
    ARangeFinder * mIr;
    /** Laser range finder */
    ARangeFinder * mLaser;
    /** Fiducial detector for charger */
    AFiducialFinder * mFiducialDetector;
    /** Odometry from drivetrain */
    COdometry * mOdo;
    /** Data Logger */
    CDataLogger * mDataLogger;
    /** ego-centric position (relative to start point) */
    CPose2d mCurrentPose;
    /** ego-centric velocity (relative to start point) */
    CVelocity2d mCurrentVelocity;
    /** Nearness Diagram (ND) obstacle avoider */
    CNd * mObstacleAvoider;
    /** Waypoint path */
    CWaypointList * mPath;
    //------------- variables ------------//
    /** Robot name */
    std::string mName;
    /** State machine */
    tState mState;
    /** Previous state of FSM */
    tState mPrevState;
    /** Name of State (data logging) */
    std::string mStateName;
    /** Elapsed state time */
    float mElapsedStateTime;
    /** Flags if the FSM state has changed in the last time step */
    bool mIsStateChanged;
    /** Accumulated run time */
    float mAccumulatedRunTime;
    /** True if robot is loaded, otherwise false */
    bool mIsLoaded;
    /** number of flags transported */
    int mFlags;
    /** Rotational transform multipliers */
    double mAngle;
};
#endif //CHATTERBOXCTRL_H
