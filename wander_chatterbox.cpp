/***************************************************************************
 * Project: wander_chatterbox                                              *
 * Author:  Ash Charles (jac27@sfu.ca)                                     *
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
#include <signal.h>
#include <stdio.h>
#include <unistd.h>
#include <string>
#include <vector>
#include <nd.h>
#include <RapiChatterbox>
#include <RapiStage>

using namespace Rapi;
CPose2d getRelPose(CPose2d absPose, CPose2d refPose)
{
	double x = cos(refPose.mYaw) * absPose.mX + sin(refPose.mYaw) * absPose.mY;
	double y = -sin(refPose.mYaw) * absPose.mY + cos(refPose.mYaw) * absPose.mY;
	double yaw = normalizeAngle(absPose.mYaw - refPose.mYaw);
	return CPose2d(x, y, yaw);
}
CVelocity2d getRelVel(CVelocity2d absVel, CPose2d refPose)
{
	double vx = cos(refPose.mYaw) * absVel.mXDot + sin(refPose.mYaw) * absVel.mYDot;
	double vy = -sin(refPose.mYaw) * absVel.mYDot + cos(refPose.mYaw) * absVel.mYDot;
	return CVelocity2d(vx, vy, absVel.mYawDot);
}

CPose2d getCarrotGoal(CPose2d pose, double r, double angleOff = 0.0)
{
	double x = r*cos(normalizeAngle(pose.mYaw+D2R(angleOff)))+pose.mX;
	double y = r*sin(normalizeAngle(pose.mYaw+D2R(angleOff)))+pose.mY;
	return CPose2d(x, y, 0.0, "goal");
}

class WanderChatterbox: public ARobotCtrl
{
public:
	WanderChatterbox(ARobot* robot) :
		ARobotCtrl(robot)
	{
		mRobot->findDevice(mDrivetrain, "drivetrain:0");
		mRobot->findDevice(mIr, "rangers:0");
		mObstacleAvoider = new CNd(0.15, 0.15, 0.15, "wander", 40*mIr->getNumSamples());
		//mObstacleAvoider->setAvoidDistance(0.4);
		//mObstacleAvoider->setSafetyDistance(0.1);
		mObstacleAvoider->addRangeFinder(mIr);
		mTime = 0.0;
	}
	;
	~WanderChatterbox()
	{
		mDrivetrain->stop();
		if (mObstacleAvoider)
			delete mObstacleAvoider;
	}
	;
private:
	ADrivetrain2dof * mDrivetrain;
	ARangeFinder * mIr;
	CNd * mObstacleAvoider;
	CPose2d mInitPose;
	double mTime;

	void updateData(float dt)
	{
		mTime += dt;
		CPose2d pose = mDrivetrain->getOdometry()->getPose();
		mObstacleAvoider->setGoal(getCarrotGoal(pose, 3.0));
		mObstacleAvoider->update(mTime, pose, mDrivetrain->getVelocity());
		mDrivetrain->setVelocityCmd(mObstacleAvoider->getRecommendedVelocity());
		mCount++;
	}
	;
};

// robotCb needs global scope for signal handler
// robotStage needs to live on passed initialisation
CCBRobot* robotCB = NULL;
CStageRobot* robotStage = NULL;
//-----------------------------------------------------------------------------
// Handle Ctrl+C on Chatterboxes.
void gotQuitSig(int signum)
{
	PRT_MSG0(4,"User requested Ctrl+C");
	if (signal(SIGINT, SIG_DFL) == SIG_ERR)
	{
		PRT_ERR1("Error resetting signal handler %s", strerror(errno));
	}
	robotCB->quit();
}
//------------------------------------------------------------------------------
// Load point for Chatterbox Robots
extern "C" int InitCB(std::string args)
{
	PRT_STATUS("Running on Chatterbox.");
	ErrorInit(4, 0);
	initRandomNumberGenerator();
	robotCB = new CCBRobot();
	if (robotCB->init() == 0)
	{
		Rapi::rapiError->print();
		delete robotCB;
		return -1;
	}
	// TODO: do I actually need this?
	if (signal(SIGINT, gotQuitSig) == SIG_ERR)
	{
		PRT_ERR1("Error setting signal handler %s", strerror(errno));
	}
	WanderChatterbox* robotCtrl = new WanderChatterbox(robotCB);
	// Blocking call
	robotCB->run();

	if (robotCtrl)
	{
		delete robotCtrl;
		robotCtrl = NULL;
	}
	if (robotCB)
		delete robotCB;
	return 0;
}
//------------------------------------------------------------------------------
// Load point for Stage Robots
extern "C" int Init(Stg::Model * mod, Stg::CtrlArgs* args)
{
	PRT_STATUS("Running simulation in Stage.");
	ErrorInit(4, 0);
	initRandomNumberGenerator();
	robotStage = new Rapi::CStageRobot(mod);
	if (robotStage->init() == 0)
	{
		Rapi::rapiError->print();
		delete robotStage;
		return -1;
	}
	WanderChatterbox* robotCtrl = new WanderChatterbox(robotStage);

	// Stage will clean up, this is just to get rid of 'unused variable' warning
	robotCtrl->rprintf("started.\n");
	return 0;
}
//------------------------------------------------------------------------------


