/***************************************************************************
 * Project: ash-test (RAPI)                                                *
 * Author:  Ash Charles (jac27@sfu.ca)                                     *
 * $Id: ash-test.cpp,v 1.5 2009-09-01 20:04:05 gumstix Exp $
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
#include "chatterboxctrl.h"

// robotCb needs global scope for signal handler
// robotStage needs to live on passed initialisation
CCBRobot* robotCB = NULL;
CStageRobot* robotStage = NULL;

//-----------------------------------------------------------------------------
// Handle Ctrl+C on Chatterboxes.
void gotQuitSig(int signum) {
	PRT_MSG0(4,"User requested Ctrl+C");
	if (signal(SIGINT, SIG_DFL) == SIG_ERR) {
		PRT_ERR1("Error resetting signal handler %s", strerror(errno));
	}
	robotCB->quit();
}
//------------------------------------------------------------------------------
// Load point for Chatterbox Robots
extern "C" int InitCB(std::string args) {
	ErrorInit(4, 0);
	initRandomNumberGenerator();
	robotCB = new CCBRobot();
	if (robotCB->init() == 0) {
		Rapi::rapiError->print();
		delete robotCB;
		return -1;
	}
	// TODO: do I actually need this?
	if (signal(SIGINT, gotQuitSig) == SIG_ERR) {
		PRT_ERR1( "Error setting signal handler %s", strerror( errno ) );
	}
	CChatterboxCtrl* robotCtrl = new CChatterboxCtrl(robotCB);
	// Blocking call
	robotCB->run();

	if (robotCtrl) {
		delete robotCtrl;
		robotCtrl = NULL;
	}
	if (robotCB)
		delete robotCB;
	return 0;
}
//------------------------------------------------------------------------------
// Load point for Stage Robots
extern "C" int Init(Stg::Model * mod, Stg::CtrlArgs* args) {
	ErrorInit(4, 0);
	initRandomNumberGenerator();
	robotStage = new Rapi::CStageRobot(mod);
	if (robotStage->init() == 0) {
		Rapi::rapiError->print();
		delete robotStage;
		return -1;
	}
	CChatterboxCtrl* robotCtrl = new CChatterboxCtrl(robotStage);
	return 0;
}
//------------------------------------------------------------------------------
