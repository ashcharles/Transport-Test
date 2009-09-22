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

#include "RapiStage"
#ifdef RAPI_GUI
#include "RapiGui"
#endif
#include "chatterboxctrl.h"

//------------------------------------------------------------------------------
extern "C" int Init ( Stg::Model * mod )
{
  Rapi::CStageRobot* robot = NULL;
  CChatterboxCtrl* robotCtrl = NULL;

  // init general stuff
  ErrorInit ( 8, false );
  initRandomNumberGenerator();

  // create robot and its controller
  robot = new Rapi::CStageRobot( mod );
  robotCtrl = new CChatterboxCtrl( robot );

#ifdef RAPI_GUI
  CGui * gui = Rapi::CGui::getInstance( 0, NULL );
  gui->registerRobot( robot );
#endif

  return 0;
}
