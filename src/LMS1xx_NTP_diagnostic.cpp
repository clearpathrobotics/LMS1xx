/*
 *  LMS1xx_NTP_diagnostic.cpp
 *
 *  Created on: 24-01-2017
 *  Author: Wojciech Dudek
 ***************************************************************************
 *   This library is free software; you can redistribute it and/or         *
 *   modify it under the terms of the GNU Lesser General Public            *
 *   License as published by the Free Software Foundation; either          *
 *   version 2.1 of the License, or (at your option) any later version.    *
 *                                                                         *
 *   This library is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU     *
 *   Lesser General Public License for more details.                       *
 *                                                                         *
 *   You should have received a copy of the GNU Lesser General Public      *
 *   License along with this library; if not, write to the Free Software   *
 *   Foundation, Inc., 59 Temple Place,                                    *
 *   Suite 330, Boston, MA  02111-1307  USA                                *
 *                                                                         *
 ***************************************************************************/

#include <lms1xx/LMS1xx.h>
#include "console_bridge/console.h"
#include <stdlib.h> 

int main(int argc, char** argv){

	LMS1xx laser;
	NTPstatus ntp_status;
	scanCfg cfg;
	scanOutputRange outputRange;
	std::string host = "192.168.0.20";
	console_bridge::setLogLevel(console_bridge::CONSOLE_BRIDGE_LOG_INFO);

	if (argc == 2){
		host = argv[1];
	}else{
		logWarn("\nUsage: LMS1xx_NTP_diagnostic <HOST_IP> \nUsing default: %s",host.c_str());
	} 

	laser.connect(host);
	while (!laser.isConnected()){
  		logWarn("Unable to connect, retrying.");
      	sleep(1);
      	continue;
	} 
    // get NTP stats
    ntp_status = laser.getNTPstatus();
    // get scan params
    cfg = laser.getScanCfg();
    outputRange = laser.getScanOutputRange();
    logInform("Laser configuration: scaningFrequency %d, angleResolution %d, startAngle %d, stopAngle %d",
              cfg.scaningFrequency, cfg.angleResolution, cfg.startAngle, cfg.stopAngle);
    logInform("Laser output range: angleResolution %d, startAngle %d, stopAngle %d",
              outputRange.angleResolution, outputRange.startAngle, outputRange.stopAngle);
    logInform("Max NTP offset: %f", ntp_status.maxOffsetNTP);
    logInform("NTP time delay: %f", ntp_status.timeDelay);
}