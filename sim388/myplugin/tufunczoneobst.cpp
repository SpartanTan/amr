/***************************************************************************
 *   Copyright (C) 2005 by Christian Andersen and DTU                      *
 *   jca@oersted.dtu.dk                                                    *
 *                                                                         *
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
 ***************************************************************************/
#include "ufunczoneobst.h"
#include "math.h"
#ifdef LIBRARY_OPEN_NEEDED
#define pi 3.14159

/**
 * This function is needed by the server to create a version of this plugin */
UFunctionBase * createFunc()
{ // create an object of this type
  /** replace 'UFunczoneobst' with your class name */
  return new UFunczoneobst();
}
#endif

bool UFunczoneobst::setResource(UResBase * resource, bool remove) { // load resource as provided by the server (or other plugins)
	bool result = true;

	if (resource->isA(UResPoseHist::getOdoPoseID())) { // pointer to server the resource that this plugin can provide too
		// but as there might be more plugins that can provide the same resource
		// use the provided
		if (remove)
			// the resource is unloaded, so reference must be removed
			poseHist = NULL;
		else if (poseHist != (UResPoseHist *) resource)
			// resource is new or is moved, save the new reference
			poseHist = (UResPoseHist *) resource;
		else
			// reference is not used
			result = false;
	}

	// other resource types may be needed by base function.
	result = UFunctionBase::setResource(resource, remove);
	return result;
}



///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////
// #define SMRWIDTH 0.4
bool UFunczoneobst::handleCommand(UServerInMsg * msg, void * extra)
{  // handle a plugin command
  const int MRL = 500;
  char reply[MRL];
  bool ask4help;
  const int MVL = 30;
  char value[MVL];
  ULaserData * data;
  double robotwidth;
  //
  double box_point1_x = 0;
  double box_point1_y = 0;
  double box_point2_x = 0;
  double box_point2_y = 0;
  double box_point_min_x = 0;
  double box_point_min_y = 0;
  double box_center[2];
  double orientation;
  //
  int i,j,imax;
  double r,delta;
  double minRange; // min range in meter
  // double minAngle = 0.0; // degrees
//   double d,robotwidth;
  double zone[9];
  // check for parameters - one parameter is tested for - 'help'
  ask4help = msg->tag.getAttValue("help", value, MVL);
  if (ask4help)
  { // create the reply in XML-like (html - like) format
    sendHelpStart(msg, "zoneobst");
    sendText("--- available zoneobst options\n");
    sendText("help            This message\n");
    sendText("fake=F          Fake some data 1=random, 2-4 a fake corridor\n");
    sendText("device=N        Laser device to use (see: SCANGET help)\n");
    sendText("see also: SCANGET and SCANSET\n");
    sendHelpDone();
  }
  else
  { // do some action and send a reply
    data = getScan(msg, (ULaserData*)extra);
    // 
    if (data->isValid())
    {
    // check if a attribute (parameter) with name width exists and if so get its value
       bool gotwidth = msg->tag.getAttValue("width", value, MVL);
       if (gotwidth) {
        	robotwidth=strtod(value, NULL);   
       }
       else {
        	robotwidth=0.26;
      }
      UPose poseAtScan = poseHist->getPoseAtTime(data->getScanTime());
      // Gets the odometry pose at the time when the laserscan was taken, poseAtScan.x poseAtScan.y poseAtScan.a (x,y,angle)
      // make analysis for closest measurement
      minRange = 1000; // long range in meters
      imax=data->getRangeCnt();
      delta=imax/9.0;
      for (j=0;j<9;j++)
	 zone[j]=minRange;
      for(j=0;j<9;j++){
      for (i = 0+(int)(j*delta); i < (int)((j+1)*delta); i++)
      { // range are stored as an integer in current units
	r = data->getRangeMeter(i);
        if (r >= 0.020)
        { // less than 20 units is a flag value for URG scanner
          if (r<zone[j])
	     zone[j]=r;
        }
      }
      }
      // Get the box center and orientation
      	#define robot_x 0
	#define robot_y 2
	#define laser_x 0.26
	#define laser_y 0
	
	double min_dist = 10000;
	bool flag = 1;
	for(i=0; i <= 500; i++ ){
		if(data -> getRangeMeter(i) < 1.3 && data -> getRangeMeter(i) > 0.2 && flag){
			flag = 0;
			double theta = i *0.36/180*pi;//resolution 0.36degrees, degree2radian
			box_point1_x = data->getRangeMeter(i) * sin(theta) + robot_x + laser_x;
			box_point1_y = data->getRangeMeter(i) * cos(theta) + robot_y + laser_y;
		}
		if(data -> getRangeMeter(i) < 1.3 && data -> getRangeMeter(i) > 0.2 && data -> getRangeMeter(i) < min_dist) {
			double theta = i*0.36/180*pi;
			min_dist = data->getRangeMeter(i);//once got a smaller distance, replacing the min_dist
		    	box_point_min_x = data->getRangeMeter(i) * sin(theta) + robot_x + laser_x;
		     	box_point_min_y = data->getRangeMeter(i) * cos(theta) + robot_y + laser_y;
		}
	}
      for(i = 500; i >= 0; --i) {
	      	if(data->getRangeMeter(i) < 1.3 && data->getRangeMeter(i) > 0.2) {
		      	double theta = i*0.36/180*pi;
		      	box_point2_x = data->getRangeMeter(i) * sin(theta) + robot_x + laser_x;
		      	box_point2_y = data->getRangeMeter(i) * cos(theta) + robot_y + laser_y;
		      	break;
	      }
      }	
      box_center[0] = (box_point1_x + box_point2_x) / 2;
      box_center[1] = (box_point1_y + box_point2_y) / 2;
      orientation = atan((box_point1_y-box_point_min_y)/(box_point1_x-box_point_min_x));

      /* SMRCL reply format */
      //snprintf(reply, MRL, "<box center=\"%g\" \"%g\" orientation of long side=\"%g\"/>\n", box_center[0], box_center[1], orientation);
      //snprintf(reply, MRL, "<min=\"%g\", \"%g\" point_1=\"%g\", \"%g\"/>\n", box_point_min_x, box_point_min_y, box_point1_x, box_point1_y);
      snprintf(reply, MRL, "<laser l1=\"%g\" l1=\"%g\" l2=\"%g\" l3=\"%g\" l4=\"%g\" "
                                  "l5=\"%g\" l6=\"%g\" l7=\"%g\" l8=\"%g\" boxx=\"%g\" boxy=\"%g\" orientation=\"%g\"/>\n", 
	                  zone[0],zone[1],zone[2],zone[3],zone[4],
                           zone[5],zone[6],zone[7],zone[8],
			   box_center[0], box_center[1], orientation);
			   
      // send this string as the reply to the client
      sendMsg(msg, reply);
      // save also as gloabl variable
      for(i = 0; i < 9; i++)
      		var_zone->setValued(zone[i], i);
	var_zone->setValued(box_center[0], 9);
	var_zone->setValued(box_center[1], 10);
	var_zone->setValued(orientation, 11);   

    }
    else
      sendWarning(msg, "No scandata available");
  }
  // return true if the function is handled with a positive result
  // used if scanpush or push has a count of positive results
  return true;
}

void UFunczoneobst::createBaseVar()
{ // add also a global variable (global on laser scanner server) with latest data
  var_zone = addVarA("zone", "0 0 0 0 0 0 0 0 0 0 0 0", "d", "Value of each laser zone. Updated by zoneobst.");
}
