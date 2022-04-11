// Includes all of the ROS libraries needed
#include "ros/ros.h"
#include <sstream>
#include "math.h"

#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"

// libraries needed for subscribers and publishers
#include "usv_msgs/RangeBearing.h" 
#include "nav_msgs/Odometry.h"
#include "geographic_msgs/GeoPoseStamped.h"
#include "amore/NED_acoustic.h"											// msg that holds location of acoustic source in NED
#include "geometry_msgs/Point.h"										// message type used to hold the goal waypoints w/headings
#include "amore/state_msg.h"												// message type used to recieve state of operation from mission_control

#define PI 3.14159265

using namespace std;

// GLOBAL VARIABLES
int loop_count = 0;																											// loop counter, first 10 loops used to intitialize subscribers  (ACOUSTICS_inspector function)

// STATES CONCERNED WITH "acoustics" 
int A_state = 0;
int k=0; // Counter used to keep track of how many times a new source location has been obtained
// 0 = On standby
// 1 = Navigating to acoustic source

float range_msg, bearing_msg, elevation_msg; 																									// range, bearing, and elevation of acoustic source wrt USV (sub_acoustic function)
float x_usv_NED, y_usv_NED, psi_NED; 																												// vehicle position and heading (pose) in NED (sub_USVloc function)
float source_x_calc, source_y_calc; 
float source_x[10] = {0,0,0,0,0,0,0,0,0,0}, source_y[10] = {0,0,0,0,0,0,0,0,0,0}, goal_poses_quantity = 1;			// x,y position of acoustic source in global coordiates (sourceloc_calc function)
float source_lat, source_long; 																																// lat and long coordinates of acoustic source (sub_sourceloc function)

bool system_initialized = false;																																// false means the system has not been initialized (ACOUSTICS_inspector function)
bool source_goal_recieved = false; 																														// source_goal_recieved = false, acoustic source goal has not been acquired

ros::Time current_time, last_time;																															// creates time variables

// PUBLISHERS
std_msgs::Bool a_initialization_status;													// "a_initialization_state" message
ros::Publisher a_initialization_state_pub;												// "a_initialization_state" publisher

amore::NED_acoustic acoustic_source_location_msg;							// "acoustic_source_location" message
ros::Publisher acoustic_source_location_pub;										// "acoustic_source_location" publisher

std_msgs::Bool acoustic_sourceloc_publish_status;								// "acoustic_source_location_state" message; false means goal NED acoustic source location has not been published
ros::Publisher acoustic_sourceloc_publish_state_pub;							// "acoustic_source_location_state" publisher for whether NED acoustic source location has been published

// FUNCTIONS

// THIS FUNCTION: Determines if acoustics is ready to be used
// ACCEPTS: (VOID)
// RETURNS: (VOID)
// =============================================================================
void ACOUSTICS_inspector(void)
{
	current_time = ros::Time::now();   		// sets current_time to the time it is now
	loop_count += 1;									// increment loop counter
	if (loop_count > 10)
	{
		system_initialized = true;
		//ROS_INFO("acoustics_initialized -- NA");
	}
	else
	{
		system_initialized = false;
		//ROS_INFO("!acoustics_initialized -- NA");
	}
	a_initialization_status.data = system_initialized;
	a_initialization_state_pub.publish(a_initialization_status);						// publish the initialization status of acoustics to "a_initialization_state"
} // END OF PERCEPTION_ARRAY_inspector()

// THIS FUNCTION: Updates the state of "acoustics" given by "mission_control"
// ACCEPTS: acoustics state_msg from "a_state"
// RETURNS: (VOID) Updates global variable
// =============================================================================
void a_state_update(const amore::state_msg::ConstPtr& msg) 
{
	if (system_initialized)
	{
		A_state = msg->state.data;
	}
} // END OF a_state_update()

// THIS FUNCTION: Retrieves range, bearing, and elevation of acoustic source
// ACCEPTS: Current acoustic source info from "/wamv/sensors/pingers/pinger/range_bearing" topic
// RETURNS: (VOID)
// =============================================================================
void sub_acoustic(const usv_msgs::RangeBearing::ConstPtr& acoustic_msg)
{
	if(!source_goal_recieved)
	{
		range_msg = acoustic_msg->range;
		bearing_msg = acoustic_msg->bearing;
		elevation_msg = acoustic_msg->elevation;
	}
	
	//cout<<"The range is: "<<range_msg<<endl;
	//cout<<"The bearing is: "<<bearing_msg<<endl;
	//cout<<"The elevation is: "<<elevation_msg<<endl;
} // END OF sub_acoustic()

// THIS FUNCTION: Retrieves current USV pose in NED
// ACCEPTS: Current USV NED pose info from "nav_ned" topic
// RETURNS: (VOID)
// =============================================================================
void sub_USVloc(const nav_msgs::Odometry::ConstPtr& USVloc)
{
	// Update NED USV pose 
	x_usv_NED = USVloc->pose.pose.position.x;
	y_usv_NED = USVloc->pose.pose.position.y;
	psi_NED = USVloc->pose.pose.orientation.z;
 
	//cout<<"The x position is: "<<x_usv_NED <<endl;
	//cout<<"The y position is: "<<y_usv_NED <<endl;
	//cout<<"The orientation is:"<<psi_NED<<endl;
} // END OF sub_USVloc()

// THIS FUNCTION: Retrieves acoustic source location in WGS84
// ACCEPTS: Acoustic source location info from "/vrx/gymkhana_blackbox/goal" topic -- NOT USED IN COMPETITION
// RETURNS: (VOID)
// =============================================================================
void sub_sourceloc(const geographic_msgs::GeoPoseStamped::ConstPtr& sourceloc)
{	
	source_lat = sourceloc->pose.position.latitude;
	source_long = sourceloc->pose.position.longitude;
 
	//cout<<"The latitude is: "<<source_lat <<endl;
	//cout<<"The longitude is: "<<source_long<<endl;
} // END OF sub_sourceloc()

// THIS FUNCTION: Calculates location of acoustic pinger and publishes it to "acoustic_source_location"
// ACCEPTS: (VOID)
// RETURNS: (VOID)
// =============================================================================
void sourceloc_calc(void)
{
	bool steady_state = false; // Checks to see if the source location being read is consistent across multiple readings
	if(!acoustic_sourceloc_publish_status.data)
	{
		source_x_calc = range_msg*cos(psi_NED-bearing_msg) + x_usv_NED;//x location of the acoustic source
		source_y_calc = range_msg*sin(psi_NED-bearing_msg) + y_usv_NED; //y location of the acoustic source
		
		//cout<<"What should be in source_x[k]:"<<range_msg*cos(psi_NED-bearing_msg) + x_usv_NED<<endl;
		//cout<<"What should be in source_y[k]: "<<range_msg*sin(psi_NED-bearing_msg) + y_usv_NED<<endl;
		//cout<<"Calculated source x"<<source_x_calc<<endl;
		//cout<<"Calculated source y"<<source_y_calc<<endl;
			
		source_x[k] = source_x_calc; 
		source_y[k] = source_y_calc;
		
		//cout<<"The source x position is: "<<source_x[k] <<endl;
		//cout<<"The source y position is:"<<source_y[k]<<endl;
	
		k = k+1;
		cout<<"k: "<<k<<endl;
		if(k == 10)
		{
			cout<<"k has reached 10"<<endl;
			cout<<"   "<<endl;
			if((abs(source_x[9] - source_x[8])<3)&&(abs(source_x[9] - source_x[7])<3)&&(abs(source_y[9] - source_y[8])<3)&&(abs(source_y[9] - source_y[8])<3))
			{
				k = 0;
				steady_state = true; 
			
				geometry_msgs::Point point;
				acoustic_source_location_msg.quantity = 1; 
				point.x = source_x[9];
				point.y = source_y[9];
				point.z = 0; //orientation does not matter so it was set to 0
				acoustic_source_location_msg.points.push_back(point);
			}
			else 
			{
				k = 0; 
			}
		}
	}
	
	if(steady_state==true)
	{
		acoustic_source_location_pub.publish(acoustic_source_location_msg); //publish acoustic source location to "acoustic_source_location"
		acoustic_sourceloc_publish_status.data = true;
		steady_state = false;
		//ROS_INFO("WAYPOINTS HAVE BEEN PUBLISHED -- NA");
	}
	
	//EVENTUALLY NEED TO HAVE A BOOL SHOWING THAT THIS MESSAGE WAS  PUBLISHED??
	
	//cout<<"We calculated the x position to be: "<<source_x<<endl;
	//cout<<"We calculated the y position to be: "<<source_y<<endl;
	
} // END OF beaconloc()

int main(int argc, char **argv)
{
	// names the program for visual purposes
	ros::init(argc, argv, "acoustics");
	ros::NodeHandle n;
  
	// Subscribers
	ros::Subscriber ping = n.subscribe("/wamv/sensors/pingers/pinger/range_bearing", 1000, sub_acoustic); 				// gets range, bearing, and elevation of acoustic source
	ros::Subscriber USVned = n.subscribe("nav_ned", 1000, sub_USVloc); 																// gets the current NED pose of the USV
	ros::Subscriber sourcespher= n.subscribe("/vrx/gymkhana_blackbox/goal", 1000, sub_sourceloc); 						// gets the location of the acoustic source in WGS84 coordinates (NOT AVAILABLE DURING COMPETITION)
	ros::Subscriber a_state_sub = n.subscribe("a_state", 1, a_state_update);
	
	// Publishers
	a_initialization_state_pub = n.advertise<std_msgs::Bool>("a_initialization_state", 1);												// publisher for state of initialization
	acoustic_source_location_pub = n.advertise<amore::NED_acoustic>("acoustic_source_location", 1);						// publisher for NED location of acoustic source
	acoustic_sourceloc_publish_state_pub = n.advertise<std_msgs::Bool>("acoustic_sourceloc_publish_state", 1);		// "acoustic_sourceloc_publish_state" publisher for whether NED source location has been published
	
	// Initialize global variables
	acoustic_sourceloc_publish_status.data = false;
	a_initialization_status.data = false;
	current_time = ros::Time::now();   											// sets current time to the time it is now
	last_time = ros::Time::now();        											// sets last time to the time it is now
	
	// sets the frequency for which the program sleeps at. 10=1/10 second
	ros::Rate loop_rate(10);
	
	// rosk::ok() will stop when the user inputs Ctrl+C
	while(ros::ok())
	{
		ACOUSTICS_inspector();
		acoustic_sourceloc_publish_state_pub.publish(acoustic_sourceloc_publish_status);	// publish whether NED converted waypoints have been published
		
		if(A_state == 0)
		{
			// reset
			bool system_initialized = false;																							
			bool source_goal_recieved = false; 	
		}
		else if (A_state == 1)
		{
			sourceloc_calc(); // function getting beacon location
		}
	
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}	

