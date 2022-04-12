//  Filename:											path_planner.cpp
//  Creation Date:									04/07/2022
//  Last Revision Date:							04/07/2022
//  Author(s) [email]:								Bradley Hacker [bhacker@lssu.edu]
//  Revisor(s) [email] {Revision Date}:	Bradley Hacker [bhacker@lssu.edu] {04/07/2022}
//  Organization/Institution:					Lake Superior State University - Team AMORE
// 
// ...............................About path_planner.cpp......................................
//  This code acts as the autonomous state machine of the WAM-V USV.
//  It will subscribe to the vrx/task/info to control the state of the system.
//  This code will subscribe to goal poses given from the gps_imu node.
//  Dependent on the current task state and system state, mission_control
//  will publish whether or not the low level controllers should be on, as well 
//  as the goal of the low level controllers.
//
//  Inputs and Outputs of the path_planner.cpp file
//				Inputs [subscribers]: "waypoints_NED" (converted goal pose array), "usv_ned", "/vrx/task/info", 
//				Outputs [publishers]: state and goal of low level controllers


//................................................Included Libraries and Message Types..........................................
#include "ros/ros.h"
#include "ros/console.h"
#include "time.h"
#include <sstream>
#include <iostream>
#include "math.h"
#include "stdio.h"
#include "nav_msgs/Odometry.h"					// message type used for receiving NED USV state from navigation_array
#include "amore/state_msg.h"						// message type used to recieve state of operation from mission_control
#include "std_msgs/Bool.h"							// message used to communicate publish state to propulsion_system
#include "amore/usv_pose_msg.h"				// message that holds usv position as a geometry_msgs/Point and heading in radians as a Float64
#include "amore/NED_waypoints.h"				// message that holds array of converted WF goal waypoints w/ headings and number of waypoints
#include "amore/NED_acoustic.h"					// message that holds position of beacon
#include "amore/NED_objects.h"
//#include "geometry_msgs/PoseArray.h"		// message type used to get buoy locations from navigation_array
//...........................................End of Included Libraries and Message Types....................................


//.................................................................Constants....................................................................
#define PI 3.14159265
//............................................................End of Constants.............................................................


//..............................................................Global Variables............................................................
int loop_count = 0;                                    				// loop counter, first 10 loops used to intitialize subscribers
bool system_initialized = false;								// false means the system has not been initialized

//	STATES CONCERNED WITH "navigation_array"
int NA_state = 0;
//	0 = On standby
//	1 = USV NED pose converter
//	2 = Station-Keeping NED goal pose converter
//	3 = Wayfinding NED goal pose converter
//	4 = Wildlife NED animals converter

//	STATES CONCERNED WITH "path_planner"
int PP_state = 0;
//	0 = On standby
//	1 = Task 1: Station-Keeping
//	2 = Task 2: Wayfinding
//	4 = Task 4: Wildlife Encounter and Avoid
//	5 = Task 5: Channel Navigation, Acoustic Beacon Localization and Obstacle Avoidance
//	6 = Task 6: Scan and Dock and Deliver
	
//	STATES CONCERNED WITH "propulsion_system"
int PS_state = 0;
//	0 = On standby
//	1 = Propulsion system ON

//	STATES CONCERNED WITH "perception_array"
int PA_state = 0;
//	0 = On standby
//	1 = General State
//	3 = Task 3: Landmark Localization and Characterization
//	4 = Task 4: Wildlife Encounter and Avoid
//	5 = Task 5: Channel Navigation, Acoustic Beacon Localization and Obstacle Avoidance
//	6 = Task 6: Scan and Dock and Deliver

// STATES CONCERNED WITH "acoustics" 
int A_state = 0;
// 0 = On standby
// 1 = Finding entrance gate (white buoy)
// 2 = Navigating between red and green buoys
// 3 = Finding exit gate (black buoy)
// 4 = Navigating to acoustic source

int point = 0;                     		    							// number of points on trajectory reached 
int goal_poses;              											// total number of poses to reach 
int loop_goal_recieved;         									// this is kept in order to ensure planner doesn't start controller until the goal is published

float x_goal[100], y_goal[100], psi_goal[100];		// arrays to hold the NED goal poses
std::string Animal[10];										// string array to hold the names of the animals
float x_animals_NED[10], y_animals_NED[10];	// arrays to hold animal locations
float x_usv_NED, y_usv_NED, psi_NED; 			// vehicle position and heading (pose) in NED

float e_x, e_y, e_xy, e_psi;									// current errors between goal pose and usv pose

bool NA_goal_recieved = false;				// false means NED goal poses have not been acquired from navigation_array for task 1 or 2
bool E_reached = false;        									// false means the last point has not been reached

float e_xy_allowed = 0.4;       								// positional error tolerance threshold; NOTE: make as small as possible
float e_psi_allowed = 0.4;      									// heading error tolerance threshold; NOTE: make as small as possible

float x_c_NED, y_c_NED, psi_c_NED; 				// crocodile position and heading (pose) in NED
float x_p_NED, y_p_NED, psi_p_NED; 				// platypus position and heading (pose) in NED
float x_t_NED, y_t_NED, psi_t_NED; 				// turtle position and heading (pose) in NED
float r; 															//Radius for the circles

//Array of poses for making the circle for the turtle
float x_turt_g[9];
float y_turt_g[9];
float psi_turt_g[9];

//Array of poses for making the circle for the platypus
float x_plat_g[9];
float y_plat_g[9];
float psi_plat_g[9];

float dc_USV,dt_USV,dp_USV,dp_c,dt_c;      //DIstances between animals and USV in NED


// VARIABLES FOR THE BUOY NAVIGATION CALCULATIONS
// for calculating desired poses
float CL_x;				// x-location of left buoy wrt USV
float CL_y;				// y-location of left buoy wrt USV
float CR_x;				// x-location of right buoy wrt USV
float CR_y;				// y-location of right buoy wrt USV

float CL_x_NED;	// x-location of left buoy centroid in global frame
float CL_y_NED;	// y-location of left buoy centroid in global frame
float CR_x_NED;	// x-location of right buoy centroid in global frame
float CR_y_NED;	// y-location of right buoy centroid in global frame

float I_x;					// x-coord. of intermediate point wrt global 
float I_y;					// y-coord. of intermediate point wrt global

float M_x;				// x-location of midpoint
float M_y;				// y-location of midpoint

float E_x;				// x-coord. of exit point wrt global 
float E_y;				// y-coord. of exit point wrt global

float d_L;					// distance from USV to left buoy
float d_M;				// distance from USV to midpoint
float d_R;				// distance from USV to right buoy
float d_I;					// distance from midpoint to approach point

float d_LM;				// half distance between left and right buoys
float a_L;					// distance between left buoy and the approach point
float theta;				// angle created by d_I and d_LM

float x_I_CL;			// x-coord. of intermediate point wrt left buoy 
float y_I_CL;			// y-coord. of intermediate point wrt left buoy 

float x_E_CL;			// x-coord. of exit point wrt left buoy 
float y_E_CL;			// y-coord. of exit point wrt left buoy 

float s_M;				// slope of of line from CL to CR 
float alpha;				// angle of frame CL wrt global frame

bool NED_buoys_recieved = false;			// false means the NED_buoys array has not yet been acquired
bool calculations_done = false; 				// true means ... intermediate approach, mid-, and exit points have been calculated and updated globally if PP_state = 5, if PP_state = 4 then this means the path has been made

std_msgs::Bool pp_initialization_status;							// "pp_initialization_state" message
ros::Publisher pp_initialization_state_pub;						// "pp_initialization_state" publisher

std_msgs::Bool goal_pose_publish_status;						// "goal_pose_publish_state" message; false means goal NED pose has not been published
ros::Publisher goal_pose_publish_state_pub;					// "goal_pose_publish_state" publisher for whether goal NED pose has been published

amore::usv_pose_msg current_goal_pose_msg;			// "current_goal_pose" message
ros::Publisher current_goal_pose_pub;							// "current_goal_pose" publisher

ros::Time current_time, last_time;									// creates time variables
//..............................................................End of Global Variables..........................................................


//..................................................................Functions...........................................................................
// THIS FUNCTION: Updates global current_time, loop_count, and publishes initialization status to "pp_initialization_state"
// ACCEPTS: (VOID)
// RETURNS: (VOID)
// =============================================================================
void PATH_PLANNER_inspector()
{
	current_time = ros::Time::now();   		// sets current_time to the time it is now
	loop_count += 1;									// increment loop counter
	if (loop_count > 10)
	{
		system_initialized = true;
		//ROS_INFO("path_planner_initialized -- PP");
	}
	else
	{
		system_initialized = false;
		ROS_INFO("!path_planner_initialized -- PP");
	}
	pp_initialization_status.data = system_initialized;
	pp_initialization_state_pub.publish(pp_initialization_status);		// publish the initialization status of the path_planner to "pp_initialization_state"
} // END OF PATH_PLANNER_inspector()

/////////////////////////////////////////////////////////////////		STATE UPDATERS		///////////////////////////////////////////////////////////////////
// THIS FUNCTION: Updates the state of "navigation_array" given by "mission_control"
// ACCEPTS: navigation_array state_msg from "na_state"
// RETURNS: (VOID)		Updates global variables
// =============================================================================
void na_state_update(const amore::state_msg::ConstPtr& msg)
{
	if (system_initialized)
	{
		NA_state = msg->state.data;
	}
} // END OF na_state_update()

// THIS FUNCTION: Updates the state of "path_planner" given by "mission_control"
// ACCEPTS: path_planner state_msg from "pp_state"
// RETURNS: (VOID)		Updates global variables
// =============================================================================
void pp_state_update(const amore::state_msg::ConstPtr& msg)
{
	if (system_initialized)
	{
		PP_state = msg->state.data;
	}
} // END OF pp_state_update()

// THIS FUNCTION: Updates the state of "propulsion_system" given by "mission_control"
// ACCEPTS: propulsion_system state_msg from "ps_state"
// RETURNS: (VOID)
// =============================================================================
void ps_state_update(const amore::state_msg::ConstPtr& msg) 
{
	if (system_initialized)
	{
		PS_state = msg->state.data;
	}
} // END OF ps_state_update()

// THIS FUNCTION: Updates the state of "perception_array" given by "mission_control"
// ACCEPTS: perception_array state_msg from "pa_state"
// RETURNS: (VOID) Updates global variable
// =============================================================================
void pa_state_update(const amore::state_msg::ConstPtr& msg) 
{
	if (system_initialized)
	{
		PA_state = msg->state.data;
	}
} // END OF pa_state_update()

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
//////////////////////////////////////////////////////////////		STATE UPDATERS END		///////////////////////////////////////////////////////////////////

// THIS FUNCTION: Updates the current NED USV pose converted through the navigation_array
// ACCEPTS: Current NED USV pose and velocities from "nav_ned"
// RETURNS: (VOID)
// =============================================================================
void pose_update(const nav_msgs::Odometry::ConstPtr& odom) 
{
	if (NA_state == 1) // if navigation_array is in standard USV NED pose converter mode 
	{
		// Update NED USV pose 
		x_usv_NED = odom->pose.pose.position.x;
		y_usv_NED = odom->pose.pose.position.y;
		psi_NED = odom->pose.pose.orientation.z;
	}
} // END OF pose_update()

// THIS FUNCTION: Updates the goal NED waypoints converted through the navigation_array
// ACCEPTS: Goal NED waypoints from "waypoints_NED"
// RETURNS: (VOID)
// =============================================================================
void goal_NED_waypoints_update(const amore::NED_waypoints::ConstPtr& goal) 
{
	if ((system_initialized) && ((PP_state == 1) || (PP_state == 2)))		// if the system is initialized and station-keeping or wayfinding task 
	{
		if (!NA_goal_recieved)				// if the NED goal waypoints have been published but not recieved yet
		{
			goal_poses = goal->quantity;
			for (int i = 0; i < goal_poses; i++)
			{
				x_goal[i] = goal->points[i].x;
				y_goal[i] = goal->points[i].y;
				psi_goal[i] = goal->points[i].z;
				ROS_INFO("Point x: %4.2f		Point y: %4.2f --PP", x_goal[i], y_goal[i]);
			}
			NA_goal_recieved = true;
			loop_goal_recieved = loop_count;
			
			// UPDATES STATUSES TO USER ///////////////////////////////////////////////
			ROS_INFO("GOAL POSES ACQUIRED BY PLANNER. --MC");
			ROS_INFO("Quantity of goal poses: %i --MC", goal_poses);
		} // if (!NA_goal_recieved)
	}
} // END OF goal_NED_waypoints_update()

// THIS FUNCTION: Updates the NED acoustic source point
// ACCEPTS: NED acoustic source point from "acoustic_source_location"
// RETURNS: (VOID)
// =============================================================================
void goal_NED_acoustic_update(const amore::NED_acoustic::ConstPtr& sourceloc) 
{
	if ((system_initialized) && (PP_state == 5))	// if the system is initialized and gymkhana task (task 5)
	{
		if (!NA_goal_recieved)				// if the NED goal waypoints have been published but not recieved yet
		{
			goal_poses = sourceloc->quantity;
			for (int i = 0; i < goal_poses; i++)
			{
				x_goal[i] = sourceloc->points[i].x;
				y_goal[i] = sourceloc->points[i].y;
				psi_goal[i] = sourceloc->points[i].z;
				//ROS_INFO("Point x: %4.2f		Point y: %4.2f --PP", x_goal[i], y_goal[i]);
			}
			NA_goal_recieved = true;
			loop_goal_recieved = loop_count;
			
			// UPDATES STATUSES TO USER ///////////////////////////////////////////////
			ROS_INFO("GOAL POSES ACQUIRED BY PLANNER. --MC");
			ROS_INFO("Quantity of goal poses: %i --MC", goal_poses);
		} // if (!NA_goal_recieved)
	}
} // END OF goal_NED_acoustic_update() 

// THIS FUNCTION: Updates the local NED positions of the animals converted through the navigation_array
// ACCEPTS: Goal NED animals from "NED_animals"
// RETURNS: (VOID)
// =============================================================================
void goal_NED_animals_update(const amore::NED_objects::ConstPtr& object)
{
	if ((system_initialized) && (PP_state == 4))		// if the system is initialized and wildlife task
	{
		if (!NA_goal_recieved)				// if the NED goal waypoints have been published but not recieved yet
		{
			ROS_INFO("Subscribing to NED animals positions");
			for (int i = 0; i < object->quantity; i++)
			{
			   Animal[i] = object->objects[i].header.frame_id;						//Getting array of animal names
			   if (Animal[i] == "crocodile")
			   {
				   x_c_NED = object->objects[i].point.x;
				   y_c_NED = object->objects[i].point.y;
			   }
			   if (Animal[i] == "turtle")
			   {
				   x_t_NED = object->objects[i].point.x;
				   y_t_NED = object->objects[i].point.y;
			   }
			   if (Animal[i] == "platypus")
			   {
				   x_p_NED = object->objects[i].point.x;
				   y_p_NED = object->objects[i].point.y;
			   }
			   x_animals_NED[i] = object->objects[i].point.x;						//Getting x position of animals 
			   y_animals_NED[i] = object->objects[i].point.y;						//Getting y position of animals
			   ROS_INFO("Animal: %s		x: %4.2f			y: %4.2f", Animal[i].c_str(), x_animals_NED[i], y_animals_NED[i]);
			}
			NA_goal_recieved = true;
			loop_goal_recieved = loop_count;
		}
	}
} // end of goal_NED_animals_update()

void animal_distances_calculate()
{
	dc_USV = sqrt(pow(x_usv_NED - x_c_NED, 2.0)+pow(y_usv_NED - y_c_NED, 2.0));							//Distance from USV to crocodile
	dp_USV = sqrt(pow(x_usv_NED - x_p_NED, 2.0)+pow(y_usv_NED - y_p_NED, 2.0));							//Distance from USV to platypus
	dt_USV = sqrt(pow(x_usv_NED - x_t_NED, 2.0)+pow(y_usv_NED - y_t_NED, 2.0));							//Distance from USV to turtle
	dt_c = sqrt(pow(x_t_NED - x_c_NED, 2.0)+pow(y_t_NED - y_c_NED, 2.0));										//Distance from turtle to crocodile
	dp_c  = sqrt(pow(x_p_NED - x_c_NED, 2.0)+pow(y_p_NED - y_c_NED, 2.0));									//Distance from platypus to crocodile
	ROS_INFO("dp_USV: %4.2f			dt_USV: %4.2f", dp_USV, dt_USV);
} // end of animal_distances_calculate()

// THIS FUNCTION: Generates the updated array of poses to accomplish the task
// ACCEPTS: (VOID)
// RETURNS: (VOID)
// =============================================================================
void update_animal_path()
{
	if ((!calculations_done) && (NA_state == 1))
	{
		r = 5;
		//Making the circle ccw
		x_turt_g[0] = x_t_NED - r;
		y_turt_g[0] = y_t_NED;
		psi_turt_g[0] = 90.0 * (PI/180);
		x_turt_g[1] = x_t_NED - sqrt(2)/2*r;
		y_turt_g[1] = y_t_NED + sqrt(2)/2*r;
		psi_turt_g[1] = 45.0 * (PI/180);
		x_turt_g[2] = x_t_NED;
		y_turt_g[2] = y_t_NED + r;
		psi_turt_g[2] = 0.0 * (PI/180);
		x_turt_g[3] = x_t_NED + sqrt(2)/2*r;
		y_turt_g[3] = y_t_NED + sqrt(2)/2*r;
		psi_turt_g[3] = -45.0 * (PI/180);
		x_turt_g[4] = x_t_NED + r;
		y_turt_g[4] = y_t_NED;
		psi_turt_g[4] = -90.0 * (PI/180);
		x_turt_g[5] = x_t_NED + sqrt(2)/2*r;
		y_turt_g[5] = y_t_NED - sqrt(2)/2*r;
		psi_turt_g[5] = -135.0 * (PI/180);
		x_turt_g[6] = x_t_NED;
		y_turt_g[6] = y_t_NED - r;
		psi_turt_g[6] = 180.0 * (PI/180);
		x_turt_g[7] = x_t_NED - sqrt(2)/2*r;
		y_turt_g[7] = y_t_NED - sqrt(2)/2*r;
		psi_turt_g[7] = 135.0 * (PI/180);
		x_turt_g[8] = x_t_NED - r;
		y_turt_g[8] = y_t_NED;
		psi_turt_g[8] = 90.0 * (PI/180);

		//Making the circle cw
		x_plat_g[0] = x_p_NED + r;
		y_plat_g[0] = y_p_NED;
		psi_plat_g[0] = 90.0 * (PI/180);
		x_plat_g[1] = x_p_NED + sqrt(2)/2*r;
		y_plat_g[1] = y_p_NED + sqrt(2)/2*r;
		psi_plat_g[1] = 135.0 * (PI/180);
		x_plat_g[2] = x_p_NED;
		y_plat_g[2] = y_p_NED + r;
		psi_plat_g[2] = 180.0 * (PI/180);
		x_plat_g[3] = x_p_NED - sqrt(2)/2*r;
		y_plat_g[3] = y_p_NED + sqrt(2)/2*r;
		psi_plat_g[3] = -135.0 * (PI/180);
		x_plat_g[4] = x_p_NED - r;
		y_plat_g[4] = y_p_NED;
		psi_plat_g[4] = -90.0 * (PI/180);
		x_plat_g[5] = x_p_NED - sqrt(2)/2*r;
		y_plat_g[5] = y_p_NED - sqrt(2)/2*r;
		psi_plat_g[5] = -45.0 * (PI/180);
		x_plat_g[6] = x_p_NED;
		y_plat_g[6] = y_p_NED - r;
		psi_plat_g[6] = 0.0 * (PI/180);
		x_plat_g[7] = x_p_NED + sqrt(2)/2*r;
		y_plat_g[7] = y_p_NED - sqrt(2)/2*r;
		psi_plat_g[7] = 45.0 * (PI/180);
		x_plat_g[8] = x_p_NED + r;
		y_plat_g[8] = y_p_NED;
		psi_plat_g[8] = 90.0 * (PI/180);
		
		animal_distances_calculate();			// updates distances from USV to each animal 
		// if (dt_USV <= dp_USV)
		// {
			// for (int i=0; i<9; i++)
			// {
				// x_goal[i] = x_turt_g[i];
				// y_goal[i] = y_turt_g[i];
				// psi_goal[i] = psi_turt_g[i];
			// }
			// ROS_INFO("x_t_NED: %4.2f			y_t_NED: %4.2f			x_p_NED: %4.2f			y_p_NED: %4.2f", x_t_NED, y_t_NED, x_p_NED, y_p_NED);
			
			// for (int i=9; i<18; i++)
			// {
				// x_goal[i] = x_plat_g[i-9];
				// y_goal[i] = y_plat_g[i-9];
				// psi_goal[i] = psi_plat_g[i-9];
			// }
		// }
		//else
		//{
			for (int i=0; i<9; i++)
			{
				x_goal[i] = x_plat_g[i];
				y_goal[i] = y_plat_g[i];
				psi_goal[i] = psi_plat_g[i];
			}
			for (int i=9; i<18; i++)
			{
				x_goal[i] = x_turt_g[i-9];
				y_goal[i] = y_turt_g[i-9];
				psi_goal[i] = psi_turt_g[i-9];
			}
		//}
		calculations_done = true;
		goal_poses = 18;
	}
	
} // end of update_animal_path()

// THIS FUNCTION: Fills out current_goal_pose_msg and publishes to "current_goal_pose" for the propulsion_system
// ACCEPTS: Nothing. Uses global variable pose arrays
// RETURNS: (VOID)
// =============================================================================
void current_goal_pose_publish()
{
	// PUBLISH THE CURRENT GOAL POSE
	current_goal_pose_msg.header.seq = 0;
	current_goal_pose_msg.header.seq +=1;									// sequence number
	current_goal_pose_msg.header.stamp = current_time;				// sets stamp to current time
	current_goal_pose_msg.header.frame_id = "mission_control";	// header frame
	current_goal_pose_msg.position.x = x_goal[point];						// sets x-location
	current_goal_pose_msg.position.y = y_goal[point];						// sets y-location
	current_goal_pose_msg.position.z = 0.0;										// sets z-location
	current_goal_pose_msg.psi.data = psi_goal[point];						// sets psi
	
	//ROS_INFO("Publishing point --PP");
	//ROS_INFO("Point x: %4.2f		Point y: %4.2f --PP", x_goal[point], y_goal[point]);

	current_goal_pose_pub.publish(current_goal_pose_msg);			// publish goal usv pose to "current_goal_pose"
	goal_pose_publish_status.data = true;
} // END OF current_goal_pose_publish()

/* // START OF FUNCTIONS FOR VISION PATH PLANNER 
// update left and right buoy centroid location (x,y) in global frame
void NED_buoys_update (const geometry_msgs::PoseArray::ConstPtr& buoys)
{
	if ((system_initialized) && (PP_state == 5) && (!NED_buoys_recieved))
	{
		CL_x = buoys->poses[0].position.x;
		CL_y = buoys->poses[0].position.y;
		CR_x = buoys->poses[1].position.x;
		CR_y = buoys->poses[1].position.y;
		NED_buoys_recieved = true;
	}
}

void calculate_buoy_waypoints()
{
	if ((PP_state == 5) && (!calculations_done) && (NED_buoys_recieved))			// if the intermediate approach, mid-, and exit points have been calculated
	{
		// hardcode values
		//CL_x = 37.85;
		//CL_y = 2.2;
		//CR_x = 37.85;
		//CR_y = 15.75;
		//x_usv_NED = 24.24;
		//y_usv_NED = 11.88;
		//psi_NED = 0.0;
		ROS_DEBUG("________vvvvvvvvvvv  {PP}  vvvvvvvvvvv___________________\n");
		ROS_DEBUG("~~~USV POSE~~~");
		ROS_DEBUG("Psi_NED: %f", psi_NED);
		ROS_DEBUG("x_NED: %f", x_usv_NED);
		ROS_DEBUG("y_NED: %f\n", y_usv_NED);
		ROS_DEBUG("~~~Buoy locations wrt USV~~~");
		ROS_DEBUG("LB_x: %f", CL_x);
		ROS_DEBUG("LB_y: %f", CL_y);
		ROS_DEBUG("RB_x: %f", CR_x);
		ROS_DEBUG("RB_y: %f\n", CR_y);
		CL_x_NED = cos(psi_NED)*CL_x - sin(psi_NED)*CL_y + x_usv_NED;
		CL_y_NED = sin(psi_NED)*CL_x + cos(psi_NED)*CL_y + y_usv_NED;
		CR_x_NED = cos(psi_NED)*CR_x - sin(psi_NED)*CR_y + x_usv_NED;
		CR_y_NED = sin(psi_NED)*CR_x + cos(psi_NED)*CR_y + y_usv_NED;
		ROS_DEBUG("~~~Buoy locations in NED frame~~~");
		ROS_DEBUG("LB_x: %f", CL_x_NED);
		ROS_DEBUG("LB_y: %f", CL_y_NED);
		ROS_DEBUG("RB_x: %f", CR_x_NED);
		ROS_DEBUG("RB_y: %f\n", CR_y_NED);

		x_goal[1] = (CL_x_NED+CR_x_NED)/2.0; // x-location of midpoint
		y_goal[1] = (CL_y_NED+CR_y_NED)/2.0; // y-location of midpoint

		d_L = sqrt(pow((CL_x_NED-x_usv_NED),2.0)+pow((CL_y_NED-y_usv_NED),2.0));    // distance from USV to left buoy
		d_M = sqrt(pow((M_x-x_usv_NED),2.0)+pow((M_y-y_usv_NED),2.0));      // distance from USV to midpoint
		d_R = sqrt(pow((CR_x_NED-x_usv_NED),2.0)+pow((CR_y_NED-y_usv_NED),2.0));  // distance from USV to right buoy
		d_I = (d_L+d_M+d_R)/3.0; // distance from midpoint to approach point

		// intermediate approach point doesn't need to be more than 8 meters back from midpoint
		if (d_I > 4.0)
		{
		  d_I = 4.0;
		}

		d_LM = 0.5*sqrt(pow((CR_x_NED-CL_x_NED),2.0)+pow((CR_y_NED-CL_y_NED),2.0));		// half distance between left and right buoys
		a_L = sqrt(pow(d_LM,2.0)+pow(d_I,2.0));																						// distance between left buoy and the approach point
		theta = atan(d_I/d_LM);																													// angle created by d_I and d_LM

		x_I_CL = a_L*cos(theta);																												// x-coord. of intermediate point wrt left buoy 
		y_I_CL = a_L*sin(theta);																												// y-coord. of intermediate point wrt left buoy

		x_E_CL = a_L*cos(theta);																												// x-coord. of exit point wrt left buoy 
		y_E_CL = -a_L*sin(theta);																												// y-coord. of exit point wrt left buoy

		// calculate intermediate position wrt global
		s_M = (CR_y_NED-CL_y_NED)/(CR_x_NED-CL_x_NED);														// slope of of line from CL to CR 
		alpha = atan2((CR_y_NED-CL_y_NED),(CR_x_NED-CL_x_NED));											// angle of frame CL wrt global frame
		
		x_goal[0] = cos(alpha)*x_I_CL - sin(alpha)*y_I_CL + CL_x_NED;												// x-coord. of intermediate point wrt global
		y_goal[0] = sin(alpha)*x_I_CL + cos(alpha)*y_I_CL + CL_y_NED;												// y-coord. of intermediate point wrt global

		x_goal[2] = cos(alpha)*x_E_CL - sin(alpha)*y_E_CL + CL_x_NED;											// x-coord. of exit point wrt global
		y_goal[2] = sin(alpha)*x_E_CL + cos(alpha)*y_E_CL + CL_y_NED;											// y-coord. of exit point wrt global

		// display calculated goal points to reach
		ROS_DEBUG("~~~Desired points~~~");
		ROS_DEBUG("I_x: %f", x_goal[0]);
		ROS_DEBUG("I_y: %f", y_goal[0]);
		ROS_DEBUG("M_x: %f", x_goal[1]);
		ROS_DEBUG("M_y: %f", y_goal[1]);
		ROS_DEBUG("E_x: %f", x_goal[2]);
		ROS_DEBUG("E_y: %f\n", y_goal[2]);
		ROS_DEBUG("________^^^^^^^^^^^  {PP}  ^^^^^^^^^^^___________________|\n");
		
		point = 0;
		goal_poses = 3;
		calculations_done = true;
		loop_goal_recieved = loop_count;
	} // if (!calculations_done)
} // END OF calculate_buoy_waypoints()
// END OF FUNCTIONS FOR VISIONS PATH PLANNER  */
//............................................................End of Functions............................................................

int main(int argc, char **argv)
{
	// names the program for visual purposes
	ros::init(argc, argv, "path_planner");

	ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

	// NodeHandles
	ros::NodeHandle nh1, nh2, nh3, nh4, nh5, nh6, nh7, nh8, nh9, nh10, nh11,nh12;

	// Subscribers
	ros::Subscriber na_state_sub = nh1.subscribe("na_state", 1, na_state_update);
	ros::Subscriber pp_state_sub = nh2.subscribe("pp_state", 1, pp_state_update);
	ros::Subscriber ps_state_sub = nh3.subscribe("ps_state", 1, ps_state_update);
	ros::Subscriber pa_state_sub = nh4.subscribe("pa_state", 1, pa_state_update);
	ros::Subscriber a_state_sub = nh5.subscribe("a_state", 1, a_state_update);
	ros::Subscriber nav_NED_sub = nh6.subscribe("nav_ned", 1, pose_update);														// Obtains the USV pose in global NED from mission_control
	ros::Subscriber waypoints_NED_sub = nh7.subscribe("waypoints_ned", 1, goal_NED_waypoints_update);				// goal waypoints converted to NED
	ros::Subscriber acousticsource_NED_sub = nh8.subscribe("acoustic_source_location", 1, goal_NED_acoustic_update);		// acoustic pinger converted to NED
	ros::Subscriber ned_animals_sub = nh9.subscribe("ned_animals", 1, goal_NED_animals_update);				// goal animal locations converted to NED
	
	// Publishers
	pp_initialization_state_pub = nh10.advertise<std_msgs::Bool>("pp_initialization_state", 1);						// publisher for state of initialization
	current_goal_pose_pub = nh11.advertise<amore::usv_pose_msg>("current_goal_pose", 1);						// current goal for low level controller (propulsion_system)
	goal_pose_publish_state_pub = nh12.advertise<std_msgs::Bool>("goal_pose_publish_state", 1);			// "goal_pose_publish_state" publisher for whether NED converted waypoints have been published

	// Timers ///////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Initialize simulation time
	ros::Time::init();

	// Initialize global variables
	goal_pose_publish_status.data = false;
	pp_initialization_status.data = false;
	current_time = ros::Time::now();							// sets current time to the time it is now
	last_time = current_time;										// sets last time to the current_time

	//sets the frequency for which the program sleeps at. 10=1/10 second
	ros::Rate loop_rate(50);

	// ros::ok() will stop when the user inputs Ctrl+C
	while(ros::ok())
	{
		PATH_PLANNER_inspector();															// check that entire system is initialized before starting calculations
		goal_pose_publish_state_pub.publish(goal_pose_publish_status);	// publish whether NED goal pose has been published to propulsion_system for mission_control to know when to turn propulsion_system ON
		
		//	0 = On standby
		//	1 = Station-Keeping
		//	2 = Wayfinding
		//	4 = Wildlife Encounter and Avoid
		//	5 = Task 5: Channel Navigation, Acoustic Beacon Localization and Obstacle Avoidance
		//	6 = Task 6: Scan and Dock and Deliver
		switch(PA_state)
		{
			case 0:						// On standby
			
				break;
			case 1:						// Station-Keeping
			
				break;
			case 2:						// Wayfinding
			
				break;
			case 4:						// Wildlife Encounter and Avoid
			
				break;
			case 5:						// Channel Navigation, Acoustic Beacon Localization and Obstacle Avoidance
			
				break;
			case 6:						// Scan and Dock and Deliver
			
				break;
			default:
				break;
		}

		if (PP_state == 0)
		{
			// reset all variables to be used for next run
			goal_pose_publish_status.data = false;
			NED_buoys_recieved = false;
			calculations_done = false;
			E_reached = false;
			NA_goal_recieved = false;
			e_xy_allowed = 0.4;       								// positional error tolerance threshold; NOTE: make as small as possible
			e_psi_allowed = 0.4;      								// heading error tolerance threshold; NOTE: make as small as possible
		}
		else if ((PP_state == 1) || (PP_state == 2)|| (PP_state == 5))	// TASK 1: STATION_KEEPING, TASK 2: WAYFINDING, TASK 5: GYMKHANA
		{
			if ((loop_count > (loop_goal_recieved)) && (NA_goal_recieved))
			{
				if ((NA_state == 1) && (PP_state == 2) && (PS_state == 1))		// if the navigation_array is providing NED USV state, Wayfinding planner is active, and the propulsion_system is ON
				{
					// determine error in x and y (position)
					e_x = x_goal[point] - x_usv_NED;                                       // calculate error in x position
					e_y = y_goal[point] - y_usv_NED;                                       // calculate error in y position
					e_xy = sqrt(pow(e_x,2.0)+pow(e_y,2.0));                            // calculate magnitude of positional error
					e_psi = psi_goal[point] - psi_NED;

					if ((e_xy < e_xy_allowed) && (e_psi < e_psi_allowed) && (!E_reached))
					{
						point += 1;
						ROS_INFO("Point %i reached. --MC", point);
						if (point==goal_poses)
						{
						  E_reached = true;
						  ROS_INFO("End point has been reached. --MC\n");
						}
					}

					if (E_reached)		// reset and go to points again once last point has been reached with a smaller tolerance threshold
					{
						point = 0;
						e_xy_allowed /= 2;
						e_psi_allowed /= 2;
						E_reached = false;
					}
				} // if (PP_state == 2)
				current_goal_pose_publish();
			} // if (loop_count > loop_goal_recieved)
		} // if ((PP_state == 1) || (PP_state == 2))||(PP_state == 5)
		else if (PP_state == 4)	// TASK 4: Wildlife Encounter and Avoid
		{
			if ((loop_count > (loop_goal_recieved)) && (NA_goal_recieved))
			{
				if (calculations_done)
				{
					if ((NA_state == 1) && (PS_state == 1))		// if the navigation_array is providing NED USV state, and the propulsion_system is ON
					{
						// determine error in x and y (position)
						e_x = x_goal[point] - x_usv_NED;                                       // calculate error in x position
						e_y = y_goal[point] - y_usv_NED;                                       // calculate error in y position
						e_xy = sqrt(pow(e_x,2.0)+pow(e_y,2.0));                            // calculate magnitude of positional error
						e_psi = psi_goal[point] - psi_NED;

						if ((e_xy < 1.2) && (e_psi < 0.6) && (!E_reached))
						{
							point += 1;
							ROS_INFO("Point %i of %i reached. --MC", point, goal_poses);
							if (point==goal_poses)
							{
							  E_reached = true;
							  ROS_INFO("End point has been reached. --MC\n");
							}
						}

						if (E_reached)		// reset and go to points again once last point has been reached with a smaller tolerance threshold
						{
							point = 0;
							e_xy_allowed /= 2;
							e_psi_allowed /= 2;
							E_reached = false;
						}
					} // if ((NA_state == 1) && (PS_state == 1))
					current_goal_pose_publish();
				}
				else
				{
					animal_distances_calculate();			// updates distances from USV to each animal 
					update_animal_path();																// this function will generate the updated array of poses to accomplish task
				}
			} // if (loop_count > loop_goal_recieved)
		} // if ((PP_state == 1) || (PP_state == 2))

		ros::spinOnce();										// update subscribers
		loop_rate.sleep();									// sleep for set loop_rate
		last_time = current_time;						// update last_time
		//loop_count += 1;									// increment loop counter
	} // while(ros::ok())

	ros::spinOnce();
	loop_rate.sleep();

	return 0;
}
