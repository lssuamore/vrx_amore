//  Filename:											mission_control.cpp
//  Creation Date:									03/25/2022
//  Last Revision Date:							04/04/2022
//  Author(s) [email]:								Bradley Hacker [bhacker@lssu.edu]
//  Revisor(s) [email] {Revision Date}:	Bradley Hacker [bhacker@lssu.edu] {03/28/2022}
//  Organization/Institution:					Lake Superior State University - Team AMORE
// 
// ...............................About mission_control.cpp......................................
//  This code acts as the autonomous state machine of the WAM-V USV.
//  It will subscribe to the vrx/task/info to control the state of the system.
//  This code will subscribe to goal poses given from the gps_imu node.
//  Dependent on the current task state and system state, mission_control
//  will publish whether or not the low level controllers should be on, as well 
//  as the goal of the low level controllers.
//
//  Inputs and Outputs of the mission_control.cpp file
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
#include "nav_msgs/Odometry.h"
#include "amore/state_msg.h"												// message type used to communicate state for rudimentary codes
#include "std_msgs/Bool.h"
#include "amore/usv_pose_msg.h"										// message that holds usv position as a geometry_msgs/Point and heading in radians as a Float64
#include "vrx_gazebo/Task.h"												// message published by VRX detailing current task and state
#include "amore/NED_waypoints.h"										// message that holds array of converted WF goal waypoints w/ headings and number of waypoints
#include "geometry_msgs/PoseArray.h"			// message type used to get buoy locations from navigation_array
//...........................................End of Included Libraries and Message Types....................................


//.................................................................Constants....................................................................
#define PI 3.14159265
//............................................................End of Constants.............................................................


//..............................................................Global Variables............................................................
int loop_count = 0;                                    				// loop counter, first 10 loops used to intitialize subscribers
int point = 0;                     		    								// number of points on trajectory reached 
int goal_poses;              											// total number of poses to reach 
int loop_goal_recieved;         									// this is kept in order to ensure planner doesn't start controller until the goal is published

float x_goal[100], y_goal[100], psi_goal[100];		// arrays to hold the NED goal poses
float x_usv_NED, y_usv_NED, psi_NED; 				// vehicle position and heading (pose) in NED

float e_x, e_y, e_xy, e_psi;										// current errors between goal pose and usv pose

bool NED_waypoints_published = false;				// NED_waypoints_published = false means the NED poses have not yet been calculated and published
bool NED_waypoints_recieved = false;					// NED_waypoints_recieved = false means goal position has not been acquired from "waypoints_NED"
bool E_reached = false;        									// E_reached = false means the last point has not been reached
//float e_xy_prev, e_psi_prev;									// previous errors

float e_xy_allowed = 0.4;       									// positional error tolerance threshold; NOTE: make as small as possible
float e_psi_allowed = 0.4;      									// heading error tolerance threshold; NOTE: make as small as possible

// STATES CONCERNED WITH "mission_control"
int MC_state = 0;							// 0 = On Standby; 1 = SK Planner; 2 = WF Planner; 5 = Task 5 Planner; 4569 = HARD RESET (OR OTHER USE)
// STATES CONCERNED WITH "navigation_array"
int NA_state = 0;							// 0 = On Standby; 1 = USV NED Pose Conversion; 2 = SK NED Goal Pose Conversion; 3 = WF NED Goal Pose Conversion; 4569 = HARD RESET (OR OTHER USE)
// STATES CONCERNED WITH "propulsion_system"
int PS_state = 0;      						// 0 = On Standby, 1 = LL controller ON
// STATES CONCERNED WITH "perception_array"
int PA_state = 0;      						// 0 = On Standby, 1 = General State, 2 = Task 3: Perception

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

bool calculations_done = false; // if calculations_done is true, this means the intermediate approach, mid-, and exit points have been calculated
bool left_right_buoy_array_recieved = false;			// false means goal the left and right buoy data has not yet been acquired

// THE FOLLOWING FOUR BOOLS ARE USED TO DETERMINE IF THE SYSTEM HAS BEEN INITIALIZED
bool navigation_array_initialized = false;							// navigation_array_initialized = false means navigation_array is not initialized
bool propulsion_system_initialized = false;						// propulsion_system_initialized = false means propulsion_system is not initialized
bool perception_array_initialized = false;							// perception_array_initialized = false means the perception_array is not initialized
bool mission_control_initialized = false;							// mission_control_initialized = false means the mission_control is not initialized

amore::usv_pose_msg current_goal_pose_msg;			// "current_goal_pose" message
ros::Publisher current_goal_pose_pub;							// "current_goal_pose" publisher

amore::state_msg propulsion_system_state;					// "ps_state" message
ros::Publisher ps_state_pub;												// "ps_state" publisher

amore::state_msg navigation_array_state;						// "na_state" message
ros::Publisher na_state_pub;												// "na_state" publisher

amore::state_msg perception_array_state;						// "pa_state" message
ros::Publisher pa_state_pub;												// "pa_state" publisher

ros::Time current_time, last_time;									// creates time variables
//..............................................................End of Global Variables..........................................................


//..................................................................Functions...........................................................................

// START WITH FUNCTIONS FOR VISION PATH PLANNER 
// update left and right buoy centroid location (x,y) in global frame
void left_right_buoy_loc_update (const geometry_msgs::PoseArray::ConstPtr& buoys)
{
	if ((mission_control_initialized) && (MC_state == 5) && (!left_right_buoy_array_recieved))
	{
		CL_x = buoys->poses[0].position.x;
		CL_y = buoys->poses[0].position.y;
		CR_x = buoys->poses[1].position.x;
		CR_y = buoys->poses[1].position.y;
		left_right_buoy_array_recieved = true;
	}
}

void calculate_buoy_waypoints()
{
	if ((MC_state == 5) && (!calculations_done) && (left_right_buoy_array_recieved))			// if the intermediate approach, mid-, and exit points have been calculated
	{
	/*// hardcode values
	CL_x = 37.85;
	CL_y = 2.2;
	CR_x = 37.85;
	CR_y = 15.75;
	x_usv_NED = 24.24;
	y_usv_NED = 11.88;
	psi_NED = 0.0;*/
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
	/* M_x = (CL_x_NED+CR_x_NED)/2.0; // x-location of midpoint
	M_y = (CL_y_NED+CR_y_NED)/2.0; // y-location of midpoint */

	/* ROS_DEBUG("x_usv_NED: %f\n", x_usv_NED);
	ROS_DEBUG("y_usv_NED: %f\n", y_usv_NED); */
	d_L = sqrt(pow((CL_x_NED-x_usv_NED),2.0)+pow((CL_y_NED-y_usv_NED),2.0));    // distance from USV to left buoy
	d_M = sqrt(pow((M_x-x_usv_NED),2.0)+pow((M_y-y_usv_NED),2.0));      // distance from USV to midpoint
	d_R = sqrt(pow((CR_x_NED-x_usv_NED),2.0)+pow((CR_y_NED-y_usv_NED),2.0));  // distance from USV to right buoy
	d_I = (d_L+d_M+d_R)/3.0; // distance from midpoint to approach point

	// intermediate approach point doesn't need to be more than 8 meters back from midpoint
	if (d_I > 4.0)
	{
	  d_I = 4.0;
	}
	/* ROS_DEBUG("d_L: %f\n", d_L);
	ROS_DEBUG("d_M: %f\n", d_M);
	ROS_DEBUG("d_R: %f\n", d_R);
	ROS_DEBUG("d_I: %f\n", d_I); */

	d_LM = 0.5*sqrt(pow((CR_x_NED-CL_x_NED),2.0)+pow((CR_y_NED-CL_y_NED),2.0));                // half distance between left and right buoys
	a_L = sqrt(pow(d_LM,2.0)+pow(d_I,2.0));                                                       // distance between left buoy and the approach point
	theta = atan(d_I/d_LM);                                                                                    // angle created by d_I and d_LM
	/* ROS_DEBUG("d_LM: %f\n", d_LM);
	ROS_DEBUG("a_L: %f\n", a_L);
	ROS_DEBUG("theta: %f\n", theta); */

	x_I_CL = a_L*cos(theta);                                                                                 // x-coord. of intermediate point wrt left buoy 
	y_I_CL = a_L*sin(theta);                                                                                  // y-coord. of intermediate point wrt left buoy
	/* ROS_DEBUG("x_I_CL: %f\n", x_I_CL);
	ROS_DEBUG("y_I_CL: %f\n", y_I_CL); */

	x_E_CL = a_L*cos(theta);                                                                               // x-coord. of exit point wrt left buoy 
	y_E_CL = -a_L*sin(theta);                                                                               // y-coord. of exit point wrt left buoy
	/* ROS_DEBUG("x_E_CL: %f\n", x_E_CL);
	ROS_DEBUG("y_E_CL: %f\n", y_E_CL); */

	// calculate intermediate position wrt global
	s_M = (CR_y_NED-CL_y_NED)/(CR_x_NED-CL_x_NED);                                                                // slope of of line from CL to CR 
	alpha = atan2((CR_y_NED-CL_y_NED),(CR_x_NED-CL_x_NED));                                                                                           // angle of frame CL wrt global frame
	/* ROS_DEBUG("s_M: %f\n", s_M);
	ROS_DEBUG("alpha: %f\n", alpha); */
	
	x_goal[0] = cos(alpha)*x_I_CL - sin(alpha)*y_I_CL + CL_x_NED;                                            // x-coord. of intermediate point wrt global
	y_goal[0] = sin(alpha)*x_I_CL + cos(alpha)*y_I_CL + CL_y_NED;                                           // y-coord. of intermediate point wrt global
	/* I_x = cos(alpha)*x_I_CL - sin(alpha)*y_I_CL + CL_x_NED;                                            // x-coord. of intermediate point wrt global 
	I_y = sin(alpha)*x_I_CL + cos(alpha)*y_I_CL + CL_y_NED;                                           // y-coord. of intermediate point wrt global */

	x_goal[2] = cos(alpha)*x_E_CL - sin(alpha)*y_E_CL + CL_x_NED;                                            // x-coord. of exit point wrt global
	y_goal[2] = sin(alpha)*x_E_CL + cos(alpha)*y_E_CL + CL_y_NED;                                           // y-coord. of exit point wrt global
	/* E_x = cos(alpha)*x_E_CL - sin(alpha)*y_E_CL + CL_x_NED;                                            // x-coord. of intermediate point wrt global 
	E_y = sin(alpha)*x_E_CL + cos(alpha)*y_E_CL + CL_y_NED;                                           // y-coord. of intermediate point wrt global */

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

// NOTE: To simplify, use just message variables !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
void state_update()
{
	if (mission_control_initialized)
	{
		NA_state = 1;							// USV NED Pose Conversion mode 
		MC_state = 5;						// Buoy navigation planner mode
		PA_state = 2;							// Perception array is turned on 
		if (calculations_done)
		{
			PS_state = 1;						// Turn on propulsion_system once the goal points have been calculated
		}
		else
		{
			PS_state = 0;
		}
		// UPDATE USER OF EACH CODES STATE
		ROS_DEBUG("----------------- CURRENT STATES --------------------");
		ROS_DEBUG("NA_state: %i --MC", NA_state);
		ROS_DEBUG("PS_state: %i --MC", PS_state);
		ROS_DEBUG("MC_state: %i --MC", MC_state);
		ROS_DEBUG("PA_state: %i --MC", PA_state);
	} // if (mission_control_initialized)
	else
	{
		// ALL CODES ON STANDBY
		NA_state = 0;		// 0 = On Standby; 1 = USV NED Pose Conversion; 2 = SK NED Goal Pose Conversion; 3 = WF NED Goal Pose Conversion; 4569 = HARD RESET (OR OTHER USE)
		PS_state = 0;		// 0 = On Standby, 1 = LL controller ON
		MC_state = 0;	// 0 = On Standby; 1 = SK Planner; 2 = WF Planner; 4569 = HARD RESET (OR OTHER USE)
		PA_state = 0;      // 0 = On Standby, 1 = General State, 2 = Task 3: Perception
	}
	// PUBLISH EACH SUBSYTEMS STATE
	// SEND STATE TO PROPULSION_SYSTEM
	propulsion_system_state.header.seq +=1;										// sequence number
	propulsion_system_state.header.stamp = current_time;					// set stamp to current time
	propulsion_system_state.header.frame_id = "mission_control";			// header frame
	propulsion_system_state.state.data = PS_state;								// set propulsion system state; 0 = OFF; 1 = ON
	ps_state_pub.publish(propulsion_system_state);								// publish propulsion_system_state to "ps_state"

	// SEND STATE TO NAVIGATION_ARRAY
	navigation_array_state.header.seq +=1;											// sequence number
	navigation_array_state.header.stamp = current_time;						// set stamp to current time
	navigation_array_state.header.frame_id = "mission_control";				// header frame
	navigation_array_state.state.data = NA_state;									// set navigation_array_state; 0 = On Standby; 1 = USV NED Pose Conversion; 2 = SK NED Goal Pose Conversion; 3 = WF NED Goal Pose Conversion; 4569 = HARD RESET (OR OTHER USE)
	na_state_pub.publish(navigation_array_state);									// publish navigation_array_state to "na_state"

	// SEND STATE TO PERCEPTION_ARRAY
	perception_array_state.header.seq +=1;											// sequence number
	perception_array_state.header.stamp = current_time;						// set stamp to current time
	perception_array_state.header.frame_id = "mission_control";			// header frame
	perception_array_state.state.data = PA_state;										// set perception_array_state; 0 = On Standby, 1 = General State, 2 = Task 3: Perception
	pa_state_pub.publish(perception_array_state);									// publish perception_array_state to "pa_state"
} // END OF state_update()
// END OF FUNCTIONS FOR VISIONS PATH PLANNER 

// SYSTEM INITIALIZATION CHECK FUNCTIONS /////////////////////////////////////////////////////////////////////////////////
// THIS FUNCTION: Subscribes to the navigation_array to check initialization status
// ACCEPTS: Initialization status from "na_initialization_state"
// RETURNS: (VOID)
// =============================================================================
void NAVIGATION_ARRAY_inspector(const std_msgs::Bool status)
{
	if (status.data)
	{
		navigation_array_initialized = true;
	}
	else
	{
		navigation_array_initialized = false;
	}	
} // END OF NAVIGATION_ARRAY_inspector()

// THIS FUNCTION: Subscribes to the propulsion_system to check initialization status
// ACCEPTS: Initialization status from "ps_initialization_state"
// RETURNS: (VOID)
// =============================================================================
void PROPULSION_SYSTEM_inspector(const std_msgs::Bool status)
{
	if (status.data)
	{
		propulsion_system_initialized = true;
	}
	else
	{
		propulsion_system_initialized = false;
	}	
} // END OF PROPULSION_SYSTEM_inspector()

// THIS FUNCTION: Subscribes to the perception_array to check initialization status
// ACCEPTS: Initialization status from "pa_initialization_state"
// RETURNS: (VOID)
// =============================================================================
void PERCEPTION_ARRAY_inspector(const std_msgs::Bool status)
{
	if (status.data)
	{
		perception_array_initialized = true;
	}
	else
	{
		perception_array_initialized = false;
	}	
} // END OF PERCEPTION_ARRAY_inspector()

// THIS FUNCTION: Checks initialization status of entire system using global variable initialization statuses
// ACCEPTS: (VOID) 
// RETURNS: (VOID) 
// =============================================================================
void MISSION_CONTROL_inspector()
{
	if ((loop_count > 10) &&  (navigation_array_initialized) && (propulsion_system_initialized) && (perception_array_initialized))
	{
		mission_control_initialized = true;
		//ROS_INFO("mission_control_initialized");
	}
	else
	{
		mission_control_initialized = false;
		ROS_INFO("!mission_control_initialized --MC");
	}
	// UPDATE USER OF INITIALIZATION STATUSES
	/* if (navigation_array_initialized)
	{
		ROS_INFO("navigation_array_initialized");
	}
	if (propulsion_system_initialized)
	{
		ROS_INFO("propulsion_system_initialized");
	}
	if (perception_array_initialized)
	{
		ROS_INFO("perception_array_initialized");
	} */
} // END OF MISSION_CONTROL_inspector()
// END OF SYSTEM INITIALIZATION CHECK FUNCTIONS ////////////////////////////////////////////////////////////////////////////////////////////////////////////

// THIS FUNCTION: Updates when NED waypoints have been converted and published to "waypoints_ned" to know when to subscribe
// ACCEPTS: Goal publish status from "goal_publish_state"
// RETURNS: (VOID)
// =============================================================================
void goal_publish_state_update(const std_msgs::Bool status)
{
	if (status.data)
	{
		NED_waypoints_published = true;
		//ROS_INFO("WF POINT CONVERTER FINISHED");
	}
	else
	{
		NED_waypoints_published = false;
		//ROS_INFO("WF POINT CONVERTER NOT FINISHED");
	} // if (status.data)
} // END OF goal_publish_state_update()

// THIS FUNCTION: Updates the goal NED waypoints converted through the navigation_array
// ACCEPTS: Goal NED waypoints from "waypoints_NED"
// RETURNS: (VOID)
// =============================================================================
void goal_NED_waypoints_update(const amore::NED_waypoints::ConstPtr& goal) 
{
	if ((mission_control_initialized) && (MC_state == 0))							// if the system is initialized and task 2: wayfinding
	{
		if ((NED_waypoints_published) && (!NED_waypoints_recieved))	// if the NED goal waypoints have been published but not recieved yet
		{
			goal_poses = goal->quantity;
			for (int i = 0; i < goal_poses; i++)
			{
				x_goal[i] = goal->points[i].x;
				y_goal[i] = goal->points[i].y;
				psi_goal[i] = goal->points[i].z;
			}
			NED_waypoints_recieved = true;
			loop_goal_recieved = loop_count;
			
			// UPDATES STATUSES TO USER ///////////////////////////////////////////////
			ROS_INFO("GOAL POSES ACQUIRED BY PLANNER. --MC");
			ROS_INFO("Quantity of goal poses: %i --MC", goal_poses);
		} // if ((NED_waypoints_published) && (!NED_waypoints_recieved))
	}
} // END OF goal_NED_waypoints_update() 

// THIS FUNCTION: Updates the current NED USV pose converted through the navigation_array
// ACCEPTS: Current NED USV pose and velocities from "nav_ned"
// RETURNS: (VOID)
// =============================================================================
void pose_update(const nav_msgs::Odometry::ConstPtr& odom) 
{
	if (NA_state == 1) // if navigation_array is in standard USV Pose Conversion mode 
	{
		// Update NED USV pose 
		x_usv_NED = odom->pose.pose.position.x;
		y_usv_NED = odom->pose.pose.position.y;
		psi_NED = odom->pose.pose.orientation.z;
		
		/* printf("the x is: %f\n", odom->pose.pose.position.x); //extracts x coor from nav_odometery
		printf("the y is: %f\n", odom->pose.pose.position.y); //extracts y coor from nav_odometery
		printf("the x orientation is: %f\n", odom->pose.pose.orientation.x); //extracts x orientation
		printf("the y orientation is: %f\n", odom->pose.pose.orientation.y); //extracts y orientation
		printf("the z orientation is: %f\n", odom->pose.pose.orientation.z); //extracts z orientation
		printf("the w orientation is: %f\n", odom->pose.pose.orientation.w); //extracts w orientation
		printf("the velocity x is: %f\n", odom-> twist.twist.linear.x);//prints velocity x
		printf("the velocity y is: %f\n", odom-> twist.twist.linear.y);//prints velocity y
		printf("the velocity z is: %f\n", odom-> twist.twist.linear.z);//prints velocity z
		printf("the angular velocity x is: %f\n", odom-> twist.twist.angular.x);//prints velocity x
		printf("the angular velocity y is: %f\n", odom-> twist.twist.angular.y);//prints velocity x
		printf("the angular velocity z is: %f\n", odom-> twist.twist.angular.z);//prints velocity x */
	} // if navigation_array is in standard USV Pose Conversion mode
} // END OF pose_update()

/* // THIS FUNCTION: Updates and publishes all subsytem states, as well as mission_control state dependent on current system statuses
// ACCEPTS: Current vrx task info from "vrx/task/info"
// RETURNS: (VOID)
// =============================================================================
void state_update(const vrx_gazebo::Task::ConstPtr& msg)						// NOTE: To simplify, use just message variables !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
{
	// MUST SET THE FOLLOWING
	NA_state = 0;				// 0 = On Standby; 1 = USV NED Pose Conversion; 2 = SK NED Goal Pose Conversion; 3 = WF NED Goal Pose Conversion; 4569 = HARD RESET (OR OTHER USE)
	PS_state = 0;				// 0 = On Standby, 1 = LL controller ON
	MC_state = 0;			// 0 = On Standby; 1 = SK Planner; 2 = WF Planner; 5 = Task 5 Planner; 4569 = HARD RESET (OR OTHER USE)
	PA_state = 0;      		// 0 = On Standby, 1 = General State, 2 = Task 3: Perception
	if (mission_control_initialized)
	{
		if (msg->name == "station_keeping")
		{
			if ((msg->state == "ready") || (msg->state == "running"))
			{
				if (NED_waypoints_recieved)
				{
					NA_state = 1;					// navigation_array is in standard USV NED Pose Conversion mode
					MC_state = 1;					// mission_control is in SK Planner mode
					if (loop_count > (loop_goal_recieved + 1000))
					{
						PS_state = 1;					// propulsion_system is turned ON
					}
					else
					{
						PS_state = 0;					// propulsion_system is turned OFF
					}
				}
				else
				{
					NA_state = 2;					// navigation_array is in SK NED Goal Pose Conversion mode
					PS_state = 0;						// propulsion_system is turned OFF
					MC_state = 0;					// mission_control is On Standby
				}
			}
			else
			{
				NA_state = 0;
				PS_state = 0;
				MC_state = 0;
				// reset task statuses as long as task is in "initial" or "finished"
				NED_waypoints_published = false;
				NED_waypoints_recieved = false;
				point = 0;
			}
		} // (msg->name == "station_keeping")
		else if (msg->name == "wayfinding")
		{
			if ((msg->state == "ready") || (msg->state == "running"))
			{
				if (NED_waypoints_recieved)
				{
					NA_state = 1;					// navigation_array is in standard USV NED Pose Conversion mode
					MC_state = 2;					// mission_control is in WF Planner mode
					if (loop_count > (loop_goal_recieved + 1000))
					{
						PS_state = 1;					// propulsion_system is turned ON
					}
					else
					{
						PS_state = 0;					// propulsion_system is turned OFF
					}
				}
				else
				{
					NA_state = 3;					// navigation_array is in WF NED Goal Pose Conversion mode
					PS_state = 0;						// propulsion_system is turned OFF
					MC_state = 0;					// mission_control is On Standby
				}
			}
			else
			{
				NA_state = 0;
				PS_state = 0;
				MC_state = 0;
				// reset task statuses as long as task is in "initial" or "finished"
				NED_waypoints_published = false;
				NED_waypoints_recieved = false;
				e_xy_allowed = 0.4;       								// positional error tolerance threshold; NOTE: make as small as possible
				e_psi_allowed = 0.4;      								// heading error tolerance threshold; NOTE: make as small as possible
				point = 0;
			}
		} // (msg->name == "wayfinding")
		else if (msg->name == "perception")										// NOT DONE YET
		{
			NA_state = 1;				// navigation_array is in standard USV NED Pose Conversion mode
			PA_state = 2;				// perception_array is in Task 3: Perception mode
			if ((msg->state == "ready") || (msg->state == "running"))
			{
				NA_state = 0;
				MC_state = 0;
			}
			else
			{
				NA_state = 0;
				MC_state = 0;
			}
		} // (msg->name == "perception")
		
		// 	INTEGRATED TASK CODES FOLLOW
		else if (msg->name == "wildlife")
		{
			if ()
			{
				NA_state = 0;
				PS_state = 0;
				MC_state = 0;
			}
			else
			{
				NA_state = 0;
				PS_state = 0;
				MC_state = 0;
			}
		} // (msg->name == "wildlife")
		else if (msg->name == "gymkhana")
		{
			if ((msg->state == "ready") || (msg->state == "running"))
			{
				NA_state = 1;
				MC_state = 5;
				if (calculations_done)
				{
					PS_state = 1;						// Turn on propulsion_system once the goal points have been calculated
				}
				else
				{
					PS_state = 0;
				}
			}
			else
			{
				NA_state = 0;
				PS_state = 0;
				MC_state = 0;
				PA_state = 0;
			}
		} // (msg->name == "gymkhana")
		else if (msg->name == "scan_dock_deliver")
		{
			if ()
			{
				NA_state = 0;
				PS_state = 0;
				MC_state = 0;
			}
			else
			{
				NA_state = 0;
				PS_state = 0;
				MC_state = 0;
			}
		} // (msg->name == "scan_dock_deliver")
		
		else
		{
			// ALL CODES ON STANDBY
			NA_state = 0;
			PS_state = 0;
			MC_state = 0;
		}
		
		// PUBLISH EACH SUBSYTEMS STATE
		// SEND STATE TO PROPULSION_SYSTEM
		propulsion_system_state.header.seq +=1;										// sequence number
		propulsion_system_state.header.stamp = current_time;					// set stamp to current time
		propulsion_system_state.header.frame_id = "mission_control";	// header frame
		propulsion_system_state.state.data = PS_state;								// set propulsion system state; 0 = OFF; 1 = ON
		ps_state_pub.publish(propulsion_system_state);							// publish propulsion_system_state to "ps_state"

		// SEND STATE TO NAVIGATION_ARRAY
		navigation_array_state.header.seq +=1;											// sequence number
		navigation_array_state.header.stamp = current_time;						// set stamp to current time
		navigation_array_state.header.frame_id = "mission_control";		// header frame
		navigation_array_state.state.data = NA_state;									// set navigation_array_state; 0 = On Standby; 1 = USV NED Pose Conversion; 2 = SK NED Goal Pose Conversion; 3 = WF NED Goal Pose Conversion; 4569 = HARD RESET (OR OTHER USE)
		na_state_pub.publish(navigation_array_state);								// publish navigation_array_state to "na_state"

		// SEND STATE TO PERCEPTION_ARRAY
		perception_array_state.header.seq +=1;											// sequence number
		perception_array_state.header.stamp = current_time;						// set stamp to current time
		perception_array_state.header.frame_id = "mission_control";		// header frame
		perception_array_state.state.data = PA_state;									// set perception_array_state; 0 = On Standby, 1 = General State, 2 = Task 3: Perception
		pa_state_pub.publish(perception_array_state);								// publish perception_array_state to "pa_state"
		
		// UPDATE USER OF EACH CODES STATE
		ROS_DEBUG("----------------- CURRENT STATES --------------------");
		ROS_DEBUG("NA_state: %i --MC", NA_state);
		ROS_DEBUG("PS_state: %i --MC", PS_state);
		ROS_DEBUG("MC_state: %i --MC", MC_state);
		ROS_DEBUG("PA_state: %i --MC", PA_state);
	} // if (mission_control_initialized)
} // END OF state_update() */

// THIS FUNCTION: Fills out current_goal_pose_msg and publishes to "current_goal_pose" for the propulsion_system
// ACCEPTS: Nothing. Uses global variable pose arrays
// RETURNS: (VOID)
// =============================================================================
void current_goal_pose_publish()
{
	// PUBLISH THE CURRENT GOAL POSE
	current_goal_pose_msg.header.seq = 0;
	current_goal_pose_msg.header.seq +=1;										// sequence number
	current_goal_pose_msg.header.stamp = current_time;				// sets stamp to current time
	current_goal_pose_msg.header.frame_id = "mission_control";	// header frame
	current_goal_pose_msg.position.x = x_goal[point];						// sets x-location
	current_goal_pose_msg.position.y = y_goal[point];						// sets y-location
	current_goal_pose_msg.position.z = 0.0;										// sets z-location
	current_goal_pose_msg.psi.data = psi_goal[point];						// sets psi

	current_goal_pose_pub.publish(current_goal_pose_msg);		// publish goal usv pose to "current_goal_pose"
} // END OF current_goal_pose_publish()
//............................................................End of Functions............................................................

int main(int argc, char **argv)
{
  // names the program for visual purposes
  ros::init(argc, argv, "mission_control");
  
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
  
  // NodeHandles
  ros::NodeHandle nh1, nh2, nh3, nh4, nh5, nh6, nh7, nh8, nh9, nh10, nh11, nh12;
  
  // Subscribers
  //ros::Subscriber task_status_sub = nh1.subscribe("/vrx/task/info", 1, state_update);																				// VRX task topic
  ros::Subscriber NA_initialization_state_sub = nh2.subscribe("na_initialization_state", 1, NAVIGATION_ARRAY_inspector);		// initialization status of navigation_array
  ros::Subscriber PS_initialization_state_sub = nh3.subscribe("ps_initialization_state", 1, PROPULSION_SYSTEM_inspector);		// initialization status of propulsion_system
  ros::Subscriber pa_initialization_state_sub = nh4.subscribe("pa_initialization_state", 1, PERCEPTION_ARRAY_inspector);		// initialization status of perception_array
  ros::Subscriber nav_NED_sub = nh5.subscribe("nav_ned", 1, pose_update);																						// current USV pose converted to NED
  ros::Subscriber waypoints_NED_sub = nh6.subscribe("waypoints_ned", 1, goal_NED_waypoints_update);									// goal waypoints converted to NED
  ros::Subscriber goal_publish_state_sub = nh7.subscribe("goal_publish_state", 1, goal_publish_state_update);								// whether or not goal waypoints have been converted yet
  
  // Publishers
  current_goal_pose_pub = nh9.advertise<amore::usv_pose_msg>("current_goal_pose", 1);								// current goal for low level controller (propulsion_system)
  ps_state_pub = nh10.advertise<amore::state_msg>("ps_state", 1);																		// current propulsion_system state
  na_state_pub = nh11.advertise<amore::state_msg>("na_state", 1);																		// current navigation_array state
  pa_state_pub = nh12.advertise<amore::state_msg>("pa_state", 1);																		// current perception_array state
  
  /* // initialize header sequences
  propulsion_system_state.header.seq = 0;
  navigation_array_state.header.seq = 0;
  perception_array_state.header.seq = 0; */
  
  // Timers ///////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Initialize simulation time
  ros::Time::init();
  
  // Initialize global variables
  current_time = ros::Time::now();						// sets current time to the time it is now
  last_time = current_time;								// sets last time to the time it is now
  
  //sets the frequency for which the program sleeps at. 10=1/10 second
  ros::Rate loop_rate(100);

  // ros::ok() will stop when the user inputs Ctrl+C
  while(ros::ok())
  {
	  current_time = ros::Time::now();														// update current_time
	  
	  MISSION_CONTROL_inspector();													// check that entire system is initialized before starting calculations
	  state_update();		// For buoy_nav_planner only /////////////////////////////////////////////////////////////////////////////////////////////////////////////////// COMMENT THIS LINE OUT IF STANDARD OPERATION
	  
	  if ((MC_state == 1) || (MC_state == 2))	// TASK 1: STATION_KEEPING OR TASK 2: WAYFINDING
	  {
		  if (loop_count > loop_goal_recieved)
		  {
			  if ((MC_state == 2) && (PS_state == 1))
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
			  } // if ((MC_state == 2) && (PS_state == 1))
			  current_goal_pose_publish();
		  } // if (loop_count > loop_goal_recieved)
	  } // if ((MC_state == 1) || (MC_state == 2))
	  
	  ros::spinOnce();
	  loop_rate.sleep();
	  
	  // update previous stats
	  /* e_xy_prev = e_xy;
	  e_psi_prev = e_psi; */
	  last_time = current_time;
	  loop_count += 1;
  } // while(ros::ok())

  ros::spinOnce();
  loop_rate.sleep();

  return 0;
}