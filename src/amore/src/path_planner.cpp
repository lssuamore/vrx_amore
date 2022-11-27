//  Filename:  path_planner.cpp
//  Creation Date:  4/07/2022
//  Last Revision Date:  8/17/2022
//  Author(s) [email]:  Brad Hacker [bhacker@lssu.edu]
//  Revisor(s) [Revision Date]:
//  Organization/Institution:  Lake Superior State University RobotX Team AMORE
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
//				Inputs [subscribers]: ["waypoints_NED" - amore/NED_poses - converted goal pose array from coordinate_converter], ["NA_nav_ned" -  nav_msgs/Odometry - current pose of usv in local NED frame from navigation_array]
//				Outputs [publishers]: ["PP_propulsion_system_topic" - amore/propulsion_system - goal pose (x,y,psi) to reach in local NED frame and current USV pose to propulsion_system]

//...............................................................................................Included Libraries and Message Types.........................................................................................
#include "ros/ros.h"
#include "ros/console.h"
#include "time.h"
#include <sstream>
#include <iostream>
#include "math.h"
#include "stdio.h"
#include "nav_msgs/Odometry.h"  // message type used for receiving USV state from navigation_array
#include "amore/state.h"  // message type used to recieve state of operation from mission_control
#include "std_msgs/Bool.h"  // message type used for communicating initialization status to mission_control
#include "amore/propulsion_system.h"  // message type that holds all needed operation information from path_planner for propulsion_system to function
#	include "amore/NED_poses.h"  // message type that holds array of converted goal poses with quantity of poses
#include "amore/NED_acoustic.h"  // message type that holds position of beacon
#include "amore/NED_objects.h"  // message type that has an array of pointstamped
//#include "geometry_msgs/PoseArray.h"  // message type used to get buoy locations from navigation_array  // FOR TASK 5
//......................................................................................End of Included Libraries and Message Types.....................................................................................

//.......................................................................................................................Constants.....................................................................................................................
#define PI 3.14159265
// THE FOLLOWING PLETHORA OF CONSTANTS ARE FOR POSE TOLERANCE FOR DIFFERENT TASKS
// NOTE: make these as small as possible through tuning improvements
#define default_position_error_allowed 1.0  // [m] default position error allowed for a given goal pose
#define default_heading_error_allowed 0.785  // [rad] default heading error allowed for a given goal pose
#define VRX2_position_error_allowed 1.0  // [m] VRX Task 2: Wayfinding
#define VRX2_heading_error_allowed 0.4  // [rad] VRX Task 2: Wayfinding
#define VRX4_position_error_allowed 3.0  // [m] this can be bigger for this task so the USV moves smoothly through path (circles around animals)
#define VRX4_heading_error_allowed 0.785  // [rad] equivalent to 45 degrees
#define max_next_position_distance 5.0  // [m] this is the maximum next position distance from the USV position to place a goal pose
#define red_buoy "mb_marker_buoy_red"  // Red marker buoy
#define green_buoy "mb_marker_buoy_green"  // Green marker buoy
#define number_of_buoy_pairs 4  // this is the number of buoy pairs for the channel navigation task
//................................................................................................................End of Constants...............................................................................................................

//.................................................................................................................Global Variables..............................................................................................................
int loop_count = 0;  // loop counter

//	STATES CONCERNED WITH "path_planner"
//	0 = On standby
//  1 = Dynamic navigation demonstration
//  2 = Entrance and Exit gates
//  3 = Follow the path
//  4 = Wildlife Encounter - React and Report
//  5 = Scan the code
//  6 = Detect and dock
//  7 = Find and Fling
//  8 = UAV replenishment
//	11 = VRX1: Station-Keeping
//	12 = VRX2: Wayfinding
//	14 = VRX4: Wildlife Encounter and Avoid
//	15 = VRX5: Channel Navigation, Acoustic Beacon Localization and Obstacle Avoidance
//	16 = VRX6: Scan and Dock and Deliver
int PP_state;

//	STATES CONCERNED WITH STATE 1 - "DnD_path_planner"
//  1 = Start point search and approach
//  2 = Search for entrance buoy pair
//  3 = Going to calculated approach- and mid- points of entrance buoy pair
//	4 = Start search and approach for exit buoy pair while continuing on current updated path
//	5 = Exit point has been reached and task is complete
int DnD_path_planner;

//	STATES CONCERNED WITH STATE 1 OF  "FtP_path_planner"
//  1 = Start point search and approach
//  2 = Search for entrance buoy pair
//  3 = Going to calculated approach- and mid- points of entrance buoy pair
//	4 = Start search and approach for exit buoy pair while continuing on current updated path
//	5 = Exit point has been reached and task is complete
int FtP_path_planner;
int buoy_pairs_found = 0;  // this keeps track of the number of times the buoy pairs have been found and had waypoints calculated to navigate through them

//	STATES CONCERNED WITH "navigation_array"
//	0 = On standby
//	1 = USV NED state converter
int NA_state;

//	STATES CONCERNED WITH "propulsion_system"
//	0 = On standby
//	1 = Propulsion system ON
int PS_state;

//	STATES CONCERNED WITH "perception_array"
//	0 = On standby
//	1 = General State
//	13 = VRX3: Landmark Localization and Characterization
//	14 = VRX4: Wildlife Encounter and Avoid
//	15 = VRX5: Channel Navigation, Acoustic Beacon Localization and Obstacle Avoidance
//	16 = VRX6: Scan and Dock and Deliver
int PA_state;

// STATES CONCERNED WITH "acoustics" 
// 0 = On standby
// 1 = Finding entrance gate (white buoy)
// 2 = Navigating between red and green buoys
// 3 = Finding exit gate (black buoy)
// 4 = Navigating to acoustic source
int A_state;

bool vrx_mode;  // false means vrx mode is not true and that user mode should be used

int point = 0;  // number of points on trajectory reached
int goal_poses_quantity;  // total number of poses to reach
int loop_goal_recieved;  // this is kept in order to ensure planner doesn't start controller until the goal is published		// CHECK IF UNEEDED AFTER REBUILD

float x_goal_poses[100], y_goal_poses[100], psi_goal_poses[100];  // arrays to hold the NED goal poses for tasks
float x_North, y_North, x_mid, y_mid, x_South, y_South;  // positions of North, mid, and South points for figure 8
float x_goal_pose, y_goal_pose, psi_goal_pose;  // current goal pose to be published to propulsion_system
int buoy_pair_updated;  // 0 = false  1 = true
float x_DnD_start, y_DnD_start, psi_DnD_start;  // start location for the dynamic navigation demonstration
float x_FtP_start, y_FtP_start, psi_FtP_start;  // start location for the follow the path task
float x_usv_NED, y_usv_NED, psi_usv_NED;  // vehicle position and heading (pose) in NED
float e_x, e_y, e_xy, e_psi;  // current errors between goal pose and usv pose
float position_tolerance;  // this is used in the figure eight path planner



bool params_updated = false;  // false means that the params for current task aren't ready to be GOTTED
bool params_GOT = false;  // false means that the params for current task haven't done been GOTTED
// Task 2: Entrance and Exit gates variables
float x_t2_black_buoy;  // task 2 black buoy x position in local NED frame
float y_t2_black_buoy;  // task 2 black buoy y position in local NED frame
float x_t2_start_end;  // task 2 start/end x position in local NED frame
float y_t2_start_end;  // task 2 start/end y position in local NED frame
float psi_t2_start_end;  // task 2 start/end heading in local NED frame

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Task 6: Detect and dock
float x_red_dock;	// x-coord of red dock in local NED frame
float y_red_dock;	// y-coord of red dock in local NED frame
float x_green_dock;	// x-coord of green dock in local NED frame
float y_green_dock;	// y-coord of green dock in local NED frame
float x_blue_dock;	// x-coord of blue dock in local NED frame
float y_blue_dock;	// y-coord of blue dock in local NED frame
float psi_dock;  // task 6 heading to station-keep in docking bay 
int Dock_Name; // 1 = Red, 2 = Green, 3 = Blue
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////





//	drive_config is the drive configuration of the low-level controller
//	1 = PID HP Dual-azimuthing station-keeping
//	2 = PID HP Differential wayfinding
//	3 = PID HP Ackermann
int drive_config = 1;

std::string Animal[3];  // string array to hold the names of the animals
float x_animals_NED[3], y_animals_NED[3];  // arrays to hold animal locations

bool CC_goal_recieved = false;  // false means NED goal poses have not been acquired from coordinate_converter
bool M_reached = false;  // false means the mid-point has not been reached for travelling through pair of buoys
bool E_reached = false;  // false means the last point has not been reached
bool E_never_reached = true;  // true means the last point has never been reached

float e_xy_allowed = default_position_error_allowed;  // positional error tolerance threshold; NOTE: make as small as possible
float e_psi_allowed = default_heading_error_allowed;  // heading error tolerance threshold; NOTE: make as small as possible

bool calculations_done = false;  // true means the path has been made
bool propulsion_system_topic_published = false;  // false means current PP_propulsion_system_topic_msg has not been published

// VRX4: Wildlife Encounter and Avoid variables
float x_c_NED, y_c_NED, psi_c_NED;  // crocodile position and heading (pose) in NED
float x_p_NED, y_p_NED, psi_p_NED;  // platypus position and heading (pose) in NED
float x_t_NED, y_t_NED, psi_t_NED;  // turtle position and heading (pose) in NED
float dc_USV, dt_USV, dp_USV , dp_c, dt_c;  // [m] distances between animals and USV
bool animal_usv_distances_calculated = false;  // false means the distances between the usv and the turtle and platypus have not been calculated
float r;  // [m] radius for the circle paths around the animals
// Array of poses for making the turtle circle
float x_turt_g[9];
float y_turt_g[9];
float psi_turt_g[9];
// Array of poses for making the platypus circle
float x_plat_g[9];
float y_plat_g[9];
float psi_plat_g[9];

// VARIABLES FOR THE BUOY NAVIGATION CALCULATIONS - calculate_buoy_waypoints()
// Centroid locations of left and right buoys given by "perception_array"
float CL_x;  // x-location of left buoy wrt USV
float CL_y;  // y-location of left buoy wrt USV
float CR_x;  // x-location of right buoy wrt USV
float CR_y;  // y-location of right buoy wrt USV

float CL_x_NED;  // x-location of left buoy centroid in global frame
float CL_y_NED;  // y-location of left buoy centroid in global frame
float CR_x_NED;  // x-location of right buoy centroid in global frame
float CR_y_NED;  // y-location of right buoy centroid in global frame

float I_x;  // x-coord. of intermediate point wrt global
float I_y;  // y-coord. of intermediate point wrt global

float M_x;  // x-location of midpoint
float M_y;  // y-location of midpoint

float E_x;  // x-coord. of exit point wrt global
float E_y;  // y-coord. of exit point wrt global

float d_L;  // distance from USV to left buoy
float d_M;  // distance from USV to midpoint
float d_R;  // distance from USV to right buoy
float d_I;  // distance from midpoint to approach point

float d_LM;  // half distance between left and right buoys
float a_L;  // distance between left buoy and the approach point
float theta;  // angle created by d_I and d_LM

float x_I_CL;  // x-coord. of intermediate point wrt left buoy
float y_I_CL;  // y-coord. of intermediate point wrt left buoy

float x_E_CL;  // x-coord. of exit point wrt left buoy
float y_E_CL;  // y-coord. of exit point wrt left buoy

float s_M;  // slope of of line from CL to CR
float alpha;  // angle of frame CL wrt global frame

bool buoy_positions_recieved = false; // false means the NED_buoys array has not yet been acquired USED FOR BUOY NAVIGATION POSSIBLY
bool left_buoy_found = false;  // false means that the left buoy position was not found
bool right_buoy_found = false;  // false means that the right buoy position was not found

std_msgs::Bool PP_initialization_state_msg;  // "PP_initialization_state" message
ros::Publisher PP_initialization_state_pub;  // "PP_initialization_state" publisher

std_msgs::Bool PP_params_GOT_msg;  // "PP_params_GOT" message
ros::Publisher PP_params_GOT_pub;  // "PP_params_GOT" publisher

std_msgs::Bool PP_USV_pose_update_state_msg;  // "PP_USV_pose_update_state" message; false means NED usv pose has not been updated 
ros::Publisher PP_USV_pose_update_state_pub;  // "PP_USV_pose_update_state" publisher

amore::propulsion_system PP_propulsion_system_topic_msg;  // "PP_propulsion_system_topic" message
ros::Publisher PP_propulsion_system_topic_pub;  // "PP_propulsion_system_topic" publisher

ros::Time current_time;  // creates time variables
//..........................................................................................................End of Global Variables........................................................................................................

//.....................................................................................................................Functions.......................................................................................................................
// THIS FUNCTION: Updates the state of mission_control when not in vrx_mode
// ACCEPTS: (VOID)
// RETURNS: (VOID)
// =====================================================
void parameters_function()
{
	if (PP_state == 1)  // 1 = Dynamic navigation demonstration
	{
		ros::param::get("/x_DnD", x_DnD_start);
		ros::param::get("/y_DnD", y_DnD_start);
		ros::param::get("/psi_DnD", psi_DnD_start);
	}
	if (PP_state == 2)	//	2 = Entrance and Exit gates
	{
		ros::param::get("/x_t2_first", x_t2_start_end);
		ros::param::get("/y_t2_first", y_t2_start_end);
		ros::param::get("/psi_t2_first", psi_t2_start_end);
		while ((psi_t2_start_end < -PI) || (psi_t2_start_end > PI))
		{
			// Adjust psi_t2_start_end back within -PI and PI
			if (psi_t2_start_end < -PI)
			{
				psi_t2_start_end = psi_t2_start_end + 2.0*PI;
			}
			if (psi_t2_start_end > PI)
			{
				psi_t2_start_end = psi_t2_start_end - 2.0*PI;
			}
		}
		ros::param::get("/x_t2_blackie", x_t2_black_buoy);
		ros::param::get("/y_t2_blackie", y_t2_black_buoy);
	}
	if (PP_state == 3)  // 3 = Follow the path
	{
		ros::param::get("/x_FtP", x_FtP_start);
		ros::param::get("/y_FtP", y_FtP_start);
		ros::param::get("/psi_FtP", psi_FtP_start);
	}
	if(PP_state == 6)  // 6 = Detect and dock params
	{
		ros::param::get("/x_Red_Dock", x_red_dock);
		ros::param::get("/y_Red_Dock", y_red_dock);
		ros::param::get("/x_Green_Dock", x_green_dock);
		ros::param::get("/y_Green_Dock", y_green_dock);
		ros::param::get("/x_Blue_Dock", x_blue_dock);
		ros::param::get("/y_Blue_Dock", y_blue_dock);
		ros::param::get("/psi_t6_dock", psi_dock);
		while ((psi_dock < -PI) || (psi_dock > PI))
		{
			// Adjust psi_dock back within -PI and PI
			if (psi_dock < -PI)
			{
				psi_dock += 2.0*PI;
			}
			if (psi_dock > PI)
			{
				psi_dock -= 2.0*PI;
			}
		}
		ros::param::get("/Dock_color", Dock_Name);
	}
	if (PP_state == 11)	//	11 = Station-Keeping
	{
		ros::param::get("/x_G", x_goal_poses[0]);
		ros::param::get("/y_G", y_goal_poses[0]);
		ros::param::get("/psi_G", psi_goal_poses[0]);
		while ((psi_goal_poses[0] < -PI) || (psi_goal_poses[0] > PI))
		{
			// Adjust psi_goal_poses[0] back within -PI and PI
			if (psi_goal_poses[0] < -PI)
			{
				psi_goal_poses[0] = psi_goal_poses[0] + 2.0*PI;
			}
			if (psi_goal_poses[0] > PI)
			{
				psi_goal_poses[0] = psi_goal_poses[0] - 2.0*PI;
			}
		}
		point = 0;
		goal_poses_quantity = 1;
	}
	if (PP_state == 12)  //	12 = VRX2: Wayfinding - Figure Eight
	{
		ros::param::get("/x_north", x_North);
		ros::param::get("/y_north", y_North);
		ros::param::get("/x_south", x_South);
		ros::param::get("/y_south", y_South);
	}	
	ros::param::get("/updated_buoy_pair", buoy_pair_updated);
	if (buoy_pair_updated == 1)
	{
		ros::param::get("/x_LB", CL_x);
		ros::param::get("/y_LB", CL_y);
		ros::param::get("/x_RB", CR_x);
		ros::param::get("/y_RB", CR_y);
		buoy_positions_recieved = true;
		ros::param::set("/updated_buoy_pair", 0);
	}

	params_GOT = true;  // true means the task parameters have been acquired
	// reset that the parameters have been updated
	ros::param::set("/params_Updated", false);
	params_updated = false;
} // END OF parameters_function()

// THIS FUNCTION: Updates global current_time, loop_count, and publishes initialization status to "PP_initialization_state"
// ACCEPTS: (VOID)
// RETURNS: (VOID)
//=============================================================================================================
void PATH_PLANNER_inspector()
{
	loop_count += 1;  // increment loop counter
	if (loop_count > 3)
	{
		PP_initialization_state_msg.data = true;
		//ROS_INFO("PATH_PLANNER: path_planner_initialized");
	}
	else
	{
		PP_initialization_state_msg.data = false;
		//ROS_INFO("PATH_PLANNER: !path_planner_initialized");
	}
	ros::param::get("/params_Updated", params_updated);
	if ((!vrx_mode) && (params_updated))
	{
		parameters_function();
	}
	if ((propulsion_system_topic_published) && (PP_state == 14))  // if the PP_propulsion_system_topic has been published and VRX4: Wildlife Encounter and Avoid
	{
		// reset for next update of animal locations
		CC_goal_recieved = false;  // false means NED goal poses have not been acquired from coordinate_converter
		calculations_done = false;  // false means the wildlife path has not been made
	}
	PP_initialization_state_pub.publish(PP_initialization_state_msg);  // publish the initialization status of the path_planner to "PP_initialization_state"

	PP_params_GOT_msg.data = params_GOT;
	PP_params_GOT_pub.publish(PP_params_GOT_msg);  // publish the status of whether or not the params have been GOT to let mission_control know if propulsion system should be started
	
	PP_USV_pose_update_state_pub.publish(PP_USV_pose_update_state_msg);  // publish whether or not NED usv pose has been updated
	// reset for next main loop
	propulsion_system_topic_published = false;  // false means current PP_propulsion_system_topic_msg has not been published
	PP_USV_pose_update_state_msg.data = false;  // false means NED USV pose has not been updated
}  // END OF PATH_PLANNER_inspector()

// THIS FUNCTION: Updates the state of "path_planner" given by "mission_control"
// ACCEPTS: amore::state from "MC_pp_state"
// RETURNS: (VOID)
//=============================================================================================================
void MC_pp_state_update(const amore::state::ConstPtr& msg)
{
	if (PP_initialization_state_msg.data)
	{
		PP_state = msg->state.data;
		vrx_mode = msg->sim_mode.data;
		//ROS_INFO("PATH_PLANNER: PP_state = %i", PP_state);
	}
}  // END OF MC_pp_state_update()

// THIS FUNCTION: Updates the state of "navigation_array" given by "mission_control"
// ACCEPTS: amore::state from "MC_na_state"
// RETURNS: (VOID)
//=============================================================================================================
void MC_na_state_update(const amore::state::ConstPtr& msg)
{
	if (PP_initialization_state_msg.data)
	{
		NA_state = msg->state.data;
	}
}  // END OF MC_na_state_update()

// THIS FUNCTION: Updates the state of "propulsion_system" given by "mission_control"
// ACCEPTS: amore::state from "MC_ps_state"
// RETURNS: (VOID)
//=============================================================================================================
void MC_ps_state_update(const amore::state::ConstPtr& msg)
{
	if (PP_initialization_state_msg.data)
	{
		PS_state = msg->state.data;
	}
}  // END OF MC_ps_state_update()

// THIS FUNCTION: Updates the state of "perception_array" given by "mission_control"
// ACCEPTS: amore::state from "MC_pa_state"
// RETURNS: (VOID)
//=============================================================================================================
void MC_pa_state_update(const amore::state::ConstPtr& msg)
{
	if (PP_initialization_state_msg.data)
	{
		PA_state = msg->state.data;
	}
}  // END OF MC_pa_state_update()

// THIS FUNCTION: Updates the state of "acoustics" given by "mission_control"
// ACCEPTS: amore::state from "MC_a_state"
// RETURNS: (VOID) Updates global variable
//=============================================================================================================
void MC_a_state_update(const amore::state::ConstPtr& msg)
{
	if (PP_initialization_state_msg.data)
	{
		A_state = msg->state.data;
	}
} // END OF MC_a_state_update()

// THIS FUNCTION: Updates the current NED USV pose converted through the navigation_array
// ACCEPTS: nav_msgs::Odometry from "NA_nav_ned"
// RETURNS: (VOID)
//=============================================================================================================
void NA_nav_ned_update(const nav_msgs::Odometry::ConstPtr& odom)
{
	// Update NED USV pose
	x_usv_NED = odom->pose.pose.position.x;
	y_usv_NED = odom->pose.pose.position.y;
	psi_usv_NED = odom->pose.pose.orientation.z;
	PP_USV_pose_update_state_msg.data = true;  // true means NED USV pose has been updated
} // END OF NA_nav_ned_update()

// THIS FUNCTION: Updates the goal poses for VRX Tasks 1 & 2 converted to NED through the coordinate_converter
// ACCEPTS: amore::NED_poses from "CC_goal_poses_ned"
// RETURNS: (VOID)
//=============================================================================================================
void CC_goal_poses_ned_update(const amore::NED_poses::ConstPtr& goal)
{
	if (!CC_goal_recieved)  // if the NED goal poses have been published but not recieved yet
	{
		ROS_INFO("PATH_PLANNER: GOAL POSES ACQUIRED BY PLANNER");  // UPDATE USER
		goal_poses_quantity = goal->quantity;  // get the data of the goal pose array
		for (int i = 0; i < goal_poses_quantity; i++)
		{
			x_goal_poses[i] = goal->poses[i].x;
			y_goal_poses[i] = goal->poses[i].y;
			psi_goal_poses[i] = goal->poses[i].z;
			// UPDATE USER
			if (i < (goal_poses_quantity-1))
			{
				ROS_INFO("PATH_PLANNER: pose: %i    x: %.2f    y: %.2f    psi: %.2f", i, x_goal_poses[i], y_goal_poses[i], psi_goal_poses[i]);
			}
			else  // if its the last pose print with the next line (\n) to make the monitor display nice
			{
				ROS_INFO("PATH_PLANNER: pose: %i    x: %.2f    y: %.2f    psi: %.2f\n", i, x_goal_poses[i], y_goal_poses[i], psi_goal_poses[i]);
			}
		}
		loop_goal_recieved = loop_count;
		CC_goal_recieved = true;  // true means NED goal poses have been acquired from coordinate_converter
	}  // END OF if (!CC_goal_recieved)
} // END OF CC_goal_poses_ned_update()

// THIS FUNCTION: Sets goal pose
// ACCEPTS: float values for x,y, and psi of goal pose to be set in local NED frame
// RETURNS: (VOID)
//=============================================================================================================
void set_goal_pose(float x_goal, float y_goal, float psi_goal)
{
	x_goal_pose = x_goal;
	y_goal_pose = y_goal;
	psi_goal_pose = psi_goal;
	// ensure psi_goal_pose stays within -PI and PI 
	while ((psi_goal_pose < -PI) || (psi_goal_pose > PI))
	{
		if (psi_goal_pose < -PI)
		{
			psi_goal_pose += 2.0*PI;
		}
		if (psi_goal_pose > PI)
		{
			psi_goal_pose -= 2.0*PI;
		}
	}
}  // END OF set_goal_pose()

// THIS FUNCTION: Sets next goal pose x meters in front of USV
// ACCEPTS: float values for distance to move forward
// RETURNS: (VOID)
//=============================================================================================================
void go_forward(float dist)
{
	x_goal_poses[point] = x_usv_NED + cos(psi_usv_NED)*dist;
	y_goal_poses[point] = y_usv_NED + sin(psi_usv_NED)*dist;
	psi_goal_poses[point] = psi_usv_NED;
	
	// x_goal_pose = x_usv_NED + cos(psi_usv_NED)*dist;
	// y_goal_pose = y_usv_NED + sin(psi_usv_NED)*dist;
	// psi_goal_pose = psi_usv_NED;
}  // END OF go_forward()

// THIS FUNCTION: Calculates pose errors
// ACCEPTS: (VOID)
// RETURNS: (VOID)
//=============================================================================================================
void calculate_pose_errors()
{
	e_x = x_goal_pose - x_usv_NED;  // calculate error in x position
	e_y = y_goal_pose - y_usv_NED;  // calculate error in y position
	e_xy = sqrt(pow(e_x,2.0)+pow(e_y,2.0));  // calculate magnitude of positional error
	e_psi = psi_goal_pose - psi_usv_NED;  // calculate error in heading
	while ((e_psi < -PI) || (e_psi > PI))
	{
		// Adjust e_psi back within -PI and PI
		if (e_psi < -PI)
		{
			e_psi += 2.0*PI;
		}
		if (e_psi > PI)
		{
			e_psi -= 2.0*PI;
		}
	}
}  // END OF calculate_pose_errors()

// THIS FUNCTION: Display USV and goal pose
// ACCEPTS: (VOID)
// RETURNS: (VOID)
//=============================================================================================================
void display_usv_and_goal_pose()
{
	ROS_INFO("PATH_PLANNER:-----USV POSE-----");
	ROS_INFO("PATH_PLANNER:     x   :  %4.2f", x_usv_NED);  // x posn.
	ROS_INFO("PATH_PLANNER:     y   :  %4.2f", y_usv_NED);  // y posn.
	ROS_INFO("PATH_PLANNER:     psi :  %4.2f", psi_usv_NED);  // heading
	ROS_INFO("PATH_PLANNER:-----USV POSE-----\n");
	ROS_INFO("PATH_PLANNER:-----GOAL POSE-----");
	ROS_INFO("PATH_PLANNER:     x   :  %4.2f", x_goal_pose);  // x posn.
	ROS_INFO("PATH_PLANNER:     y   :  %4.2f", y_goal_pose);  // y posn.
	ROS_INFO("PATH_PLANNER:     psi :  %4.2f", psi_goal_pose);  // heading
	ROS_INFO("PATH_PLANNER:-----GOAL POSE-----\n");
}  // END OF display_usv_and_goal_pose()

// THIS FUNCTION: Display pose errors
// ACCEPTS: (VOID)
// RETURNS: (VOID)
//=============================================================================================================
void display_pose_errors()
{
	ROS_INFO("PATH_PLANNER:-----POSE ERRORS-----");
	ROS_INFO("PATH_PLANNER:     e_x   :  %4.2f", e_x);  // x posn. error
	ROS_INFO("PATH_PLANNER:     e_y   :  %4.2f", e_y);  // y posn. error
	ROS_INFO("PATH_PLANNER:     e_xy  :  %4.2f", e_xy);  // magnitude of posn. error
	ROS_INFO("PATH_PLANNER:     e_psi :  %4.2f", e_psi);  // heading error
	ROS_INFO("PATH_PLANNER:-----POSE ERRORS-----\n");
}  // END OF display_pose_errors()

// THIS FUNCTION: Fills out PP_propulsion_system_topic_msg and publishes to "PP_propulsion_system_topic" for the propulsion_system
// ACCEPTS: (VOID) Uses global variable pose arrays
// RETURNS: (VOID)
//=============================================================================================================
void propulsion_system_topic_publish()
{
	// PUBLISH THE CURRENT GOAL POSE
	// fill message header
	PP_propulsion_system_topic_msg.header.seq = 0;
	PP_propulsion_system_topic_msg.header.seq +=1;  // sequence number
	PP_propulsion_system_topic_msg.header.stamp = current_time;  // sets stamp to current time
	PP_propulsion_system_topic_msg.header.frame_id = "path_planner";  // header frame
	// fill goal pose
	PP_propulsion_system_topic_msg.goal_position.x = x_goal_pose;  // sets goal x-location
	PP_propulsion_system_topic_msg.goal_position.y = y_goal_pose;  // sets goal y-location
	PP_propulsion_system_topic_msg.goal_position.z = 0.0;  // sets goal z-location
	PP_propulsion_system_topic_msg.goal_psi.data = psi_goal_pose;  // sets goal psi
	// fill current USV pose
	PP_propulsion_system_topic_msg.usv_position.x = x_usv_NED;  // sets current USV x-location 
	PP_propulsion_system_topic_msg.usv_position.y = y_usv_NED;  // sets current USV y-location
	PP_propulsion_system_topic_msg.usv_position.z = 0.0;  // sets current USV z-location
	PP_propulsion_system_topic_msg.usv_psi.data = psi_usv_NED;  // sets current USV psi
	// fill pose error tolerance
	PP_propulsion_system_topic_msg.e_xy_allowed.data = e_xy_allowed;  // sets position error tolerance
	PP_propulsion_system_topic_msg.e_psi_allowed.data = e_psi_allowed;  // sets heading error tolerance
	PP_propulsion_system_topic_msg.drive_configuration.data = drive_config;  // define which drive configuration to use

	// ROS_INFO("PATH_PLANNER:------PUBLISHING POINT-----");  // UPDATE USER
	// ROS_INFO("PATH_PLANNER: Point x: %4.2f    Point y: %4.2f\n", x_goal_poses[point], y_goal_poses[point]);  // UPDATE USER

	PP_propulsion_system_topic_pub.publish(PP_propulsion_system_topic_msg);  // publish goal usv pose to "current_goal_pose"
	propulsion_system_topic_published = true;  // true means current PP_propulsion_system_topic_msg has been published
}  // END OF propulsion_system_topic_publish()

// VRX TASK 4: Wildlife Encounter and Avoid
// THIS FUNCTION: Updates the converted local NED positions of the animals
// ACCEPTS: amore::NED_objects from "NED_animals"
// RETURNS: (VOID)
//=============================================================================================================
void CC_animals_ned_update(const amore::NED_objects::ConstPtr& object)
{
	if (!CC_goal_recieved)  // if the NED goal waypoints have been published but not recieved yet
	{
		//ROS_INFO("PATH_PLANNER: GOAL ANIMAL POSES ACQUIRED BY PLANNER.");  // UPDATE USER
		for (int i = 0; i < object->quantity; i++)
		{
		   Animal[i] = object->objects[i].header.frame_id;  // Getting array of animal names
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
		   x_animals_NED[i] = object->objects[i].point.x;  // Getting x position of animals 
		   y_animals_NED[i] = object->objects[i].point.y;  // Getting y position of animals
		   //ROS_INFO("PATH_PLANNER: Animal: %s    x: %4.2f    y: %4.2f", Animal[i].c_str(), x_animals_NED[i], y_animals_NED[i]);  // UPDATE USER
		}
		loop_goal_recieved = loop_count;
		if ((x_t_NED != y_t_NED) || (x_t_NED != y_p_NED) || (x_t_NED != x_p_NED))
		{
			CC_goal_recieved = true;  // true means NED goal poses have been acquired from coordinate_converter
		}
	}  // END OF if (!CC_goal_recieved)
}  // END OF CC_animals_ned_update()

// THIS FUNCTION: Calculates the distances between the USV and the animals
// ACCEPTS: (VOID)
// RETURNS: (VOID)
//=============================================================================================================
void animal_distances_calculate()
{
	dc_USV = sqrt(pow(x_usv_NED - x_c_NED, 2.0)+pow(y_usv_NED - y_c_NED, 2.0));  // Distance from USV to crocodile
	dp_USV = sqrt(pow(x_usv_NED - x_p_NED, 2.0)+pow(y_usv_NED - y_p_NED, 2.0));  // Distance from USV to platypus
	dt_USV = sqrt(pow(x_usv_NED - x_t_NED, 2.0)+pow(y_usv_NED - y_t_NED, 2.0));  // Distance from USV to turtle
	dt_c = sqrt(pow(x_t_NED - x_c_NED, 2.0)+pow(y_t_NED - y_c_NED, 2.0));  // Distance from turtle to crocodile
	dp_c  = sqrt(pow(x_p_NED - x_c_NED, 2.0)+pow(y_p_NED - y_c_NED, 2.0));  // Distance from platypus to crocodile
	
	ROS_INFO("PATH_PLANNER:----DISTANCES CALCULATED----");
	ROS_INFO("PATH_PLANNER: x_USV: %4.2f     y_USV: %4.2f", x_usv_NED, y_usv_NED);
	ROS_INFO("PATH_PLANNER: x_plat: %4.2f    y_plat: %4.2f", x_p_NED, y_p_NED);
	ROS_INFO("PATH_PLANNER: x_turt: %4.2f    y_turt: %4.2f", x_t_NED, y_t_NED);
	ROS_INFO("PATH_PLANNER: dp_USV: %4.2f     dt_USV: %4.2f", dp_USV, dt_USV);
	ROS_INFO("PATH_PLANNER:----DISTANCES CALCULATED----\n");
	animal_usv_distances_calculated = true;  // true means the distances between the usv and the turtle and platypus have been calculated
}  // END OF animal_distances_calculate()

// THIS FUNCTION: Generates the updated array of poses to accomplish the task
// ACCEPTS: (VOID)
// RETURNS: (VOID)
//=============================================================================================================
void update_animal_path()
{
	if (!animal_usv_distances_calculated)  // if the distances between the USV and the animals has not yet been calculated
	{
		animal_distances_calculate();  // updates distances from USV to each animal
	}
	else if (animal_usv_distances_calculated)
	{
		r = 5;
		// Making the circle ccw around turtle
		// NOTE: currently there is 3.83 [m] between each point
		x_turt_g[0] = x_t_NED - r;
		y_turt_g[0] = y_t_NED;
		psi_turt_g[0] = 90.0 * (PI/180);
		x_turt_g[1] = x_t_NED - (sqrt(2.0)/2.0)*r;
		y_turt_g[1] = y_t_NED + (sqrt(2.0)/2.0)*r;
		psi_turt_g[1] = 45.0 * (PI/180);
		x_turt_g[2] = x_t_NED;
		y_turt_g[2] = y_t_NED + r;
		psi_turt_g[2] = 0.0 * (PI/180);
		x_turt_g[3] = x_t_NED + (sqrt(2.0)/2.0)*r;
		y_turt_g[3] = y_t_NED + (sqrt(2.0)/2.0)*r;
		psi_turt_g[3] = -45.0 * (PI/180);
		x_turt_g[4] = x_t_NED + r;
		y_turt_g[4] = y_t_NED;
		psi_turt_g[4] = -90.0 * (PI/180);
		x_turt_g[5] = x_t_NED + (sqrt(2.0)/2.0)*r;
		y_turt_g[5] = y_t_NED - (sqrt(2.0)/2.0)*r;
		psi_turt_g[5] = -135.0 * (PI/180);
		x_turt_g[6] = x_t_NED;
		y_turt_g[6] = y_t_NED - r;
		psi_turt_g[6] = 180.0 * (PI/180);
		x_turt_g[7] = x_t_NED - (sqrt(2.0)/2.0)*r;
		y_turt_g[7] = y_t_NED - (sqrt(2.0)/2.0)*r;
		psi_turt_g[7] = 135.0 * (PI/180);
		x_turt_g[8] = x_t_NED - r;
		y_turt_g[8] = y_t_NED;
		psi_turt_g[8] = 90.0 * (PI/180);  // change to point away from animal 

		// Making the circle cw around platypus
		x_plat_g[0] = x_p_NED + r;
		y_plat_g[0] = y_p_NED;
		psi_plat_g[0] = 90.0 * (PI/180.0);
		x_plat_g[1] = x_p_NED + (sqrt(2.0)/2.0)*r;
		y_plat_g[1] = y_p_NED + (sqrt(2.0)/2.0)*r;
		psi_plat_g[1] = 135.0 * (PI/180);
		x_plat_g[2] = x_p_NED;
		y_plat_g[2] = y_p_NED + r;
		psi_plat_g[2] = -180.0 * (PI/180);
		x_plat_g[3] = x_p_NED - (sqrt(2.0)/2.0)*r;
		y_plat_g[3] = y_p_NED + (sqrt(2.0)/2.0)*r;
		psi_plat_g[3] = -135.0 * (PI/180);
		x_plat_g[4] = x_p_NED - r;
		y_plat_g[4] = y_p_NED;
		psi_plat_g[4] = -90.0 * (PI/180);
		x_plat_g[5] = x_p_NED - (sqrt(2.0)/2.0)*r;
		y_plat_g[5] = y_p_NED - (sqrt(2.0)/2.0)*r;
		psi_plat_g[5] = -45.0 * (PI/180);
		x_plat_g[6] = x_p_NED;
		y_plat_g[6] = y_p_NED - r;
		psi_plat_g[6] = 0.0 * (PI/180);
		x_plat_g[7] = x_p_NED + (sqrt(2.0)/2.0)*r;
		y_plat_g[7] = y_p_NED - (sqrt(2.0)/2.0)*r;
		psi_plat_g[7] = 45.0 * (PI/180);
		x_plat_g[8] = x_p_NED + r;
		y_plat_g[8] = y_p_NED;
		psi_plat_g[8] = 90.0 * (PI/180);

		if (dt_USV <= dp_USV)  // if the distance between the USV and turtle is less than or equal to the distance between the USV and the platypus
		{
			for (int i=0; i<9; i++)
			{
				// go to the turtle first
				x_goal_poses[i] = x_turt_g[i];
				y_goal_poses[i] = y_turt_g[i];
				psi_goal_poses[i] = psi_turt_g[i];
			}
			//ROS_INFO("x_t_NED: %4.2f    y_t_NED: %4.2f    x_p_NED: %4.2f    y_p_NED: %4.2f", x_t_NED, y_t_NED, x_p_NED, y_p_NED);

			for (int i=9; i<18; i++)
			{
				// go to the platypus second
				x_goal_poses[i] = x_plat_g[i-9];
				y_goal_poses[i] = y_plat_g[i-9];
				psi_goal_poses[i] = psi_plat_g[i-9];
			}
		}
		else  // if the distance between the USV and turtle is greater than the distance between the USV and the platypus
		{
			for (int i=0; i<9; i++)
			{
				// go to the platypus first
				x_goal_poses[i] = x_plat_g[i];
				y_goal_poses[i] = y_plat_g[i];
				psi_goal_poses[i] = psi_plat_g[i];
			}
			for (int i=9; i<18; i++)
			{
				// go to the turtle second
				x_goal_poses[i] = x_turt_g[i-9];
				y_goal_poses[i] = y_turt_g[i-9];
				psi_goal_poses[i] = psi_turt_g[i-9];
			}
		}
		calculations_done = true;
		goal_poses_quantity = 18;
	}
}  // END OF update_animal_path()

// VRX TASK 5: Channel Navigation, Acoustic Beacon Localization and Obstacle Avoidance path planner
// THIS FUNCTION: Updates the NED acoustic source location
// ACCEPTS: amore::NED_acoustic from "A_source_location"
// RETURNS: (VOID)
//=============================================================================================================
void A_source_location_update(const amore::NED_acoustic::ConstPtr& sourceloc) 
{
	if (PP_state == 15) // if path_planner gymkhana task (task 5)
	{
		if (!CC_goal_recieved)  // if the NED goal waypoints have been published but not recieved yet
		{
			goal_poses_quantity = sourceloc->quantity;
			for (int i = 0; i < goal_poses_quantity; i++)
			{
				x_goal_poses[i] = sourceloc->points[i].x;
				y_goal_poses[i] = sourceloc->points[i].y;
				psi_goal_poses[i] = sourceloc->points[i].z;
				// ROS_INFO("Point x: %4.2f		Point y: %4.2f --PP", x_goal_poses[i], y_goal_poses[i]);
			}
			loop_goal_recieved = loop_count;
			CC_goal_recieved = true;  // true means NED goal poses have been acquired from coordinate_converter
		}  // END OF if (!CC_goal_recieved)
	}
}  // END OF A_source_location_update()

// START OF FUNCTIONS FOR VISION PATH PLANNER 
// update left and right buoy centroid location (x,y) in global frame
void NED_buoys_update (const amore::NED_objects::ConstPtr& buoys)
{
	ROS_INFO("Number of bouys detected: %i", buoys->quantity);
	if ((PP_initialization_state_msg.data) && (!buoy_positions_recieved))
	{
		if ((PP_state == 15) || (DnD_path_planner == 2) || (DnD_path_planner == 4) || (PP_state == 2) || (PP_state == 3))
		{
			for (int i = 0; i < buoys->quantity; i++)
			{
				ROS_INFO("Check%i", i);
				if ((buoys->objects[i].point.x < 15.0) && (buoys->objects[i].point.x > 5.0) && (buoys->objects[i].point.y < 10.0) && (buoys->objects[i].point.y > -10.0))
				{
					ROS_INFO("GOOD");
				}
				else
				{
					ROS_INFO("BAD");
				}
				if (strcmp(buoys->objects[i].header.frame_id.c_str(), red_buoy) == 0)
				{
					ROS_INFO("RED");
				}
				if (strcmp(buoys->objects[i].header.frame_id.c_str(), green_buoy) == 0)
				{
					ROS_INFO("GREEN");
				}
				if ((strcmp(buoys->objects[i].header.frame_id.c_str(), red_buoy) == 0) && (buoys->objects[i].point.x < 15.0) && (buoys->objects[i].point.x > 5.0) && (buoys->objects[i].point.y < 10.0) && (buoys->objects[i].point.y > -10.0))
				{
					CL_x = buoys->objects[i].point.x;
					CL_y = buoys->objects[i].point.y;
					left_buoy_found = true;
				}
				if ((strcmp(buoys->objects[i].header.frame_id.c_str(), green_buoy) == 0) && (buoys->objects[i].point.x < 15.0) && (buoys->objects[i].point.x > 5.0) && (buoys->objects[i].point.y < 10.0) && (buoys->objects[i].point.y > -10.0))
				{
					CR_x = buoys->objects[i].point.x;
					CR_y = buoys->objects[i].point.y;
					right_buoy_found = true;
				}
			}
			if (!left_buoy_found)
			{
				ROS_INFO("PATH_PLANNER:---LEFT BUOY NOT FOUND---");
			}
			if (!right_buoy_found)
			{
				ROS_INFO("PATH_PLANNER:---RIGHT BUOY NOT FOUND---");
			}
			if ((left_buoy_found) && (right_buoy_found))
			{
				buoy_positions_recieved = true;
			}
			// PERHAPS ADD CODE HERE TO SLIGHLTY ADJUST CURRENT GOAL HEADING TO ROTATE THE VEHICLE SIGHT BACK TOWARDS NEXT SET OF BUOYS
			if (!buoy_positions_recieved)
			{
				if ((left_buoy_found) && (!right_buoy_found))
				{
					ROS_INFO("PATH_PLANNER:---Adjusting goal heading to find buoy pair location---");
					psi_goal_pose += PI/18;
				}
				else if ((right_buoy_found) && (!left_buoy_found))
				{
					ROS_INFO("PATH_PLANNER:---Adjusting goal heading to find buoy pair location---");
					psi_goal_pose -= PI/18;
				}
				// ensure psi_goal_pose stays within -PI and PI 
				while ((psi_goal_pose < -PI) || (psi_goal_pose > PI))
				{
					if (psi_goal_pose < -PI)
					{
						psi_goal_pose += 2.0*PI;
					}
					if (psi_goal_pose > PI)
					{
						psi_goal_pose -= 2.0*PI;
					}
				}
			}
			// reset for next update
			left_buoy_found = false;
			right_buoy_found = false;
		}
	}
}  // END OF NED_buoys_update()

// THIS FUNCTION: Calculates the updated approach-, mid-, and exit- points to go through the current pair of buoys
// ACCEPTS: (VOID)
// RETURNS: (VOID)
//=============================================================================================================
void calculate_buoy_waypoints()
{
	// hardcode values
	//CL_x = 37.85;
	//CL_y = 2.2;
	//CR_x = 37.85;
	//CR_y = 15.75;
	//x_usv_NED = 24.24;
	//y_usv_NED = 11.88;
	//psi_usv_NED = 0.0;
	ROS_INFO("________vvvvvvvvvvv  {PP}  vvvvvvvvvvv___________________\n");
	ROS_INFO("~~~USV pose wrt local NED frame~~~");
	ROS_INFO("psi: %f", psi_usv_NED);
	ROS_INFO("x: %f", x_usv_NED);
	ROS_INFO("y: %f\n", y_usv_NED);
	ROS_INFO("~~~Buoy locations wrt USV~~~");
	ROS_INFO("LB_x: %f", CL_x);
	ROS_INFO("LB_y: %f", CL_y);
	ROS_INFO("RB_x: %f", CR_x);
	ROS_INFO("RB_y: %f\n", CR_y);
	CL_x_NED = cos(psi_usv_NED)*CL_x - sin(psi_usv_NED)*CL_y + x_usv_NED;
	CL_y_NED = sin(psi_usv_NED)*CL_x + cos(psi_usv_NED)*CL_y + y_usv_NED;
	CR_x_NED = cos(psi_usv_NED)*CR_x - sin(psi_usv_NED)*CR_y + x_usv_NED;
	CR_y_NED = sin(psi_usv_NED)*CR_x + cos(psi_usv_NED)*CR_y + y_usv_NED;
	ROS_INFO("~~~Buoy locations wrt local NED frame~~~");
	ROS_INFO("LB_x: %f", CL_x_NED);
	ROS_INFO("LB_y: %f", CL_y_NED);
	ROS_INFO("RB_x: %f", CR_x_NED);
	ROS_INFO("RB_y: %f\n", CR_y_NED);

	M_x = (CL_x_NED+CR_x_NED)/2.0;  // x-location of midpoint
	M_y = (CL_y_NED+CR_y_NED)/2.0;  // y-location of midpoint

	d_L = sqrt(pow((CL_x_NED-x_usv_NED),2.0)+pow((CL_y_NED-y_usv_NED),2.0));  // distance from USV to left buoy
	d_M = sqrt(pow((M_x-x_usv_NED),2.0)+pow((M_y-y_usv_NED),2.0));  // distance from USV to midpoint
	d_R = sqrt(pow((CR_x_NED-x_usv_NED),2.0)+pow((CR_y_NED-y_usv_NED),2.0));  // distance from USV to right buoy
	d_I = (d_L+d_M+d_R)/3.0;  // distance from midpoint to approach point

	// intermediate approach point doesn't need to be more than 4 meters back from midpoint
	if (d_I > 5.0)
	{
	  d_I = 5.0;
	}

	d_LM = 0.5*sqrt(pow((CR_x_NED-CL_x_NED),2.0)+pow((CR_y_NED-CL_y_NED),2.0));  // half distance between left and right buoys
	a_L = sqrt(pow(d_LM,2.0)+pow(d_I,2.0));  // distance between left buoy and the approach point
	theta = atan(d_I/d_LM);  // angle created by d_I and d_LM

	x_I_CL = a_L*cos(theta);  // x-coord. of intermediate point wrt left buoy 
	y_I_CL = a_L*sin(theta);  // y-coord. of intermediate point wrt left buoy

	x_E_CL = a_L*cos(theta);  // x-coord. of exit point wrt left buoy 
	y_E_CL = -a_L*sin(theta);  // y-coord. of exit point wrt left buoy

	// calculate intermediate position wrt global
	s_M = (CR_y_NED-CL_y_NED)/(CR_x_NED-CL_x_NED);  // slope of of line from CL to CR 
	alpha = atan2((CR_y_NED-CL_y_NED),(CR_x_NED-CL_x_NED));  // angle of frame CL wrt global frame

	ROS_INFO("ALPHA: %4.2f    THETA: %4.2f\n", alpha, theta);

	I_x = cos(alpha)*x_I_CL - sin(alpha)*y_I_CL + CL_x_NED;  // x-coord. of intermediate point wrt global
	I_y = sin(alpha)*x_I_CL + cos(alpha)*y_I_CL + CL_y_NED;  // y-coord. of intermediate point wrt global

	E_x = cos(alpha)*x_E_CL - sin(alpha)*y_E_CL + CL_x_NED;  // x-coord. of exit point wrt global
	E_y = sin(alpha)*x_E_CL + cos(alpha)*y_E_CL + CL_y_NED;  // y-coord. of exit point wrt global

	x_goal_poses[0] = I_x;
	y_goal_poses[0] = I_y;
	psi_goal_poses[0] = alpha - PI/2;
	x_goal_poses[1] = M_x;
	y_goal_poses[1] = M_y;
	psi_goal_poses[1] = alpha - PI/2;
	x_goal_poses[2] = E_x;
	y_goal_poses[2] = E_y;
	psi_goal_poses[2] = alpha - PI/2;

	// display calculated goal points to reach
	ROS_INFO("~~~Desired points~~~");
	ROS_INFO("I_x: %f", x_goal_poses[0]);
	ROS_INFO("I_y: %f", y_goal_poses[0]);
	ROS_INFO("M_x: %f", x_goal_poses[1]);
	ROS_INFO("M_y: %f", y_goal_poses[1]);
	ROS_INFO("E_x: %f", x_goal_poses[2]);
	ROS_INFO("E_y: %f\n", y_goal_poses[2]);
	ROS_INFO("________^^^^^^^^^^^  {PP}  ^^^^^^^^^^^___________________|\n");

	point = 0;
	goal_poses_quantity = 3;
	calculations_done = true;
}  // END OF calculate_buoy_waypoints()

// END OF FUNCTIONS FOR VISIONS PATH PLANNER
// THIS FUNCTION: Generates the figure 8 pattern
// ACCEPTS: (VOID) Uses global North and South points
// RETURNS: (VOID) Populates array of poses
// =============================================================================
void calculate_fig8_path()
{
	r = 3;  // [m] radius of circles

	// first calculate the midpoint
	x_mid = (x_North + x_South)/2.0;
	y_mid = (y_North + y_South)/2.0;

	// now fill array of poses
	point = 0;

	// first pose - mid point
	x_goal_poses[point] = x_mid;
	y_goal_poses[point] = y_mid;
	psi_goal_poses[point] = 0.0;
	point++;

	// next 5 poses around North point
	// second pose - left
	x_goal_poses[point] = x_North;
	y_goal_poses[point] = y_North-r;
	psi_goal_poses[point] = 0.0;
	point++;
	// third pose
	x_goal_poses[point] = x_North+r*0.707107;
	y_goal_poses[point] = y_North-r*0.707107;
	psi_goal_poses[point] = PI/4.0;
	point++;
	// fourth pose - top
	x_goal_poses[point] = x_North+r;
	y_goal_poses[point] = y_North;
	psi_goal_poses[point] = PI/2.0;
	point++;
	// fifth pose
	x_goal_poses[point] = x_North+r*0.707107;
	y_goal_poses[point] = y_North+r*0.707107;
	psi_goal_poses[point] = 3.0*PI/4.0;
	point++;
	// sixth pose - right
	x_goal_poses[point] = x_North;
	y_goal_poses[point] = y_North+r;
	psi_goal_poses[point] = PI;
	point++;

	// seventh pose - mid point
	// x_goal_poses[point] = x_mid;
	// y_goal_poses[point] = y_mid;
	// psi_goal_poses[point] = PI;
	// point++;

	// next 5 poses around North point
	// eighth pose - left
	x_goal_poses[point] = x_South;
	y_goal_poses[point] = y_South-r;
	psi_goal_poses[point] = PI;
	point++;
	// ninth pose
	x_goal_poses[point] = x_South-r*0.707107;
	y_goal_poses[point] = y_South-r*0.707107;
	psi_goal_poses[point] = 3.0*PI/4.0;
	point++;
	// tenth pose - bottom
	x_goal_poses[point] = x_South-r;
	y_goal_poses[point] = y_South;
	psi_goal_poses[point] = PI/2.0;
	point++;
	// eleventh pose
	x_goal_poses[point] = x_South-r*0.707107;
	y_goal_poses[point] = y_South+r*0.707107;
	psi_goal_poses[point] = PI/4.0;
	point++;
	// twelfth pose - right
	x_goal_poses[point] = x_South;
	y_goal_poses[point] = y_South+r;
	psi_goal_poses[point] = 0.0;
	point++;

	// thirteenth pose - mid point
	x_goal_poses[point] = x_mid;
	y_goal_poses[point] = y_mid;
	psi_goal_poses[point] = 1.57;
	point++;

	goal_poses_quantity = point;
	point = 0;  // reset to start by feeding first point
	calculations_done = true;
} // end of calculate_fig8_path()

// THIS FUNCTION: Generates T2 pattern
// ACCEPTS: (VOID) Uses local NED hardcoded poses collected through manual nav
// RETURNS: (VOID) Populates array of poses
// =============================================================================
void calculate_t2_path()
{
	r = 5;  // [m] radius of circles

	// now fill array of poses
	point = 0;

	// first pose - entrance point
	x_goal_poses[point] = x_t2_start_end;
	y_goal_poses[point] = y_t2_start_end;
	psi_goal_poses[point] = psi_t2_start_end;
	point++;

	// next 5 poses around black buoy
	// second pose - left
	x_goal_poses[point] = x_t2_black_buoy;
	y_goal_poses[point] = y_t2_black_buoy-r;
	psi_goal_poses[point] = 0.0;
	point++;
	// third pose
	x_goal_poses[point] = x_t2_black_buoy+r*0.707107;
	y_goal_poses[point] = y_t2_black_buoy-r*0.707107;
	psi_goal_poses[point] = PI/4.0;
	point++;
	// fourth pose - top
	x_goal_poses[point] = x_t2_black_buoy+r;
	y_goal_poses[point] = y_t2_black_buoy;
	psi_goal_poses[point] = PI/2.0;
	point++;
	// fifth pose
	x_goal_poses[point] = x_t2_black_buoy+r*0.707107;
	y_goal_poses[point] = y_t2_black_buoy+r*0.707107;
	psi_goal_poses[point] = 3.0*PI/4.0;
	point++;
	// sixth pose - right
	x_goal_poses[point] = x_t2_black_buoy;
	y_goal_poses[point] = y_t2_black_buoy+r;
	psi_goal_poses[point] = PI;
	point++;

	// seventh pose - exit point
	x_goal_poses[point] = x_t2_start_end;
	y_goal_poses[point] = y_t2_start_end;
	psi_goal_poses[point] = psi_t2_start_end + PI;
	point++;

	while ((psi_goal_poses[point] < -PI) || (psi_goal_poses[point] > PI))
	{
		if (psi_goal_poses[point] < -PI)
		{
			psi_goal_poses[point] += 2.0*PI;
		}
		if (psi_goal_poses[point] > PI)
		{
			psi_goal_poses[point] -= 2.0*PI;
		}
	}

	goal_poses_quantity = point;
	point = 0;  // reset to start by feeding first point
	calculations_done = true;

	for (int i=0;i<goal_poses_quantity;i++)
	{
		ROS_INFO("PP: x_goal- %4.2f    y_goal- %4.2f    psi_goal- %4.2f", x_goal_poses[i], y_goal_poses[i], psi_goal_poses[i]);
	}
} // end of calculate_t2_path()

// RobotX TASK 6: Docking
// THIS FUNCTION: Generates the waypoints to manuever into the correct docking bay
// ACCEPTS: (VOID)
// RETURNS: (VOID)
//=============================================================================================================
/* void Task6_Docking()
{
	int point = 0;
	int d = 10;  // [m] Distance of point outside the Dock
	ROS_INFO("PATH PLANNER: X Red Dock = %f",x_Red_Dock);
	if (Dock_Name == 1)
	{

		//Going Forward
		x_goal_poses[point] = x_usv_NED + d*cos(psi_usv_NED);	
		y_goal_poses[point] = y_usv_NED + d*sin(psi_usv_NED);
		psi_goal_poses[point] = psi_usv_NED;
		point++;

		//Positioning right before the DOCK
		x_goal_poses[point] = x_Red_Dock + d*cos(psi_Red_Dock);
		y_goal_poses[point] = y_Red_Dock + d*cos(psi_Red_Dock);
		psi_goal_poses[point] = psi_usv_NED + PI/2;
		point++;

		//Final Docking Position
		x_goal_poses[point] = x_Red_Dock;
		y_goal_poses[point] = y_Red_Dock;
		psi_goal_poses[point] = psi_Red_Dock;
		point++;
	}

	else if (Dock_Name == 2)
	{
		//Going Forward
		x_goal_poses[point] = x_usv_NED + d*cos(psi_usv_NED);
		y_goal_poses[point] = y_usv_NED + d*cos(psi_usv_NED);
		psi_goal_poses[point] = psi_usv_NED;
		point++;

		//Positioning right before the DOCK
		x_goal_poses[point] = x_Green_Dock + d*cos(psi_Green_Dock);
		y_goal_poses[point] = y_Green_Dock + d*cos(psi_Green_Dock);
		psi_goal_poses[point] = psi_usv_NED + PI/2;
		point++;

		//Final Docking Position
		x_goal_poses[point] = x_Green_Dock;
		y_goal_poses[point] = y_Green_Dock;
		psi_goal_poses[point] = psi_Green_Dock;
		point++;
	}

	else if (Dock_Name == 3)
	{
		//Going Forward
		x_goal_poses[point] = x_usv_NED + d*cos(psi_usv_NED);
		y_goal_poses[point] = y_usv_NED + d*cos(psi_usv_NED);
		psi_goal_poses[point] = psi_usv_NED;
		point++;

		//Positioning right before the DOCK
		x_goal_poses[point] = x_Blue_Dock + d*cos(psi_Blue_Dock);
		y_goal_poses[point] = y_Blue_Dock + d*cos(psi_Blue_Dock);
		psi_goal_poses[point] = psi_usv_NED + PI/2;
		point++;

		//Final Docking Position
		x_goal_poses[point] = x_Blue_Dock;
		y_goal_poses[point] = y_Blue_Dock;
		psi_goal_poses[point] = psi_Blue_Dock;
		point++;
	}
	
} */
//.............................................................................................................END OF Functions...............................................................................................................

//..................................................................................................................Main Program..................................................................................................................
int main(int argc, char **argv)
{
	// names the program for visual purposes
	ros::init(argc, argv, "path_planner");

	// NodeHandles
	ros::NodeHandle nh1, nh2, nh3, nh4, nh5, nh6, nh7, nh8, nh9, nh10, nh12;

	// Subscribers
	// from mission_control
	ros::Subscriber MC_na_state_sub = nh1.subscribe("MC_na_state", 1, MC_na_state_update);
	ros::Subscriber MC_pp_state_sub = nh2.subscribe("MC_pp_state", 1, MC_pp_state_update);
	ros::Subscriber MC_ps_state_sub = nh3.subscribe("MC_ps_state", 1, MC_ps_state_update);
	ros::Subscriber MC_pa_state_sub = nh4.subscribe("MC_pa_state", 1, MC_pa_state_update);
	ros::Subscriber MC_a_state_sub = nh5.subscribe("MC_a_state", 1, MC_a_state_update);
	// from navigation_array
	ros::Subscriber NA_nav_ned_sub = nh6.subscribe("NA_nav_ned", 1, NA_nav_ned_update);  // Obtains the USV pose in local NED
	// from perception_array
	ros::Subscriber PA_NED_buoys_sub = nh7.subscribe("PA_NED_buoys", 1, NED_buoys_update);  // current buoy IDs with respective locations for planner to use to generate path
	// from coordinate_converter
	ros::Subscriber CC_goal_poses_ned_sub = nh8.subscribe("CC_goal_poses_ned", 1, CC_goal_poses_ned_update);  // goal poses converted to NED
	ros::Subscriber CC_animals_ned_sub = nh9.subscribe("CC_animals_ned", 1, CC_animals_ned_update);  // goal animal locations converted to NED
	// from acoustics
	ros::Subscriber A_source_location_sub = nh10.subscribe("A_source_location", 1, A_source_location_update);  // acoustic pinger converted to NED

	// Publishers
	// to mission_control
	PP_initialization_state_pub = nh12.advertise<std_msgs::Bool>("PP_initialization_state", 1);  // publisher for state of initialization
	PP_params_GOT_pub = nh12.advertise<std_msgs::Bool>("PP_params_GOT", 1);  // publisher for whether or not the params have been GOT to let mission_control know if propulsion system should be started
	// to coordinate_converter
	PP_USV_pose_update_state_pub = nh12.advertise<std_msgs::Bool>("PP_USV_pose_update_state", 1);  // publisher for whether USV NED pose has been updated or not
	// to propulsion_system
	PP_propulsion_system_topic_pub = nh12.advertise<amore::propulsion_system>("PP_propulsion_system_topic", 1);  // goal pose (x,y,psi) to reach in local NED frame and current USV pose to propulsion_system

	// Timers ///////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Initialize simulation time
	ros::Time::init();

	// Initialize global variables
	PP_USV_pose_update_state_msg.data = false;  // false means NED USV pose has not been updated 
	PP_initialization_state_msg.data = false;

	// sets the frequency for which the program loops at 10 = 1/10 second
	ros::Rate loop_rate(10);  // [Hz] perception_array: 30, navigation_array: 20, mission_control: 10

	// ros::ok() will stop when the user inputs Ctrl+C
	while(ros::ok())
	{
		//	0 = On standby
		//  1 = Dynamic navigation demonstration
		//  2 = Entrance and Exit gates
		//  3 = Follow the path
		//  4 = Wildlife Encounter - React and Report
		//  5 = Scan the code
		//  6 = Detect and dock
		//  7 = Find and Fling
		//  8 = UAV replenishment
		//	11 = VRX1: Station-Keeping
		//	12 = VRX2: Wayfinding
		//	14 = VRX4: Wildlife Encounter and Avoid
		//	15 = VRX5: Channel Navigation, Acoustic Beacon Localization and Obstacle Avoidance
		//	16 = VRX6: Scan and Dock and Deliver
		switch(PP_state)
		{
			case 0:  // On standby
				// reset all variables to be used for next run
				e_xy_allowed = default_position_error_allowed;  // default positional error tolerance threshold
				e_psi_allowed = default_heading_error_allowed;  // default heading error tolerance threshold
				CC_goal_recieved = false;  // false means NED goal poses have not been acquired from coordinate_converter
				E_reached = false;  // means end of path reached (E_reached) is not true (false)
				E_never_reached = true;  // means the last point has never been reached
				calculations_done = false;  // FOR CALCULATING TASK GOALS ONLY ONCE
				buoy_positions_recieved = false;  // FOR BUOY NAVIGATION
				DnD_path_planner = 1;  // DnD_path_planner reset
				FtP_path_planner = 1;  // FtP_path_planner reset
				params_GOT = false;  // false means the task parameters have not yet been acquired
				params_updated = false;  // reset that the parameters have been updated
				
				break;

			/* case 1:  // Dynamic navigation demonstration
				ROS_INFO("PATH_PLANNER: DnD State %i\n", DnD_path_planner);
				//  1 = Start point search and approach
				//  2 = Search for next buoy pair
				//  3 = Going to calculated approach and mid points of entrance buoy pair
				//  4 = Start search and approach for exit buoy pair while continuing on current path
				//	5 = Exit point has been reached and task is complete
				switch(DnD_path_planner)
				{
					case 1:  // Start point search and approach
						e_xy_allowed = 1.0;  // [m] positional error tolerance threshold
						e_psi_allowed =  0.03;  // [rad] (about 2 degrees) heading error tolerance threshold
						if ((PP_USV_pose_update_state_msg.data) && (!propulsion_system_topic_published))  // if the USV pose updated but the propulsion_system_topic not published
						{
							set_goal_pose(x_DnD_start, y_DnD_start, psi_DnD_start);  // set the goal pose to the start pose of the dynamic navigation demonstration
							propulsion_system_topic_publish();  // update the propulsion_system topic to go to the goal pose
							calculate_pose_errors();
							display_pose_errors();
							display_usv_and_goal_pose();
							if ((e_xy < e_xy_allowed) && ((float)abs(e_psi) < e_psi_allowed))
							{
								DnD_path_planner = 2;  // Search for entrance buoy pair
								ROS_INFO("PATH_PLANNER:-----Start point reached-----\n");
							}
						}
						break;
					case 2:  // Search for entrance buoy pair
						if ((PP_USV_pose_update_state_msg.data) && (!propulsion_system_topic_published))  // if the USV pose updated but the propulsion_system_topic not published
						{
							propulsion_system_topic_publish();  // update the propulsion_system topic to go to the goal pose
							if ((buoy_positions_recieved) && (!calculations_done))  // if buoy positions wrt the USV have been recieved but the path through the buoy pair has not been (re)calculated
							{
								calculate_buoy_waypoints();
							}
							if (calculations_done)
							{
								DnD_path_planner = 3;  // Going to calculated approach and mid points of entrance buoy pair
								ROS_INFO("PATH_PLANNER:-----Entrance gate approach calculated-----\n");
								// reset for next search
								buoy_positions_recieved = false;
								calculations_done = false;
							}
						}
						break;
					case 3:  // Going to calculated approach and mid points of entrance buoy pair
						e_xy_allowed = 1.0;  // [m] positional error tolerance threshold
						e_psi_allowed =  0.03;  // [rad] (about 2 degrees) heading error tolerance threshold
						if ((PP_USV_pose_update_state_msg.data) && (!propulsion_system_topic_published))  // if the USV pose updated but the propulsion_system_topic has not been published
						{
							set_goal_pose(x_goal_poses[point], y_goal_poses[point], psi_goal_poses[point]);  // set the goal pose to the current goal pose in the array of goal poses
							calculate_pose_errors();
							display_pose_errors();
							// dependent on whether or not USV is within pose error tolerance of next goal feed or skip next goal
							if ((e_xy < e_xy_allowed) && ((float)abs(e_psi) < e_psi_allowed) && (!M_reached))  // if within pose tolerance and mid-point not reached 
							{
								point += 1;  // increment the point place keeper
								// UPDATE USER
								ROS_INFO("PATH_PLANNER:-----POSE REACHED-----");
								ROS_INFO("PATH_PLANNER:        %i  of  %i", point, goal_poses_quantity);
								ROS_INFO("PATH_PLANNER:-----POSE REACHED-----\n");
								if (point == (goal_poses_quantity-1))  // if the mid-point has been reached
								{
								  M_reached = true;  // true means the mid-point has been reached
								}
							}
							else if (M_reached)  // if mid point has been reached
							{
								go_forward(20);  // tell the USV to go 30 meters forward
								DnD_path_planner = 4;  // Start search and approach for exit buoy pair while continuing on current updated path
								ROS_INFO("PATH_PLANNER:-----Mid-point of entrance gates reached-----\n");
								// reset for next time doing task
								M_reached = false;  // false means the mid point has not been reached
							}
							// decide whether to feed proposed goal pose or overide with intermediate point that is en route to proposed goal pose
							else if (e_xy > max_next_position_distance)  // if the position error is off by more than 4.0 [m] overide
							{
								// make next goal pose for propulsion_system have a heading en route to next goal pose from the tasks with a position on a line following that heading that is 4.0 [m] from the USV
								psi_goal_pose = atan2(e_y,e_x);  // [radians] atan2() returns between -PI and PI
								x_goal_pose = x_usv_NED + cos(psi_goal_pose)*max_next_position_distance;
								y_goal_pose = y_usv_NED + sin(psi_goal_pose)*max_next_position_distance;
								propulsion_system_topic_publish();  // update the propulsion_system topic to go to the intermediate goal pose
							}
							else if (e_xy <= max_next_position_distance)
							{
								propulsion_system_topic_publish();  // update the propulsion_system topic to go to the goal pose
							}
							else
							{
								ROS_INFO("PATH_PLANNER: HMMMMMMMMMMMMMMMMMMMMM look into this?\n");
							}
							display_usv_and_goal_pose();
						}  // END OF if ((PP_USV_pose_update_state_msg.data) && (!propulsion_system_topic_published))
						break;
					case 4:  // Start search and approach for exit buoy pair while continuing on current path
						e_xy_allowed = 1.5;  // [m] positional error tolerance threshold
						e_psi_allowed =  0.785;  // [rad] (about 45 degrees) heading error tolerance threshold
						if ((PP_USV_pose_update_state_msg.data) && (!propulsion_system_topic_published))  // if the USV pose updated but the propulsion_system_topic has not been published
						{
							if ((buoy_positions_recieved) && (!calculations_done))  // if buoy positions wrt the USV have been recieved but the path through the buoy pair has not been (re)calculated
							{
								calculate_buoy_waypoints();
								// reset for next search
								buoy_positions_recieved = false;
								calculations_done = false;
								ROS_INFO("PATH_PLANNER:-----Path through exit gate recalculated-----");
							}
							set_goal_pose(x_goal_poses[point], y_goal_poses[point], psi_goal_poses[point]);  // set the goal pose to the current goal pose in the array of goal poses
							calculate_pose_errors();
							display_pose_errors();
							// dependent on whether or not USV is within pose error tolerance of next goal feed or skip next goal
							if ((e_xy < e_xy_allowed) && ((float)abs(e_psi) < e_psi_allowed) && (!E_reached))  // if within pose tolerance and end point not reached 
							{
								point += 1;  // increment the point place keeper
								// UPDATE USER
								ROS_INFO("PATH_PLANNER:-----POSE REACHED-----");
								ROS_INFO("PATH_PLANNER:        %i  of  %i", point, goal_poses_quantity);
								ROS_INFO("PATH_PLANNER:-----POSE REACHED-----\n");
								if (point == goal_poses_quantity)  // if the exit point has been reached
								{
								  E_reached = true;  // true means the exit point has been reached
								}
							}
							else if (E_reached)  // if exit point has been reached
							{
								set_goal_pose(x_usv_NED, y_usv_NED, psi_usv_NED);  // have the USV station-keep until next task
								DnD_path_planner = 5;  // Exit point has been reached and task is complete
								ROS_INFO("PATH_PLANNER:-----End point of exit gates reached-----");
								// reset for next time doing task
								E_reached = false;  // false means the exit point has not been reached
								point -= 1;  // decrement point for feeding end point goal to station-keep until new task
							}
							// decide whether to feed proposed goal pose or overide with intermediate point that is en route to proposed goal pose
							else if (e_xy > max_next_position_distance)  // if the position error is off by more than 4.0 [m] overide
							{
								// make next goal pose for propulsion_system have a heading en route to next goal pose from the tasks with a position on a line following that heading that is 4.0 [m] from the USV
								psi_goal_pose = atan2(e_y,e_x);  // [radians] atan2() returns between -PI and PI
								x_goal_pose = x_usv_NED + cos(psi_goal_pose)*max_next_position_distance;
								y_goal_pose = y_usv_NED + sin(psi_goal_pose)*max_next_position_distance;
								propulsion_system_topic_publish();  // update the propulsion_system topic to go to the intermediate goal pose
							}
							else if (e_xy <= max_next_position_distance)
							{
								propulsion_system_topic_publish();  // update the propulsion_system topic to go to the goal pose
							}
							else
							{
								ROS_INFO("PATH_PLANNER: HMMMMMMMMMMMMMMMMMMMMM look into this?\n");
							}
							display_usv_and_goal_pose();
						}  // END OF if ((PP_USV_pose_update_state_msg.data) && (!propulsion_system_topic_published))
						break;
					case 5:  // Exit point has been reached and task is complete
						if ((PP_USV_pose_update_state_msg.data) && (!propulsion_system_topic_published))  // if the USV pose updated but the propulsion_system_topic has not been published
						{
							propulsion_system_topic_publish();  // update the propulsion_system topic to go to the goal pose
						}
						break;
				}
				break; */
			case 1:  // Dynamic navigation demonstration
				ROS_INFO("PATH_PLANNER: DnD State %i\n", DnD_path_planner);
				//  1 = Start point search and approach
				//  2 = End point approach
				//	3 = Task is complete, station-keep
				switch(DnD_path_planner)
				{
					case 1:  // Publish point 35 meters ahead
						if (PP_USV_pose_update_state_msg.data)
						{
							go_forward(50);  // tell the USV to go 50 meters forward
							set_goal_pose(x_goal_poses[point], y_goal_poses[point], psi_goal_poses[point]);  // set the goal pose to the current goal pose in the array of goal poses
							DnD_path_planner = 2;  // End point approach
							ROS_INFO("PATH_PLANNER:-----Published point 35 meters ahead-----\n");
							// update tolerances for next step
							e_xy_allowed = 1.5;  // [m] positional error tolerance threshold
							e_psi_allowed =  0.785;  // [rad] (about 45 degrees) heading error tolerance threshold
						}
					case 2:  // End point approach
						drive_config = 2;  // differential drive wayfinding
						if ((PP_USV_pose_update_state_msg.data) && (!propulsion_system_topic_published))  // if the USV pose updated but the propulsion_system_topic has not been published
						{
							set_goal_pose(x_goal_poses[point], y_goal_poses[point], psi_goal_poses[point]);  // set the goal pose to the current goal pose in the array of goal poses
							calculate_pose_errors();
							display_pose_errors();
							// dependent on whether or not USV is within pose error tolerance of next goal feed or skip next goal
							if ((e_xy < e_xy_allowed) && ((float)abs(e_psi) < e_psi_allowed))  // if within pose tolerance, end point reached 
							{
								E_reached = true;  // true means the exit point has been reached
							}
							// decide whether to feed proposed goal pose or overide with intermediate point that is en route to proposed goal pose
							else if (e_xy > max_next_position_distance)  // if the position error is off by more than 4.0 [m] overide
							{
								// make next goal pose for propulsion_system have a heading en route to next goal pose from the tasks with a position on a line following that heading that is 4.0 [m] from the USV
								psi_goal_pose = atan2(e_y,e_x);  // [radians] atan2() returns between -PI and PI
								x_goal_pose = x_usv_NED + cos(psi_goal_pose)*max_next_position_distance;
								y_goal_pose = y_usv_NED + sin(psi_goal_pose)*max_next_position_distance;
								propulsion_system_topic_publish();  // update the propulsion_system topic to go to the intermediate goal pose
							}
							else if (e_xy <= max_next_position_distance)
							{
								propulsion_system_topic_publish();  // update the propulsion_system topic to go to the goal pose
							}
							else
							{
								ROS_INFO("PATH_PLANNER: HMMMMMMMMMMMMMMMMMMMMM look into this?\n");
							}
							if (E_reached)  // if exit point has been reached
							{
								set_goal_pose(x_usv_NED, y_usv_NED, psi_usv_NED);  // have the USV station-keep until next task
								DnD_path_planner = 3;  // Task is complete, station-keep
								ROS_INFO("PATH_PLANNER:-----End point reached-----\n");
								// reset for next time doing task
								E_reached = false;  // false means the exit point has not been reached
							}
							display_usv_and_goal_pose();
						}  // END OF if ((PP_USV_pose_update_state_msg.data) && (!propulsion_system_topic_published))
						break;
					case 3:  // Task is complete, station-keep
						drive_config = 1;  // station-keep
						if ((PP_USV_pose_update_state_msg.data) && (!propulsion_system_topic_published))  // if the USV pose updated but the propulsion_system_topic has not been published
						{
							propulsion_system_topic_publish();  // update the propulsion_system topic to go to the goal pose
						}
						break;
				}
				break;

			case 2:  // Entrance and Exit gates
				if (params_GOT)  // if the params have been got
				{
					if (calculations_done)
					{
						drive_config = 1;  // this should be 2 for differential wayfinding
						if ((PP_USV_pose_update_state_msg.data) && (!propulsion_system_topic_published) && (!E_reached))  // if the USV pose updated but the propulsion_system_topic not published
						{
							position_tolerance = default_position_error_allowed;

							set_goal_pose(x_goal_poses[point], y_goal_poses[point], psi_goal_poses[point]);  // set the goal pose to the station-keeping goal location
							calculate_pose_errors();
							//display_pose_errors();

							// dependent on whether or not USV is within pose error tolerance of next goal feed or skip next goal
							if ((e_xy < position_tolerance) && (!E_reached))  // if within position tolerance and not finished with current waypoints
							{
								point += 1;
								ROS_INFO("Point %i of %i reached. --MC", point, goal_poses_quantity);
								if (point == goal_poses_quantity)
								{
									point -= 1;
									E_reached = true;
									ROS_INFO("End point has been reached. --MC\n");
								}
							}
							// decide whether to feed proposed goal pose or overide with intermediate point that is en route to proposed goal pose
							else if (e_xy > max_next_position_distance)  // if the position error is off by more than 4.0 [m] overide
							{
								// make next goal pose for propulsion_system have a heading en route to next goal pose from the tasks with a position on a line following that heading that is 4.0 [m] from the USV
								psi_goal_pose = atan2(e_y,e_x);  // [radians] atan2() returns between -PI and PI
								x_goal_pose = x_usv_NED + cos(psi_goal_pose)*max_next_position_distance;
								y_goal_pose = y_usv_NED + sin(psi_goal_pose)*max_next_position_distance;
								propulsion_system_topic_publish();  // update the propulsion_system topic to go to the intermediate goal pose
							}
							else if (e_xy <= max_next_position_distance)
							{
								propulsion_system_topic_publish();  // update the propulsion_system topic to go to the goal pose
							}
							else if (E_reached)
							{
								drive_config = 1;  // station-keep
								propulsion_system_topic_publish();  // update the propulsion_system topic
							}
							else
							{
								ROS_INFO("PATH_PLANNER: HMMMMMMMMMMMMMMMMMMMMM look into this?\n");
							}
							//display_usv_and_goal_pose();
						} // if ((PP_USV_pose_update_state_msg.data) && (!propulsion_system_topic_published))
					}
					else
					{
						calculate_t2_path();  // this function will generate the updated array of poses for task 2
						E_reached = false;  // ensure resetted
					}
				}
				else 
				{
					ROS_INFO("PATH_PLANNER: WAITING TO RECIEVE THE PARAMETERS\n");
				}
				break;

			case 3:  // Follow the path
				ROS_INFO("PATH_PLANNER: FtP State %i\n", FtP_path_planner);
				//  1 = Start point search and approach
				//  2 = Search for next buoy pair
				//  3 = Going to calculated approach and mid points of entrance buoy pair
				//  4 = Start search and approach for exit buoy pair while continuing on current path
				//	5 = Exit point has been reached and task is complete
				switch(FtP_path_planner)
				{
					case 1:  // Start point search and approach
						e_xy_allowed = 1.0;  // [m] positional error tolerance threshold
						e_psi_allowed =  0.03;  // [rad] (about 2 degrees) heading error tolerance threshold
						if ((PP_USV_pose_update_state_msg.data) && (!propulsion_system_topic_published))  // if the USV pose updated but the propulsion_system_topic not published
						{
							set_goal_pose(x_FtP_start, y_FtP_start, psi_FtP_start);  // set the goal pose to the start pose of the dynamic navigation demonstration
							propulsion_system_topic_publish();  // update the propulsion_system topic to go to the goal pose
							calculate_pose_errors();
							display_pose_errors();
							display_usv_and_goal_pose();
							if ((e_xy < e_xy_allowed) && ((float)abs(e_psi) < e_psi_allowed))
							{
								FtP_path_planner = 2;  // Search for entrance buoy pair
								ROS_INFO("PATH_PLANNER:-----Start point reached-----\n");
							}
						}
						break;
					case 2:  // Search for entrance buoy pair
						if ((PP_USV_pose_update_state_msg.data) && (!propulsion_system_topic_published))  // if the USV pose updated but the propulsion_system_topic not published
						{
							propulsion_system_topic_publish();  // update the propulsion_system topic to go to the goal pose
							if ((buoy_positions_recieved) && (!calculations_done))  // if buoy positions wrt the USV have been recieved but the path through the buoy pair has not been (re)calculated
							{
								calculate_buoy_waypoints();
							}
							if (calculations_done)
							{
								FtP_path_planner = 3;  // Going to calculated approach and mid points of entrance buoy pair
								ROS_INFO("PATH_PLANNER:-----Entrance gate approach calculated-----\n");
								// reset for next search
								buoy_positions_recieved = false;
								calculations_done = false;
							}
						}
						break;
					case 3:  // Going to calculated approach and mid points of entrance buoy pair
						e_xy_allowed = 1.0;  // [m] positional error tolerance threshold
						e_psi_allowed =  0.03;  // [rad] (about 2 degrees) heading error tolerance threshold
						if ((PP_USV_pose_update_state_msg.data) && (!propulsion_system_topic_published))  // if the USV pose updated but the propulsion_system_topic has not been published
						{
							set_goal_pose(x_goal_poses[point], y_goal_poses[point], psi_goal_poses[point]);  // set the goal pose to the current goal pose in the array of goal poses
							calculate_pose_errors();
							display_pose_errors();
							// dependent on whether or not USV is within pose error tolerance of next goal feed or skip next goal
							if ((e_xy < e_xy_allowed) && ((float)abs(e_psi) < e_psi_allowed) && (!M_reached))  // if within pose tolerance and mid-point not reached 
							{
								point += 1;  // increment the point place keeper
								// UPDATE USER
								ROS_INFO("PATH_PLANNER:-----POSE REACHED-----");
								ROS_INFO("PATH_PLANNER:        %i  of  %i", point, goal_poses_quantity);
								ROS_INFO("PATH_PLANNER:-----POSE REACHED-----\n");
								if (point == (goal_poses_quantity-1))  // if the mid-point has been reached
								{
								  M_reached = true;  // true means the mid-point has been reached
								}
							}
							else if (M_reached)  // if mid point has been reached
							{
								go_forward(20);  // tell the USV to go 30 meters forward
								FtP_path_planner = 4;  // Start search and approach for exit buoy pair while continuing on current updated path
								ROS_INFO("PATH_PLANNER:-----Mid-point of entrance gates reached-----\n");
								// reset for next time doing task
								M_reached = false;  // false means the mid point has not been reached
							}
							// decide whether to feed proposed goal pose or overide with intermediate point that is en route to proposed goal pose
							else if (e_xy > max_next_position_distance)  // if the position error is off by more than 4.0 [m] overide
							{
								// make next goal pose for propulsion_system have a heading en route to next goal pose from the tasks with a position on a line following that heading that is 4.0 [m] from the USV
								psi_goal_pose = atan2(e_y,e_x);  // [radians] atan2() returns between -PI and PI
								x_goal_pose = x_usv_NED + cos(psi_goal_pose)*max_next_position_distance;
								y_goal_pose = y_usv_NED + sin(psi_goal_pose)*max_next_position_distance;
								propulsion_system_topic_publish();  // update the propulsion_system topic to go to the intermediate goal pose
							}
							else if (e_xy <= max_next_position_distance)
							{
								propulsion_system_topic_publish();  // update the propulsion_system topic to go to the goal pose
							}
							else
							{
								ROS_INFO("PATH_PLANNER: HMMMMMMMMMMMMMMMMMMMMM look into this?\n");
							}
							display_usv_and_goal_pose();
						}  // END OF if ((PP_USV_pose_update_state_msg.data) && (!propulsion_system_topic_published))
						break;
					case 4:  // Start search and approach for exit buoy pair while continuing on current path
						e_xy_allowed = 1.5;  // [m] positional error tolerance threshold
						e_psi_allowed =  0.785;  // [rad] (about 45 degrees) heading error tolerance threshold
						if ((PP_USV_pose_update_state_msg.data) && (!propulsion_system_topic_published))  // if the USV pose updated but the propulsion_system_topic has not been published
						{
							if ((buoy_positions_recieved) && (!calculations_done))  // if buoy positions wrt the USV have been recieved but the path through the buoy pair has not been (re)calculated
							{
								calculate_buoy_waypoints();
								// reset for next search
								buoy_positions_recieved = false;
								calculations_done = false;
								ROS_INFO("PATH_PLANNER:-----Path through exit gate recalculated-----");
							}
							set_goal_pose(x_goal_poses[point], y_goal_poses[point], psi_goal_poses[point]);  // set the goal pose to the current goal pose in the array of goal poses
							calculate_pose_errors();
							display_pose_errors();
							// dependent on whether or not USV is within pose error tolerance of next goal feed or skip next goal
							if ((e_xy < e_xy_allowed) && ((float)abs(e_psi) < e_psi_allowed) && (!E_reached))  // if within pose tolerance and end point not reached 
							{
								point += 1;  // increment the point place keeper
								// UPDATE USER
								ROS_INFO("PATH_PLANNER:-----POSE REACHED-----");
								ROS_INFO("PATH_PLANNER:        %i  of  %i", point, goal_poses_quantity);
								ROS_INFO("PATH_PLANNER:-----POSE REACHED-----\n");
								if (point == goal_poses_quantity)  // if the exit point has been reached
								{
								  E_reached = true;  // true means the exit point has been reached
								}
							}
							else if (E_reached)  // if exit point has been reached
							{
								set_goal_pose(x_usv_NED, y_usv_NED, psi_usv_NED);  // have the USV station-keep until next task
								FtP_path_planner = 5;  // Exit point has been reached and task is complete
								ROS_INFO("PATH_PLANNER:-----End point of exit gates reached-----");
								// reset for next time doing task
								E_reached = false;  // false means the exit point has not been reached
								point -= 1;  // decrement point for feeding end point goal to station-keep until new task
							}
							// decide whether to feed proposed goal pose or overide with intermediate point that is en route to proposed goal pose
							else if (e_xy > max_next_position_distance)  // if the position error is off by more than 4.0 [m] overide
							{
								// make next goal pose for propulsion_system have a heading en route to next goal pose from the tasks with a position on a line following that heading that is 4.0 [m] from the USV
								psi_goal_pose = atan2(e_y,e_x);  // [radians] atan2() returns between -PI and PI
								x_goal_pose = x_usv_NED + cos(psi_goal_pose)*max_next_position_distance;
								y_goal_pose = y_usv_NED + sin(psi_goal_pose)*max_next_position_distance;
								propulsion_system_topic_publish();  // update the propulsion_system topic to go to the intermediate goal pose
							}
							else if (e_xy <= max_next_position_distance)
							{
								propulsion_system_topic_publish();  // update the propulsion_system topic to go to the goal pose
							}
							else
							{
								ROS_INFO("PATH_PLANNER: HMMMMMMMMMMMMMMMMMMMMM look into this?\n");
							}
							display_usv_and_goal_pose();
						}  // END OF if ((PP_USV_pose_update_state_msg.data) && (!propulsion_system_topic_published))
						break;
					case 5:  // Exit point has been reached and task is complete
						if ((PP_USV_pose_update_state_msg.data) && (!propulsion_system_topic_published))  // if the USV pose updated but the propulsion_system_topic has not been published
						{
							propulsion_system_topic_publish();  // update the propulsion_system topic to go to the goal pose
						}
						break;
				}
				break;

			/* case 3:  // Follow the path
				ROS_INFO("PATH_PLANNER: FtP State %i\n", FtP_path_planner);
				//  1 = Start point search and approach
				//  2 = Search for entrance buoy pair
				//  3 = Going to calculated approach and mid points of entrance buoy pair
				//	4 = Exit point has been reached and task is complete
				switch(FtP_path_planner)
				{
					case 1:  // Start point search and approach
						e_xy_allowed = 1.0;  // [m] positional error tolerance threshold
						e_psi_allowed =  0.03;  // [rad] (about 2 degrees) heading error tolerance threshold
						if ((PP_USV_pose_update_state_msg.data) && (!propulsion_system_topic_published))  // if the USV pose updated but the propulsion_system_topic not published
						{
							set_goal_pose(x_FtP_start, y_FtP_start, psi_FtP_start);  // set the goal pose to the start pose of the follow the path task
							propulsion_system_topic_publish();  // update the propulsion_system topic to go to the goal pose
							calculate_pose_errors();
							display_pose_errors();
							display_usv_and_goal_pose();
							if ((e_xy < e_xy_allowed) && (e_psi < e_psi_allowed))
							{
								FtP_path_planner = 2;  // Search for entrance buoy pair
								ROS_INFO("PATH_PLANNER:-----Start point reached-----\n");
							}
						}
						break;
					case 2:  // Search for next buoy pair
						if ((PP_USV_pose_update_state_msg.data) && (!propulsion_system_topic_published))  // if the USV pose updated but the propulsion_system_topic not published
						{
							propulsion_system_topic_publish();  // update the propulsion_system topic to go to the goal pose
							if ((buoy_positions_recieved) && (!calculations_done))  // if buoy positions wrt the USV have been recieved but the path through the buoy pair has not been (re)calculated
							{
								calculate_buoy_waypoints();
							}
							if (calculations_done)
							{
								buoy_pairs_found += 1;  // increment the number of buoy pairs found
								FtP_path_planner = 3;  // Going to calculated approach, mid, and exit points of next buoy pair
								ROS_INFO("PATH_PLANNER:-----Gate approach calculated-----\n");
								// reset for next search
								buoy_positions_recieved = false;
								calculations_done = false;
							}
						}
						break;
					case 3:  // Going to calculated approach, mid, and exit points of next buoy pair
						e_xy_allowed = 1.0;  // [m] positional error tolerance threshold
						e_psi_allowed =  0.17;  // [rad] (about 10 degrees) heading error tolerance threshold
						if ((PP_USV_pose_update_state_msg.data) && (!propulsion_system_topic_published))  // if the USV pose updated but the propulsion_system_topic has not been published
						{
							set_goal_pose(x_goal_poses[point], y_goal_poses[point], psi_goal_poses[point]);  // set the goal pose to the current goal pose in the array of goal poses
							calculate_pose_errors();
							display_pose_errors();
							// dependent on whether or not USV is within pose error tolerance of next goal feed or skip next goal
							if ((e_xy < e_xy_allowed) && ((float)abs(e_psi) < e_psi_allowed) && (!E_reached))  // if within pose tolerance and end-point not reached 
							{
								point += 1;  // increment the point place keeper
								// UPDATE USER
								ROS_INFO("PATH_PLANNER:-----POSE REACHED-----");
								ROS_INFO("PATH_PLANNER:        %i  of  %i", point, goal_poses_quantity);
								ROS_INFO("PATH_PLANNER:-----POSE REACHED-----\n");
								if (point == goal_poses_quantity)  // if the end-point has been reached
								{
								  E_reached = true;  // true means the end-point has been reached
								}
							}
							else if (E_reached)  // if end point has been reached
							{
								//go_forward(20);  // tell the USV to go 30 meters forward
								if (buoy_pairs_found == number_of_buoy_pairs)
								{
									set_goal_pose(x_usv_NED, y_usv_NED, psi_usv_NED);  // have the USV station-keep until next task
									FtP_path_planner = 4;  // Exit point has been reached and task is complete
									ROS_INFO("PATH_PLANNER:-----End point of exit gates reached-----");
								}
								else
								{
									FtP_path_planner = 2;  // Search for next buoy pair
									ROS_INFO("PATH_PLANNER:-----End-point of gates reached, searching for next buoy pair-----\n");
								}
								// reset for next time doing task
								E_reached = false;  // false means the end point has not been reached
								point -= 1;  // decrement point for feeding end point to station-keep until new task
							}
							// decide whether to feed proposed goal pose or overide with intermediate point that is en route to proposed goal pose
							else if (e_xy > max_next_position_distance)  // if the position error is off by more than 4.0 [m] overide
							{
								// make next goal pose for propulsion_system have a heading en route to next goal pose from the tasks with a position on a line following that heading that is 4.0 [m] from the USV
								psi_goal_pose = atan2(e_y,e_x);  // [radians] atan2() returns between -PI and PI
								x_goal_pose = x_usv_NED + cos(psi_goal_pose)*max_next_position_distance;
								y_goal_pose = y_usv_NED + sin(psi_goal_pose)*max_next_position_distance;
								propulsion_system_topic_publish();  // update the propulsion_system topic to go to the intermediate goal pose
							}
							else if (e_xy <= max_next_position_distance)
							{
								propulsion_system_topic_publish();  // update the propulsion_system topic to go to the goal pose
							}
							else
							{
								ROS_INFO("PATH_PLANNER: HMMMMMMMMMMMMMMMMMMMMM look into this?\n");
							}
							display_usv_and_goal_pose();
						}  // END OF if ((PP_USV_pose_update_state_msg.data) && (!propulsion_system_topic_published))
						break;
					case 4:  // Exit point has been reached and task is complete
						if ((PP_USV_pose_update_state_msg.data) && (!propulsion_system_topic_published))  // if the USV pose updated but the propulsion_system_topic has not been published
						{
							propulsion_system_topic_publish();  // update the propulsion_system topic to go to the goal pose
						}
						break;
				}
				break; */

			case 4:  // Wildlife Encounter - React and Report
				
				break;

			case 5:  // Scan the code
				
				break;

			case 6:  // Detect and dock
				
				break;

			case 7:  // Find and Fling
				
				break;

			case 8:  // UAV replenishment
				
				break;

			case 11:  // VRX1: Station-Keeping
				if ((loop_count > (loop_goal_recieved)))  // && (CC_goal_recieved)   -- taken out for SYDNEY TESTING
				{
					if ((PP_USV_pose_update_state_msg.data) && (!propulsion_system_topic_published))  // if the USV pose updated but the propulsion_system_topic not published
					{
						set_goal_pose(x_goal_poses[point], y_goal_poses[point], psi_goal_poses[point]);  // set the goal pose to the station-keeping goal location
						propulsion_system_topic_publish();  // update the propulsion_system topic to go to the goal pose
						calculate_pose_errors();
						display_usv_and_goal_pose();
						// UPDATE USER OF POSE ERRORS WHEN CLOSE TO A GOAL POSE
						if (e_xy < 1.0)
						{
							display_pose_errors();
						}
					}
				}
				break;

			case 12:  // VRX2: Wayfinding
				if (E_never_reached)  // if the end pose has never been reached
				{
					e_xy_allowed = VRX2_position_error_allowed;  // default VRX2 positional error tolerance threshold
					e_psi_allowed = VRX2_heading_error_allowed;  // default VRX2 heading error tolerance threshold
				}
				if ((loop_count > (loop_goal_recieved)) && (CC_goal_recieved))  // if the goal poses have been recieved
				{
					if ((PP_USV_pose_update_state_msg.data) && (!propulsion_system_topic_published))  // if the USV pose updated but the propulsion_system_topic has not been published
					{
						// start by proposing that the goal pose to go to next is the next goal pose from the tasks
						set_goal_pose(x_goal_poses[point], y_goal_poses[point], psi_goal_poses[point]);  // set the goal pose to the current goal pose in the array of goal poses
						calculate_pose_errors();
						// UPDATE USER OF POSE ERRORS WHEN CLOSE TO A GOAL POSE
						if (e_xy < 1.0)
						{
							display_pose_errors();
						}
						// dependent on whether or not USV is within pose error tolerance of next goal feed or skip next goal
						if ((e_xy < e_xy_allowed) && ((float)abs(e_psi) < e_psi_allowed) && (!E_reached))  // if within pose tolerance and last pose not reached 
						{
							point += 1;  // increment the point place keeper
							// UPDATE USER
							ROS_INFO("PATH_PLANNER:-----POSE REACHED-----");
							ROS_INFO("PATH_PLANNER:        %i  of  %i", point, goal_poses_quantity);
							ROS_INFO("PATH_PLANNER:-----POSE REACHED-----\n");
							if (point == goal_poses_quantity)  // if all the goal poses have been reached
							{
							  E_reached = true;  // true means the last pose has been reached
							}
						}
						else if (E_reached)  // if last point has been reached
						{
							E_never_reached = false;  // false means the last pose has been reached before
							// use a smaller tolerance allowance next turn through
							e_xy_allowed /= 2;  // EXPERIMENT WITH THIS
							e_psi_allowed = e_psi_allowed - 0.1;  // EXPERIMENT WITH THIS
							point = 0;  // reset point place holder to go to points again
							E_reached = false;  // false means the last pose has not been reached
						}
						// decide whether to feed proposed goal pose or overide with intermediate point that is en route to proposed goal pose
						else if (e_xy > max_next_position_distance)  // if the position error is off by more than 4.0 [m] overide
						{
							// make next goal pose for propulsion_system have a heading en route to next goal pose from the tasks with a position on a line following that heading that is 4.0 [m] from the USV
							psi_goal_pose = atan2(e_y,e_x);  // [radians] atan2() returns between -PI and PI
							x_goal_pose = x_usv_NED + cos(psi_goal_pose)*max_next_position_distance;
							y_goal_pose = y_usv_NED + sin(psi_goal_pose)*max_next_position_distance;
							propulsion_system_topic_publish();  // update the propulsion_system topic to go to the intermediate goal pose
						}
						else if (e_xy <= max_next_position_distance)
						{
							propulsion_system_topic_publish();  // update the propulsion_system topic to go to the goal pose
						}
						else
						{
							ROS_INFO("PATH_PLANNER: HMMMMMMMMMMMMMMMMMMMMM look into this?\n");
						}
					}  // END OF if ((PP_USV_pose_update_state_msg.data) && (!propulsion_system_topic_published))
				}  // END OF if ((loop_count > (loop_goal_recieved)) && (CC_goal_recieved))
				break;

			case 14:  // VRX4: Wildlife Encounter and Avoid
				e_xy_allowed = VRX4_position_error_allowed;  // VRX4 positional error tolerance threshold
				e_psi_allowed = VRX4_heading_error_allowed;  // VRX4 heading error tolerance threshold

				if ((loop_count > (loop_goal_recieved)) && (CC_goal_recieved))  // if the goal poses have been recieved
				{
					if ((PP_USV_pose_update_state_msg.data) && (!propulsion_system_topic_published))  // if the USV pose updated but the propulsion_system_topic has not been published
					{
						if (calculations_done)  // if the updated array of poses to accomplish task has been calculated
						{
							// start by proposing that the goal pose to go to next is the next goal pose from the tasks
							set_goal_pose(x_goal_poses[point], y_goal_poses[point], psi_goal_poses[point]);  // set the goal pose to the current goal pose in the array of goal poses
							calculate_pose_errors();
							// UPDATE USER OF POSE ERRORS WHEN CLOSE TO A GOAL POSE
							if (e_xy < 1.0)
							{
								display_pose_errors();
							}
							// dependent on whether or not USV is within pose error tolerance of next goal feed or skip next goal
							if ((e_xy < e_xy_allowed) && ((float)abs(e_psi) < e_psi_allowed) && (!E_reached))  // if within pose tolerance and last pose not reached 
							{
								point += 1;  // increment the point place keeper
								// UPDATE USER
								ROS_INFO("PATH_PLANNER:-----POSE REACHED-----");
								ROS_INFO("PATH_PLANNER:        %i  of  %i", point, goal_poses_quantity);
								ROS_INFO("PATH_PLANNER:-----POSE REACHED-----\n");
								if (point == goal_poses_quantity)  // if all the goal poses have been reached
								{
								  E_reached = true;  // true means the last pose has been reached
								}
							}
							else if (E_reached)  // if last point has been reached
							{
								ROS_INFO("PATH_PLANNER: End pose has been reached.\n");
							}
							// decide whether to feed proposed goal pose or overide with intermediate point that is en route to proposed goal pose
							else if (e_xy > max_next_position_distance)  // if the position error is off by more than 4.0 [m] overide
							{
								// make next goal pose for propulsion_system have a heading en route to next goal pose from the tasks with a position on a line following that heading that is 4.0 [m] from the USV
								psi_goal_pose = atan2(e_y,e_x);  // [radians] atan2() returns between -PI and PI
								x_goal_pose = x_usv_NED + cos(psi_goal_pose)*max_next_position_distance;
								y_goal_pose = y_usv_NED + sin(psi_goal_pose)*max_next_position_distance;
								propulsion_system_topic_publish();  // update the propulsion_system topic to go to the intermediate goal pose
							}
							else if (e_xy <= max_next_position_distance)
							{
								propulsion_system_topic_publish();  // update the propulsion_system topic to go to the goal pose
							}
							else
							{
								ROS_INFO("PATH_PLANNER: HMMMMMMMMMMMMMMMMMMMMM look into this?\n");
							}
						}  // END OF if (calculations_done)
						else if (!calculations_done)  // if the updated array of poses to accomplish task has not been calculated
						{
							update_animal_path();  // this function will generate the updated array of poses to accomplish task
						}  // END OF else if (!calculations_done)
					}  // END OF if ((PP_USV_pose_update_state_msg.data) && (!propulsion_system_topic_published))
				}  // END OF if ((loop_count > (loop_goal_recieved)) && (CC_goal_recieved))
				break;

			case 15:  // VRX5: Channel Navigation, Acoustic Beacon Localization and Obstacle Avoidance
				if ((loop_count > (loop_goal_recieved)) && (CC_goal_recieved))  // if the goal poses have been recieved
				{
					if ((NA_state == 1) && (PS_state == 1) && (!E_reached))  // if the navigation_array is providing NED USV state and the propulsion_system is ON
					{
						set_goal_pose(x_goal_poses[point], y_goal_poses[point], psi_goal_poses[point]);  // set the goal pose to the current goal pose in the array of goal poses
						calculate_pose_errors();
						if ((e_xy < e_xy_allowed) && ((float)abs(e_psi) < e_psi_allowed) && (!E_reached))  // if within pose tolerance and last pose not reached 
						{
							point += 1;  // increment the point place keeper
							ROS_INFO("Point %i of %i reached. --MC", point, goal_poses_quantity);
							if (point == goal_poses_quantity)  // if all the goal poses have been reached
							{
							  E_reached = true;  // true means the last pose has been reached
							  ROS_INFO("End pose has been reached. --MC\n");
							}
						}
					}
					propulsion_system_topic_publish();
				}
				break;

			case 16:  // VRX6: Scan and Dock and Deliver
			
				break;
				
			default:
				break;
		}  // END OF switch(PP_state)

		PATH_PLANNER_inspector();  // check that entire system is ready for next cycle
		ros::spinOnce();  // update subscribers
		loop_rate.sleep();  // sleep to accomplish set loop_rate
	}  // END OF while(ros::ok())

	ros::spinOnce();
	loop_rate.sleep();

	return 0;
}  // END OF main()
//.........................................................................................................END OF Main Program...........................................................................................................