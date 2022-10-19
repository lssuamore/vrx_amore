//  Filename:  mission_control.cpp
//  Creation Date:  3/25/2022
//  Last Revision Date:  9/21/2022
//  Author [email]:  Brad Hacker [bhacker@lssu.edu]
//  Revisor(s) [Revision Date]:
//  Organization/Institution:  Lake Superior State University RobotX Team AMORE
// 
//  Notes from author: PLEASE MAKE ANY PRINTOUTS START WITH "MISSION_CONTROL: " TO KNOW THAT IT IS FROM MISSION_CONTROL
//
// ...............................About mission_control.cpp......................................
//  This code acts as the autonomous state machine of the WAM-V USV.
//  It will subscribe to the "vrx/task/info" to control the state of the system.
//  This code will subscribe to goal poses given from the navigation_array.
//  Dependent on the current task state and system state, mission_control
//  will publish whether or not the low level controllers should be on.
//
//		Inputs [subscribers]: "waypoints_NED" (converted goal pose array), "/vrx/task/info", "initialization_states"
//		Outputs [publishers]: states of all other executables

//...............................................................................................Included Libraries and Message Types.........................................................................................
#include "ros/ros.h"
#include "ros/console.h"
#include "time.h"
#include <sstream>
#include <iostream>
#include "math.h"
#include "stdio.h"
#include "amore/state.h"  // message type used to communicate state for rudimentary codes
#include "std_msgs/Bool.h"  // message type used for communicating initialization status to mission_control
#include "std_msgs/Int64.h"  // message type used for acoustic status feedback
#include "vrx_gazebo/Task.h" // message type published by VRX detailing current task and state
//......................................................................................End of Included Libraries and Message Types.....................................................................................

//.......................................................................................................................Constants.....................................................................................................................
#define PI 3.14159265
//................................................................................................................End of Constants...............................................................................................................

//.................................................................................................................Global Variables..............................................................................................................
int loop_count = 0;  // loop counter
// initialization bools
bool system_initialized = false;  // false means the system has not been initialized
// THE FOLLOWING SIX BOOLS ARE USED TO DETERMINE IF SUBSYTEMS HAVE BEEN INITIALIZED
bool navigation_array_initialized = false;
bool coordinate_converter_initialized = false;
bool path_planner_initialized = false;
bool propulsion_system_initialized = false;
bool perception_array_initialized = false;
bool acoustic_initialized = false;

bool vrx_mode;  // false means  vrx mode is not true, and that mission_control should get the mission from /MC parameter in launch file
bool CC_goal_poses_published = false;  // false means the NED poses have not yet been calculated and published by navigation_array
int acoustic_task_status;  // status feedback from the acoustic system (subscribed value)

float start_sec, current_sec;
ros::Duration duration;  // used for calculations of time durations
ros::Time start_time, current_time, last_time;  // creates time variables

//	STATES CONCERNED WITH "mission_control"
//	0 = On standby
//  1 = Dynamic navigation demonstration
//  2 = Entrance and Exit gates
//  3 = Follow the path
//  4 = Wildlife Encounter - React and Report
//  5 = Scan the code
//  6 = Detect and dock
//  7 = Find and Fling
//  8 = UAV replenishment
int MC_state;

//=====STATE MESSAGES AND PUBLISHERS=====
//	STATES CONCERNED WITH "navigation_array"
//	0 = On standby
//	1 = USV NED state converter
amore::state MC_na_state_msg;  // "MC_na_state" message
ros::Publisher MC_na_state_pub;  // "MC_na_state" publisher

//	STATES CONCERNED WITH "coordinate_converter"
//	0 = On standby
//	1 = VRX1: Station-Keeping NED goal pose converter
//	2 = VRX2: Wayfinding NED goal pose converter
//	4 = VRX4: Wildlife NED animals converter
amore::state MC_cc_state_msg;  // "MC_cc_state" message
ros::Publisher MC_cc_state_pub;  // "MC_cc_state" publisher

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
amore::state MC_pp_state_msg;  // "MC_pp_state" message
ros::Publisher MC_pp_state_pub;  // "MC_pp_state" publisher

//	STATES CONCERNED WITH "propulsion_system"
//	0 = On standby
//	1 = Propulsion system ON
amore::state MC_ps_state_msg;  // "MC_ps_state" message
ros::Publisher MC_ps_state_pub;  // "MC_ps_state" publisher

//	STATES CONCERNED WITH "perception_array"
//	0 = On standby
//	1 = General State
//	13 = VRX3: Landmark Localization and Characterization
//	14 = VRX4: Wildlife Encounter and Avoid
//	15 = VRX5: Channel Navigation, Acoustic Beacon Localization and Obstacle Avoidance
//	16 = VRX6: Scan and Dock and Deliver
amore::state MC_pa_state_msg;  // "MC_pa_state" message
ros::Publisher MC_pa_state_pub;  // "MC_pa_state" publisher

// STATES CONCERNED WITH "acoustics" 
// 0 = On standby
// 1 = Finding entrance gate (white buoy)
// 2 = Navigating between red and green buoys
// 3 = Finding exit gate (black buoy)
// 4 = Navigating to acoustic source
amore::state MC_a_state_msg;  // "MC_a_state" message
ros::Publisher MC_a_state_pub;  // "MC_a_state" publisher
//..........................................................................................................End of Global Variables........................................................................................................

//.....................................................................................................................Functions.......................................................................................................................
// THIS FUNCTION: Updates the state of mission_control when not in vrx_mode
// ACCEPTS: (VOID)
// RETURNS: (VOID)
// =====================================================
void parameters_function()
{
	ros::param::get("/MC", MC_state);
} // END OF parameters_function()

// THIS FUNCTION: Publishes all subsystem states
// ACCEPTS: (VOID)
// RETURNS: (VOID)
// =============================================================================
void publish_states()
{
	// PUBLISH EACH SUBSYTEMS STATE
	// SEND STATE TO NAVIGATION_ARRAY
	MC_na_state_msg.header.seq +=1;  // sequence number
	MC_na_state_msg.header.stamp = current_time;  // set stamp to current time
	MC_na_state_msg.header.frame_id = "mission_control";  // header frame
	MC_na_state_msg.sim_mode.data = vrx_mode;
	MC_na_state_pub.publish(MC_na_state_msg);  // publish MC_na_state_msg to "MC_na_state"

	// SEND STATE TO COORDINATE_CONVERTER
	MC_cc_state_msg.header.seq +=1;  // sequence number
	MC_cc_state_msg.header.stamp = current_time;  // set stamp to current time
	MC_cc_state_msg.header.frame_id = "mission_control";  // header frame
	MC_cc_state_msg.sim_mode.data = vrx_mode;
	MC_cc_state_pub.publish(MC_cc_state_msg);  // publish MC_cc_state_msg to "MC_cc_state"

	// SEND STATE TO PATH_PLANNER
	MC_pp_state_msg.header.seq +=1;  // sequence number
	MC_pp_state_msg.header.stamp = current_time;  // set stamp to current time
	MC_pp_state_msg.header.frame_id = "mission_control";  // header frame
	MC_pp_state_msg.sim_mode.data = vrx_mode;
	MC_pp_state_pub.publish(MC_pp_state_msg);  // publish MC_pp_state_msg to "MC_pp_state"

	// SEND STATE TO PROPULSION_SYSTEM
	MC_ps_state_msg.header.seq +=1;  // sequence number
	MC_ps_state_msg.header.stamp = current_time;  // set stamp to current time
	MC_ps_state_msg.header.frame_id = "mission_control";  // header frame
	MC_ps_state_msg.sim_mode.data = vrx_mode;
	MC_ps_state_pub.publish(MC_ps_state_msg);  // publish MC_ps_state_msg to "MC_ps_state"

	// SEND STATE TO PERCEPTION_ARRAY
	MC_pa_state_msg.header.seq +=1;  // sequence number
	MC_pa_state_msg.header.stamp = current_time;  // set stamp to current time
	MC_pa_state_msg.header.frame_id = "mission_control";  // header frame
	MC_pa_state_msg.sim_mode.data = vrx_mode;
	MC_pa_state_pub.publish(MC_pa_state_msg);  // publish MC_pa_state_msg to "MC_pa_state"

	// SEND STATE TO ACOUSTICS
	MC_a_state_msg.header.seq +=1;  // sequence number
	MC_a_state_msg.header.stamp = current_time;  // set stamp to current time
	MC_a_state_msg.header.frame_id = "mission_control";  // header frame
	MC_a_state_msg.sim_mode.data = vrx_mode;
	MC_a_state_pub.publish(MC_a_state_msg);  // publish MC_a_state_msg to "MC_a_state"

	/* // UPDATE USER OF EACH CODES STATE
	ROS_INFO("MISSION_CONTROL:-----CURRENT STATES-----");
	if (!vrx_mode)  // if in user control mode
	{
		ROS_INFO("MISSION_CONTROL:      MC_state: %i", MC_state);
	}
	ROS_INFO("MISSION_CONTROL:      NA_state: %i", MC_na_state_msg.state.data);
	ROS_INFO("MISSION_CONTROL:      CC_state: %i", MC_cc_state_msg.state.data);
	ROS_INFO("MISSION_CONTROL:      PP_state: %i", MC_pp_state_msg.state.data);
	ROS_INFO("MISSION_CONTROL:      PS_state: %i", MC_ps_state_msg.state.data);
	ROS_INFO("MISSION_CONTROL:      PA_state: %i", MC_pa_state_msg.state.data);
	ROS_INFO("MISSION_CONTROL:       A_state: %i", MC_a_state_msg.state.data);
	ROS_INFO("MISSION_CONTROL:-----CURRENT STATES-----\n"); */
} // END OF publish_states()

// THIS FUNCTION: Updates all subsystem states if in real-world mode
// ACCEPTS: (VOID)
// RETURNS: (VOID)
// =============================================================================
void user_state_update()
{
	// FIRST: update the user desired mission
	parameters_function();
	
	// NEXT: reset states to be set accordingly to current system statuses
	//	STATES CONCERNED WITH "navigation_array"
	//	0 = On standby
	//	1 = USV NED state converter
	MC_na_state_msg.state.data = 0;

	//	STATES CONCERNED WITH "coordinate_converter"
	//	0 = On standby
	MC_cc_state_msg.state.data = 0;

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
	MC_pp_state_msg.state.data = 0;

	//	STATES CONCERNED WITH "propulsion_system"
	//	0 = On standby
	//	1 = Propulsion system ON
	MC_ps_state_msg.state.data = 0;

	//	STATES CONCERNED WITH "perception_array"
	//	0 = On standby
	//	1 = General State
	MC_pa_state_msg.state.data = 0;

	// STATES CONCERNED WITH "acoustics"
	// 0 = On standby
	// 1 = Finding entrance gate (white buoy)
	// 2 = Navigating between red and green buoys
	// 3 = Finding exit gate (black buoy)
	// 4 = Navigating to acoustic source
	MC_a_state_msg.state.data = 0;

	if (system_initialized)  // if entire system is initialized
	{
		if (MC_state == 1)  // Dynamic navigation demonstration
		{
			MC_na_state_msg.state.data = 1;  // 1 = USV NED state converter
			MC_pp_state_msg.state.data = 1;  // 1 = Dynamic navigation demonstration path planner
			MC_ps_state_msg.state.data = 1;  // 1 = Propulsion system ON
			MC_pa_state_msg.state.data = 1;  // 1 = General State
		}
		else if (MC_state == 2)  // Entrance and Exit gates
		{
			MC_na_state_msg.state.data = 1;  // 1 = USV NED state converter
			MC_pp_state_msg.state.data = 2;  // 2 = Entrance and Exit gates path planner
			MC_ps_state_msg.state.data = 1;  // 1 = Propulsion system ON
			MC_pa_state_msg.state.data = 1;  // 1 = General State
		}
		else if (MC_state == 3)  // Follow the path
		{
			MC_na_state_msg.state.data = 1;  // 1 = USV NED state converter
			MC_pp_state_msg.state.data = 3;  // 3 = Follow the path path planner
			MC_ps_state_msg.state.data = 1;  // 1 = Propulsion system ON
			MC_pa_state_msg.state.data = 1;  // 1 = General State
		}
		else if (MC_state == 4)  // Wildlife Encounter - React and Report
		{
			MC_na_state_msg.state.data = 1;  // 1 = USV NED state converter
			MC_pp_state_msg.state.data = 4;  // 4 = Wildlife Encounter - React and Report path planner
			MC_ps_state_msg.state.data = 1;  // 1 = Propulsion system ON
			MC_pa_state_msg.state.data = 1;  // 1 = General State
		}
		else if (MC_state == 5)  // Scan the code
		{
			MC_na_state_msg.state.data = 1;  // 1 = USV NED state converter
			MC_pp_state_msg.state.data = 5;  // 5 = Scan the code path planner
			MC_ps_state_msg.state.data = 1;  // 1 = Propulsion system ON
			MC_pa_state_msg.state.data = 1;  // 1 = General State
		}
		else if (MC_state == 6)  // Detect and dock
		{
			MC_na_state_msg.state.data = 1;  // 1 = USV NED state converter
			MC_pp_state_msg.state.data = 6;  // 6 = Detect and dock path planner
			MC_ps_state_msg.state.data = 1;  // 1 = Propulsion system ON
			MC_pa_state_msg.state.data = 1;  // 1 = General State
		}
		else if (MC_state == 7)  // Find and Fling
		{
			MC_na_state_msg.state.data = 1;  // 1 = USV NED state converter
			MC_pp_state_msg.state.data = 7;  // 7 = Find and Fling path planner
			MC_ps_state_msg.state.data = 1;  // 1 = Propulsion system ON
			MC_pa_state_msg.state.data = 1;  // 1 = General State
		}
		else if (MC_state == 8)  // UAV replenishment
		{
			MC_na_state_msg.state.data = 1;  // 1 = USV NED state converter
			MC_pp_state_msg.state.data = 8;  // 8 = UAV replenishment path planner
			MC_ps_state_msg.state.data = 1;  // 1 = Propulsion system ON
			MC_pa_state_msg.state.data = 1;  // 1 = General State
		}
		else if (MC_state == 0)
		{
			// ALL CODES ON STANDBY
			ROS_INFO("MISSION_CONTROL: ON STANDBY.\n");
		}
		else
		{
			ROS_ERROR("MISSION_CONTROL: CAN'T RETRIEVE MC STATE FROM LAUNCH FILE.\n");
		}

		publish_states();  // PUBLISH EACH SUBSYTEMS STATE
	}  // END OF if (system_initialized)
} // END OF user_state_update()

// THIS FUNCTION: Checks initialization status of entire system and runs function to update states
// ACCEPTS: (VOID) 
// RETURNS: (VOID) 
//=============================================================================================================
void MISSION_CONTROL_inspector()
{
	current_time = ros::Time::now();  // sets current_time to the time it is now
	loop_count += 1;  // increment loop counter
	if ((navigation_array_initialized) && (coordinate_converter_initialized) && (path_planner_initialized) && (propulsion_system_initialized) && (perception_array_initialized)&& (acoustic_initialized))
	{
		system_initialized = true;
	}
	else
	{
		system_initialized = false;
		ROS_INFO("MISSION_CONTROL: System is not initialized yet.\n");
	}
	if (!vrx_mode)
	{
		user_state_update();
	}
	// UPDATE USER OF INITIALIZATION STATUSES
	//ROS_DEBUG("MISSION_CONTROL: Loop count = %i", loop_count);
	// if (system_initialized)
	// {
		// ROS_DEBUG("MISSION_CONTROL: mission_control_initialized");
	// }
	// if (navigation_array_initialized)
	// {
		// ROS_DEBUG("MISSION_CONTROL: navigation_array_initialized");
	// }
	// if (coordinate_converter_initialized)
	// {
		// ROS_DEBUG("MISSION_CONTROL: coordinate_converter_initialized");
	// }
	// if (path_planner_initialized)
	// {
		// ROS_DEBUG("MISSION_CONTROL: path_planner_initialized");
	// }
	// if (propulsion_system_initialized)
	// {
		// ROS_DEBUG("MISSION_CONTROL: propulsion_system_initialized");
	// }
	// if (perception_array_initialized)
	// {
		// ROS_DEBUG("MISSION_CONTROL: perception_array_initialized");
	// }
}  // END OF MISSION_CONTROL_inspector()

// THIS FUNCTION: Subscribes to the navigation_array to check initialization status
// ACCEPTS: std_msgs::Bool from "NA_initialization_state"
// RETURNS: (VOID)
//=============================================================================================================
void NA_initialization_state_update(const std_msgs::Bool status)
{
	navigation_array_initialized = status.data;
}  // END OF NA_initialization_state_update()

// THIS FUNCTION: Subscribes to the coordinate_converter to check initialization status
// ACCEPTS: std_msgs::Bool from "CC_initialization_state"
// RETURNS: (VOID)
//=============================================================================================================
void CC_initialization_state_update(const std_msgs::Bool status)
{
	coordinate_converter_initialized = status.data;
}  // END OF CC_initialization_state_update()

// THIS FUNCTION: Subscribes to the path_planner to check initialization status
// ACCEPTS: std_msgs::Bool from "PP_initialization_state"
// RETURNS: (VOID)
//=============================================================================================================
void PP_initialization_state_update(const std_msgs::Bool status)
{
	path_planner_initialized = status.data;
}  // END OF PP_initialization_state_update()

// THIS FUNCTION: Subscribes to the propulsion_system to check initialization status
// ACCEPTS: std_msgs::Bool from "PS_initialization_state"
// RETURNS: (VOID)
//=============================================================================================================
void PS_initialization_state_update(const std_msgs::Bool status)
{
	propulsion_system_initialized = status.data;
}  // END OF PS_initialization_state_update()

// THIS FUNCTION: Subscribes to the perception_array to check initialization status
// ACCEPTS: std_msgs::Bool from "PA_initialization_state"
// RETURNS: (VOID)
//=============================================================================================================
void PA_initialization_state_update(const std_msgs::Bool status)
{
	perception_array_initialized = status.data;
}  // END OF PA_initialization_state_update()

// THIS FUNCTION: Subscribes to acoustics to check initialization status
// ACCEPTS:std_msgs::Bool from "A_initialization_state"
// RETURNS: (VOID)
//=============================================================================================================
void A_initialization_state_update(const std_msgs::Bool status)
{
	acoustic_initialized = status.data;
}  // END OF A_initialization_state_update()

// THIS FUNCTION: Subscribes to acoustics to check status
// ACCEPTS: std_msgs::Int64 from "A_system_state"
// RETURNS: (VOID)
//=============================================================================================================
void A_system_state_update(const std_msgs::Int64 status)
{
	// Status;
	// 1 = entrance gate has been traversed
	// 2 = buoy channel has been traversed
	// 3 = exit gate has been traversed
	acoustic_task_status  = status.data; 
}  // END OF A_system_state_update()

// THIS FUNCTION: Updates when VRX Task goal poses have been converted and published
// ACCEPTS: std_msgs::Bool from "CC_goal_poses_publish_state"
// RETURNS: (VOID)
//=============================================================================================================
void CC_goal_poses_publish_state_update(const std_msgs::Bool status)
{
	CC_goal_poses_published = status.data;
	// UPDATE USER OF COORDINATE_CONVERTER STATUS
	// if (CC_goal_poses_published)
	// {
		// ROS_DEBUG("MISSION_CONTROL: COORDINATE CONVERTER FINISHED");
	// }
	// else
	// {
		// ROS_DEBUG("MISSION_CONTROL: COORDINATE CONVERTER NOT FINISHED");
	// }
}  // END OF CC_goal_poses_publish_state_update()

// THIS FUNCTION: Updates and publishes all subsytem states, as well as mission_control state dependent on current system statuses
// ACCEPTS: vrx_gazebo::Task from "vrx/task/info"
// RETURNS: (VOID)
//=============================================================================================================
void vrx_state_update(const vrx_gazebo::Task::ConstPtr& msg)
{
	if (!vrx_mode)  // if vrx_mode is false
	{
		vrx_mode = true;  // make vrx_mode true to notify mission_control to launch accordingly
	}
	
	// FIRST RESET STATES TO BE SET ACCORDINGLY TO CURRENT SYSTEM STATUSES
	//	STATES CONCERNED WITH "navigation_array"
	//	0 = On standby
	//	1 = USV NED state converter
	MC_na_state_msg.state.data = 0;

	//	STATES CONCERNED WITH "coordinate_converter"
	//	0 = On standby
	//	1 = VRX1: Station-Keeping NED goal pose converter
	//	2 = VRX2: Wayfinding NED goal pose converter
	//	4 = VRX4: Wildlife NED animals converter
	MC_cc_state_msg.state.data = 0;

	//	STATES CONCERNED WITH "path_planner"
	//	0 = On standby
	//	11 = VRX1: Station-Keeping
	//	12 = VRX2: Wayfinding
	//	14 = VRX4: Wildlife Encounter and Avoid
	//	15 = VRX5: Channel Navigation, Acoustic Beacon Localization and Obstacle Avoidance
	//	16 = VRX6: Scan and Dock and Deliver
	MC_pp_state_msg.state.data = 0;

	//	STATES CONCERNED WITH "propulsion_system"
	//	0 = On standby
	//	1 = Propulsion system ON
	MC_ps_state_msg.state.data = 0;

	//	STATES CONCERNED WITH "perception_array"
	//	0 = On standby
	//	1 = General State
	//	13 = VRX3: Landmark Localization and Characterization
	//	14 = VRX4: Wildlife Encounter and Avoid
	//	15 = VRX5: Channel Navigation, Acoustic Beacon Localization and Obstacle Avoidance
	//	16 = VRX6: Scan and Dock and Deliver
	MC_pa_state_msg.state.data = 0;

	// STATES CONCERNED WITH "acoustics"
	// 0 = On standby
	// 1 = Finding entrance gate (white buoy)
	// 2 = Navigating between red and green buoys
	// 3 = Finding exit gate (black buoy)
	// 4 = Navigating to acoustic source
	MC_a_state_msg.state.data = 0;

	if (system_initialized)  // Do not begin subsytem activity until system is initialized
	{
		if ((msg->name == "station_keeping") && ((msg->state == "ready") || (msg->state == "running")))  // if the station_keeping task is ready or running
		{
			MC_pp_state_msg.state.data = 11;  // set path_planner to VRX1: Station-Keeping
			if (CC_goal_poses_published)  // if the goal pose has been converted from lat/long to NED and published by coordinate_converter
			{
				MC_na_state_msg.state.data = 1;  // set navigation_array to USV NED state converter
				MC_ps_state_msg.state.data = 1;  // set propulsion_system to Propulsion system ON
			}
			else
			{
				MC_cc_state_msg.state.data = 1;  // set coordinate_converter to VRX1: Station-Keeping NED goal pose converter
			}
		}  // END OF if ((msg->name == "station_keeping") && ((msg->state == "ready") || (msg->state == "running")))

		else if ((msg->name == "wayfinding") && ((msg->state == "ready") || (msg->state == "running")))  // if the wayfinding task is ready or running
		{
			MC_pp_state_msg.state.data = 12;  // set path_planner to VRX2: Wayfinding  // WAS TURNED ON ONCE GOAL POSES WERE PUBLISHED
			if (CC_goal_poses_published)  // if the goal poses have been converted from lat/long to NED and published by coordinate_converter
			{
				MC_na_state_msg.state.data = 1;  // set navigation_array to USV NED state converter
				MC_ps_state_msg.state.data = 1;  // set propulsion_system to Propulsion system ON
			}
			else
			{
				MC_cc_state_msg.state.data = 2;  // set coordinate_converter to VRX2: Wayfinding NED goal pose converter
			}
		}  // END OF else if ((msg->name == "wayfinding") && ((msg->state == "ready") || (msg->state == "running")))

		// 	NEEDS WORK
		else if (msg->name == "perception")
		{
			MC_na_state_msg.state.data = 1;  // set navigation_array to USV NED state converter
			MC_pa_state_msg.state.data = 13;  // set perception_array to VRX3: Landmark Localization and Characterization
		}  // END OF else if (msg->name == "perception")

		// INTEGRATED TASK CODES FOLLOW
		// FOR TASK 4, NEEDS WORK
		else if ((msg->name == "wildlife") && ((msg->state == "ready") || (msg->state == "running")))  // if the wildlife task is ready or running
		{
			MC_pp_state_msg.state.data = 4;  // set path_planner to VRX4: Wildlife Encounter and Avoid
			MC_ps_state_msg.state.data = 1;  // set propulsion_system to Propulsion system ON
			if (!CC_goal_poses_published)  // if the goal poses have not been converted and published to path_planner from navigation_array
			{
				MC_cc_state_msg.state.data = 4;  // set coordinate_converter to VRX4: Wildlife NED animals converter
			}
			else
			{
				MC_na_state_msg.state.data = 1;  // set navigation_array to USV NED state converter
			}
		}  // END OF else if ((msg->name == "wildlife") && ((msg->state == "ready") || (msg->state == "running")))

		// NEEDS WORK
		else if ((msg->name == "gymkhana") && ((msg->state == "ready") || (msg->state == "running")))  // if the gymkhana task is ready or running
		{
			MC_na_state_msg.state.data = 1;  // set navigation_array to USV NED state converter
			MC_pp_state_msg.state.data = 5;  // set path_planner to VRX5: Channel Navigation, Acoustic Beacon Localization and Obstacle Avoidance
			MC_pa_state_msg.state.data = 15;  // set perception_array to VRX5: Channel Navigation, Acoustic Beacon Localization and Obstacle Avoidance
			MC_a_state_msg.state.data = 1;  // set acoustics to Finding entrance gate (white buoy)
			
			if(acoustic_task_status == 1)
			{
				MC_a_state_msg.state.data = 2;  // set acoustics to Navigating between red and green buoys
			}
			else if (acoustic_task_status == 2)
			{
				MC_a_state_msg.state.data = 3;  // set acoustics to Finding exit gate (black buoy)
			}
			else if (acoustic_task_status == 3)
			{
				MC_a_state_msg.state.data = 4;  // set acoustics to Navigating to acoustic source
			}
		}  // END OF else if ((msg->name == "gymkhana") && ((msg->state == "ready") || (msg->state == "running")))

		else if ((msg->state == "initial") || (msg->state == "finished"))  // if the task is initial or finished
		{
			CC_goal_poses_published = false;  // reset task statuses as long as task is in "initial" or "finished" state
		}  // END OF else if ((msg->state == "initial") || (msg->state == "finished"))

		/* else if (msg->name == "scan_dock_deliver")
		{
			if ()
			{
				MC_na_state_msg.state.data = 0;  // set navigation_array to...
				MC_ps_state_msg.state.data = 0;  // set propulsion_system to...
				MC_pp_state_msg.state.data = 0;  // set path_planner to...
			}
			else
			{
				MC_na_state_msg.state.data = 0;  // set navigation_array to...
				MC_ps_state_msg.state.data = 0;  // set propulsion_system to...
				MC_pp_state_msg.state.data = 0;  // set path_planner to...
			}
		}  // END OF else if (msg->name == "scan_dock_deliver") */

		publish_states();  // PUBLISH EACH SUBSYTEMS STATE
	}  // END OF if (system_initialized)
}  // END OF vrx_state_update()

/* // THIS FUNCTION: Updates and publishes all subsytem states, as well as mission_control state dependent on current system statuses
// ACCEPTS: vrx_gazebo::Task from "vrx/task/info"
// RETURNS: (VOID)
//=============================================================================================================
void state_update(const vrx_gazebo::Task::ConstPtr& msg)  // NOTE: To simplify, use just message variables !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
{
	// FIRST RESET STATES TO BE SET ACCORDINGLY TO CURRENT SYSTEM STATUSES
	//	STATES CONCERNED WITH "navigation_array"
	//	0 = On standby
	//	1 = USV NED state converter
	MC_na_state_msg.state.data = 0;

	//	STATES CONCERNED WITH "coordinate_converter"
	//	0 = On standby
	//	1 = VRX1: Station-Keeping NED goal pose converter
	//	2 = VRX2: Wayfinding NED goal pose converter
	//	4 = VRX4: Wildlife NED animals converter
	MC_cc_state_msg.state.data = 0;

	//	STATES CONCERNED WITH "path_planner"
	//	0 = On standby
	//	1 = VRX1: Station-Keeping
	//	2 = VRX2: Wayfinding
	//	4 = VRX4: Wildlife Encounter and Avoid
	//	5 = VRX5: Channel Navigation, Acoustic Beacon Localization and Obstacle Avoidance
	//	6 = VRX6: Scan and Dock and Deliver
	MC_pp_state_msg.state.data = 0;

	//	STATES CONCERNED WITH "propulsion_system"
	//	0 = On standby
	//	1 = Propulsion system ON
	MC_ps_state_msg.state.data = 0;

	//	STATES CONCERNED WITH "perception_array"
	//	0 = On standby
	//	1 = General State
	//	3 = VRX3: Landmark Localization and Characterization
	//	4 = VRX4: Wildlife Encounter and Avoid
	//	5 = VRX5: Channel Navigation, Acoustic Beacon Localization and Obstacle Avoidance
	//	6 = VRX6: Scan and Dock and Deliver
	MC_pa_state_msg.state.data = 0;

	// STATES CONCERNED WITH "acoustics"
	// 0 = On standby
	// 1 = Finding entrance gate (white buoy)
	// 2 = Navigating between red and green buoys
	// 3 = Finding exit gate (black buoy)
	// 4 = Navigating to acoustic source
	MC_a_state_msg.state.data = 0;

	if (system_initialized)  // Do not begin subsytem activity until system is initialized
	{
		if ((msg->name == "station_keeping") && ((msg->state == "ready") || (msg->state == "running")))  // if the station_keeping task is ready or running
		{
			MC_pp_state_msg.state.data = 1;  // set path_planner to VRX1: Station-Keeping
			if (CC_goal_poses_published)  // if the goal pose has been converted from lat/long to NED and published by coordinate_converter
			{
				MC_na_state_msg.state.data = 1;  // set navigation_array to USV NED state converter
				MC_ps_state_msg.state.data = 1;  // set propulsion_system to Propulsion system ON
			}
			else
			{
				MC_cc_state_msg.state.data = 1;  // set coordinate_converter to VRX1: Station-Keeping NED goal pose converter
			}
		}  // END OF if ((msg->name == "station_keeping") && ((msg->state == "ready") || (msg->state == "running")))

		else if ((msg->name == "wayfinding") && ((msg->state == "ready") || (msg->state == "running")))  // if the wayfinding task is ready or running
		{
			MC_pp_state_msg.state.data = 2;  // set path_planner to VRX2: Wayfinding  // WAS TURNED ON ONCE GOAL POSES WERE PUBLISHED
			if (CC_goal_poses_published)  // if the goal poses have been converted from lat/long to NED and published by coordinate_converter
			{
				MC_na_state_msg.state.data = 1;  // set navigation_array to USV NED state converter
				MC_ps_state_msg.state.data = 1;  // set propulsion_system to Propulsion system ON
			}
			else
			{
				MC_cc_state_msg.state.data = 2;  // set coordinate_converter to VRX2: Wayfinding NED goal pose converter
			}
		}  // END OF else if ((msg->name == "wayfinding") && ((msg->state == "ready") || (msg->state == "running")))

		// 	NEEDS WORK
		else if (msg->name == "perception")
		{
			MC_na_state_msg.state.data = 1;  // set navigation_array to USV NED state converter
			MC_pa_state_msg.state.data = 3;  // set perception_array to VRX3: Landmark Localization and Characterization
		}  // END OF else if (msg->name == "perception")

		// INTEGRATED TASK CODES FOLLOW
		// FOR TASK 4, NEEDS WORK
		else if ((msg->name == "wildlife") && ((msg->state == "ready") || (msg->state == "running")))  // if the wildlife task is ready or running
		{
			MC_pp_state_msg.state.data = 4;  // set path_planner to VRX4: Wildlife Encounter and Avoid
			MC_ps_state_msg.state.data = 1;  // set propulsion_system to Propulsion system ON
			if (!CC_goal_poses_published)  // if the goal poses have not been converted and published to path_planner from navigation_array
			{
				MC_cc_state_msg.state.data = 4;  // set coordinate_converter to VRX4: Wildlife NED animals converter
			}
			else
			{
				MC_na_state_msg.state.data = 1;  // set navigation_array to USV NED state converter
			}
		}  // END OF else if ((msg->name == "wildlife") && ((msg->state == "ready") || (msg->state == "running")))

		// NEEDS WORK
		else if ((msg->name == "gymkhana") && ((msg->state == "ready") || (msg->state == "running")))  // if the gymkhana task is ready or running
		{
			MC_na_state_msg.state.data = 1;  // set navigation_array to USV NED state converter
			MC_pp_state_msg.state.data = 5;  // set path_planner to VRX5: Channel Navigation, Acoustic Beacon Localization and Obstacle Avoidance
			MC_pa_state_msg.state.data = 5;  // set perception_array to VRX5: Channel Navigation, Acoustic Beacon Localization and Obstacle Avoidance
			MC_a_state_msg.state.data = 1;  // set acoustics to Finding entrance gate (white buoy)
			
			if(acoustic_task_status == 1)
			{
				MC_a_state_msg.state.data = 2;  // set acoustics to Navigating between red and green buoys
			}
			else if (acoustic_task_status == 2)
			{
				MC_a_state_msg.state.data = 3;  // set acoustics to Finding exit gate (black buoy)
			}
			else if (acoustic_task_status == 3)
			{
				MC_a_state_msg.state.data = 4;  // set acoustics to Navigating to acoustic source
			}
		}  // END OF else if ((msg->name == "gymkhana") && ((msg->state == "ready") || (msg->state == "running")))

		else if ((msg->state == "initial") || (msg->state == "finished"))  // if the task is initial or finished
		{
			CC_goal_poses_published = false;  // reset task statuses as long as task is in "initial" or "finished" state
		}  // END OF else if ((msg->state == "initial") || (msg->state == "finished"))

		// else if (msg->name == "scan_dock_deliver")
		// {
			// if ()
			// {
				// MC_na_state_msg.state.data = 0;  // set navigation_array to...
				// MC_ps_state_msg.state.data = 0;  // set propulsion_system to...
				// MC_pp_state_msg.state.data = 0;  // set path_planner to...
			// }
			// else
			// {
				// MC_na_state_msg.state.data = 0;  // set navigation_array to...
				// MC_ps_state_msg.state.data = 0;  // set propulsion_system to...
				// MC_pp_state_msg.state.data = 0;  // set path_planner to...
			// }
		// }  // END OF else if (msg->name == "scan_dock_deliver")

		// PUBLISH EACH SUBSYTEMS STATE
		// SEND STATE TO NAVIGATION_ARRAY
		MC_na_state_msg.header.seq +=1;  // sequence number
		MC_na_state_msg.header.stamp = current_time;  // set stamp to current time
		MC_na_state_msg.header.frame_id = "mission_control";  // header frame
		MC_na_state_pub.publish(MC_na_state_msg);  // publish MC_na_state_msg to "MC_na_state"

		// SEND STATE TO COORDINATE_CONVERTER
		MC_cc_state_msg.header.seq +=1;  // sequence number
		MC_cc_state_msg.header.stamp = current_time;  // set stamp to current time
		MC_cc_state_msg.header.frame_id = "mission_control";  // header frame
		MC_cc_state_pub.publish(MC_cc_state_msg);  // publish MC_cc_state_msg to "MC_cc_state"

		// SEND STATE TO PATH_PLANNER
		MC_pp_state_msg.header.seq +=1;  // sequence number
		MC_pp_state_msg.header.stamp = current_time;  // set stamp to current time
		MC_pp_state_msg.header.frame_id = "mission_control";  // header frame
		MC_pp_state_pub.publish(MC_pp_state_msg);  // publish MC_pp_state_msg to "MC_pp_state"

		// SEND STATE TO PROPULSION_SYSTEM
		MC_ps_state_msg.header.seq +=1;  // sequence number
		MC_ps_state_msg.header.stamp = current_time;  // set stamp to current time
		MC_ps_state_msg.header.frame_id = "mission_control";  // header frame
		MC_ps_state_pub.publish(MC_ps_state_msg);  // publish MC_ps_state_msg to "MC_ps_state"		

		// SEND STATE TO PERCEPTION_ARRAY
		MC_pa_state_msg.header.seq +=1;  // sequence number
		MC_pa_state_msg.header.stamp = current_time;  // set stamp to current time
		MC_pa_state_msg.header.frame_id = "mission_control";  // header frame
		MC_pa_state_pub.publish(MC_pa_state_msg);  // publish MC_pa_state_msg to "MC_pa_state"

		// SEND STATE TO PERCEPTION_ARRAY
		MC_a_state_msg.header.seq +=1;  // sequence number
		MC_a_state_msg.header.stamp = current_time;  // set stamp to current time
		MC_a_state_msg.header.frame_id = "mission_control";  // header frame
		MC_a_state_pub.publish(MC_a_state_msg);  // publish MC_a_state_msg to "MC_a_state"

		// UPDATE USER OF EACH CODES STATE
		ROS_INFO("MISSION_CONTROL:-----CURRENT STATES-----");
		ROS_INFO("MISSION_CONTROL:      NA_state: %i", MC_na_state_msg.state.data);
		ROS_INFO("MISSION_CONTROL:      CC_state: %i", MC_cc_state_msg.state.data);
		ROS_INFO("MISSION_CONTROL:      PP_state: %i", MC_pp_state_msg.state.data);
		ROS_INFO("MISSION_CONTROL:      PS_state: %i", MC_ps_state_msg.state.data);
		ROS_INFO("MISSION_CONTROL:      PA_state: %i", MC_pa_state_msg.state.data);
		ROS_INFO("MISSION_CONTROL:       A_state: %i", MC_a_state_msg.state.data);
		ROS_INFO("MISSION_CONTROL:-----CURRENT STATES-----\n");
	}  // END OF if (system_initialized)
}  // END OF state_update() */
//.............................................................................................................END OF Functions...............................................................................................................

//..................................................................................................................Main Program..................................................................................................................
int main(int argc, char **argv)
{
	// names the program for visual purposes
	ros::init(argc, argv, "mission_control");

	// NodeHandles
	ros::NodeHandle nh1, nh2, nh3, nh4, nh5, nh6, nh7, nh8, nh9, nh10, nh11;

	// Subscribers
	ros::Subscriber NA_initialization_state_sub = nh1.subscribe("NA_initialization_state", 1, NA_initialization_state_update);  // initialization status of navigation_array
	ros::Subscriber CC_initialization_state_sub = nh2.subscribe("CC_initialization_state", 1, CC_initialization_state_update);  // initialization status of coordinate_converter
	ros::Subscriber PP_initialization_state_sub = nh3.subscribe("PP_initialization_state", 1, PP_initialization_state_update);  // initialization status of path_planner
	ros::Subscriber PS_initialization_state_sub = nh4.subscribe("PS_initialization_state", 1, PS_initialization_state_update);  // initialization status of propulsion_system
	ros::Subscriber PA_initialization_state_sub = nh5.subscribe("PA_initialization_state", 1, PA_initialization_state_update);  // initialization status of perception_array
	ros::Subscriber A_initialization_state_sub = nh6.subscribe("A_initialization_state", 1, A_initialization_state_update);  // initialization status of acoustic
	// set up vrx subscribers
	ros::Subscriber task_status_sub = nh7.subscribe("/vrx/task/info", 1, vrx_state_update);  // VRX task topic
	ros::Subscriber CC_goal_poses_publish_state_sub = nh8.subscribe("CC_goal_poses_publish_state", 1, CC_goal_poses_publish_state_update);  // whether or not goal waypoints have been converted and published yet
	ros::Subscriber A_system_state_sub = nh9.subscribe("A_system_state", 1, A_system_state_update);  // subscriber for status of acoustic program

	// Publishers
	// these publishers publish all the state topics off all the other programs so that mission_control can tell them all how to operate
	MC_na_state_pub = nh11.advertise<amore::state>("MC_na_state", 1);  // current navigation_array state
	MC_cc_state_pub = nh11.advertise<amore::state>("MC_cc_state", 1);  // current coordinate_converter state
	MC_pp_state_pub = nh11.advertise<amore::state>("MC_pp_state", 1);  // current path_planner state
	MC_ps_state_pub = nh11.advertise<amore::state>("MC_ps_state", 1);  // current propulsion_system state
	MC_pa_state_pub = nh11.advertise<amore::state>("MC_pa_state", 1);  // current perception_array state
	MC_a_state_pub = nh11.advertise<amore::state>("MC_a_state", 1);  // current acoustic state

	// initialize header sequences
	MC_na_state_msg.header.seq = 0;
	MC_cc_state_msg.header.seq = 0;
	MC_pp_state_msg.header.seq = 0;
	MC_ps_state_msg.header.seq = 0;
	MC_pa_state_msg.header.seq = 0;
	MC_a_state_msg.header.seq = 0;

	// Timers ///////////////////////////////////////////////////////////////////////////////////////////////////////////
	ros::Time::init();

	// Initialize global variables
	//sets the frequency for which the program sleeps at. 10=1/10 second
	ros::Rate loop_rate(10);
	
	// initial inspection to decide if system is in simulation or real-world mode
	vrx_mode = false;  // false means that the simulation mode is not true, and that the system launch should be set for real-world operation
	while (ros::Time::now().toSec() == 0.0)
	{
		ros::spin();  // spin while the time is not yet non-zero
		ROS_INFO("MISSION_CONTROL:---SPINNING UNTIL TIME IS NON_ZERO---\n");
	}
	ros::Duration(0.5).sleep();  // sleep for half a second
	start_time = ros::Time::now();  // sets start time to the first available non-zero time
	std::cout << "\n";
	ROS_INFO("MISSION_CONTROL: start_time: %.3f", start_time.toSec());
	current_time = ros::Time::now();  // sets current_time to the time it is now
	ROS_INFO("MISSION_CONTROL: current_time: %.3f\n", current_time.toSec());
	duration = current_time - start_time;  // calculates the duration of time since the start_time
	while (duration <= ros::Duration(5.0))
	{
		ros::spinOnce();  // update subscribers
		ros::Duration(1.0).sleep();  // sleep for a second
		std::cout << "\n";
		ROS_INFO("MISSION_CONTROL: Sleeping at 1 Hertz rate");
		ROS_INFO("MISSION_CONTROL: duration: %.3f\n", duration.toSec());
		current_time = ros::Time::now();  // sets current_time to the time it is now
		duration = current_time - start_time;  // calculates the duration of time since the start_time
	}
	if (vrx_mode)  // if vrx_mode is found to be true from initial inspection
	{
		ROS_INFO("MISSION_CONTROL:---VRX MODE ACTIVATED---\n");
	}
	else if (!vrx_mode)  // if vrx_mode is found to be false from initial inspection
	{
		ROS_INFO("MISSION_CONTROL:---USER MODE ACTIVATED---\n");
	}

	// ros::ok() will stop when the user inputs Ctrl+C
	while(ros::ok())
	{
		MISSION_CONTROL_inspector();  // check that entire system is initialized before starting calculations
		ros::spinOnce();  // update subscribers
		loop_rate.sleep();  // sleep to accomplish set loop_rate
		last_time = current_time;  // update last_time
	}  // END OF while(ros::ok())

	ros::spinOnce();
	loop_rate.sleep();

	return 0;
}  // END OF main()
//.........................................................................................................END OF Main Program...........................................................................................................