//  Filename:  propulsion_system_QUT.cpp
//  Creation Date:  11/07/2022
//  Last Revision Date:  11/07/2022
//  Author(s) [email]:  Brad Hacker [bhacker@lssu.edu]
//  Organization/Institution:  Lake Superior State University RobotX Team AMORE
// 
// ...........................propulsion_system_QUT.cpp.......................
//  This code recieves a goal pose to reach in a 2D plane. It then calculates the errors between the goal pose and the current USV pose.
//  PID control theory is used to calculate control efforts based on the errors to correct the position of the USV.
//  The control efforts are then converted to thrusts for each thruster on the QUT USV: PORT_MAIN, STBD_MAIN, PORT_BOW, STBD_BOW.
//  For station-keeping, three degrees of freedom must be controlled, position on a plane (x,y - long/lat) and the heading of the USV.
//  Having four actuators to control makes the system over-actuated since only three degrees of freedom are being controlled.
//  Since the system is over-actuated, control allocation is used to solve for the thruster outputs.
//
//  Inputs and Outputs of the propulsion_system.cpp file
//				Inputs: ["PP_propulsion_system_topic" - amore/propulsion_system - goal pose (x,y,psi) to reach in local NED frame and current USV pose from path_planner]
//
//				Outputs: "thruster_int_right", "thuster_int_left", "angle_int_right", "angle_int_left" - dual-azimuthing Minn Kota thruster commands
//								"PORT_MAIN", "STBD_MAIN", "PORT_BOW", "STBD_BOW" - QUT thruster commands


//...............................................................................................Included Libraries and Message Types.........................................................................................
#include "ros/ros.h"
#include "ros/console.h"
#include "time.h"
#include <sstream>
#include <iostream>
#include <algorithm>  // library included to be able to do std::max function
#include "math.h"
#include "stdio.h"
#include "nav_msgs/Odometry.h"  // message type used for receiving USV state from navigation_array
#include "std_msgs/Bool.h"  // message type used for communicating initialization status to mission_control
#include "amore/state.h"  // message type used to recieve state of operation from mission_control
#include "amore/propulsion_system.h"  // message type that holds all needed operation information from path_planner
#include "std_msgs/Float32.h"  // message type of thruster commands, and type for control efforts
#include "amore/control_efforts.h"  // message type that holds thrust x, thrust y, and moment z
#include "std_msgs/Int32.h"  // thruster commands
//......................................................................................End of Included Libraries and Message Types.....................................................................................

//.......................................................................................................................Constants.....................................................................................................................
#define PI 3.14159265
//................................................................................................................End of Constants...............................................................................................................

//.................................................................................................................Global Variables..............................................................................................................
int loop_count = 0;  // loop counter
int loop_new_goal;  // loop number for when the the controller recieves a new goal to ensure differential and integral terms are not started until they are calculated
bool PS_state_ON = false;  // used to set and reset the above count holder like a oneshot
	
//	STATES CONCERNED WITH "propulsion_system"
//	0 = On standby
//	1 = Propulsion system ON
int PS_state;
//	drive_config is the drive configuration of the low-level controller
//	1 = PID HP station-keeping
//	2 = PID HP Differential wayfinding
int drive_config = 1;

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

float dt = 0.1;  // [s] used for differential term  // MAKE THIS A FUNCTION OF THE LOOP RATE

float x_goal, y_goal, psi_goal, psi_to_goal;  // [m, m, radians] desired position and heading, and heading to goal pose
float x_goal_prev, y_goal_prev, psi_goal_prev;  // [m, m, radians] previous desired position and heading 
float x_usv_NED, y_usv_NED, psi_usv_NED;  // vehicle position and heading (pose) in NED
float e_x, e_y, e_xy, e_psi;  // current errors between goal pose and usv pose

// initialize accumulated total errors for integral term
float e_x_total = 0;
float e_y_total = 0;
float e_xy_total = 0;
float e_psi_total = 0;
// initialize previous errors for calculating differential term
float e_x_prev = 0;
float e_y_prev = 0;
float e_xy_prev = 0;
float e_psi_prev = 0;

// pose tolerance from path_planner, used for resetting integral terms
float e_xy_allowed;  // positional error tolerance threshold; NOTE: make as small as possible
float e_psi_allowed;  // heading error tolerance threshold; NOTE: make as small as possible

float T_x;  // thrust to set in x-direction in earth-fixed frame
float T_y;  // thrust to set in y-direction in earth-fixed frame
float T_x_bf;  // thrust in x-direction in body-fixed frame
float T_y_bf;  // thrust in y-direction in body-fixed frame
float M_z;  // desired moment around the z-axis

// CONTROL EFFORT TERMS
float T_x_P;  // Proportional term in computing effort in x-translation
float T_x_I;  // Integral term in computing effort in x-translation
float T_x_D;  // Derivative term in computing effort in x-translation

float T_y_P;  // Proportional term in computing effort in y-translation
float T_y_I;  // Integral term in computing effort in y-translation
float T_y_D;  // Derivative term in computing effort in y-translation

float M_z_P;  // Proportional term in computing effort in z-rotation
float M_z_I;  // Integral term in computing effort in z-rotation
float M_z_D;  // Derivative term in computing effort in z-rotation

// thuster outputs
float PM;  // used to set the port (left) main thruster output
float PB;  // used to set the port (left) bow thruster output
float SM;  // used to set the starboard (right) main thruster output
float SB;  // used to set the starboard (right) bow thruster output

// Used for correction of saturated thrust values
float PM_C;  // Corrected port (left) main thruster output
float PB_C;  // Corrected port (left) bow thruster output
float SM_C;  // Corrected starboard (right) main thruster output
float SB_C;  // Corrected starboard (right) bow thruster output

float Kp_x, Kp_y, Kp_psi;  // Proportional gains
float Kd_x, Kd_y, Kd_psi;  // Differential gains
float Ki_x, Ki_y, Ki_psi;  // Integration gains

// USV system id
float B = 2.0;  // [m] FAU FOUND IT TO BE 2.0
float lx = 2.3;  // [m] bf x-offset of thrusters
float ly = 1.0;  // [m] bf y-offset of thrusters
/* 
// Transformation Matrix to convert from external dynamics to internal dynamics
float Transform[3][4] = {
   {1.0, 0.0, 1.0, 0.0} ,
   {0.0, 1.0, 0.0, 1.0} ,
   {1.0, 1.39, -1.0, 1.39}
};

float Transform_transpose[4][3] = {
   {1.0, 0.0, 1.0} ,
   {0.0, 1.0, 1.39} ,
   {1.0,0.0, -1.0} ,
   {0.0,1.0, 1.39}
};
}; */

// this is the pseudoinverse of the transform matrix from
// forces experienced directly at the thrusters to forces experienced at the USV COG
// Transform_pseudoinverse = Transform_transpose * inv(Transform*Transform_transpose)
float Transform_pseudoinverse[4][3] = {
   {0.5, -0.695, 0.5} ,
   {0.0, 0.5, 0.0} ,
   {0.5, 0.695, -0.5} ,
   {0.0, 0.5, 0.0}
};

std_msgs::Bool PS_initialization_state_msg;  // "PS_initialization_state" message
ros::Publisher PS_initialization_state_pub;  // "PS_initialization_state" publisher

amore::control_efforts PS_control_efforts_topic_msg;  // "PS_control_efforts_topic" message
ros::Publisher PS_control_efforts_topic_pub;  // "PS_control_efforts_topic" publisher

ros::Time current_time, last_time;  // creates time variables
//..........................................................................................................End of Global Variables........................................................................................................

//.....................................................................................................................Functions.......................................................................................................................
// THIS FUNCTION: Updates global current_time, loop_count, and publishes initialization status to "PS_initialization_state"
// ACCEPTS: (VOID)
// RETURNS: (VOID)
//=============================================================================================================
void PROPULSION_SYSTEM_inspector()
{
	current_time = ros::Time::now();  // sets current_time to the time it is now
	loop_count += 1;  // increment loop counter
	if (loop_count > 3)
	{
		PS_initialization_state_msg.data = true;
		//ROS_INFO("PROPULSION_SYSTEM: propulsion_system_initialized");
	}
	else
	{
		PS_initialization_state_msg.data = false;
		//ROS_INFO("PROPULSION_SYSTEM: !propulsion_system_initialized");
	}
	PS_initialization_state_pub.publish(PS_initialization_state_msg);  // publish the initialization status of the propulsion_system to "PS_initialization_state"
} // END OF PROPULSION_SYSTEM_inspector()

// THIS FUNCTION: Publishes control "efforts" to "PS_control_efforts_topic"
// ACCEPTS: (VOID)
// RETURNS: (VOID)
//=============================================================================================================
void publish_control_efforts()
{
	// first, fill out control_efforts message
	PS_control_efforts_topic_msg.t_x.data = T_x;
	PS_control_efforts_topic_msg.t_x_P.data = T_x_P;
	PS_control_efforts_topic_msg.t_x_I.data = T_x_I;
	PS_control_efforts_topic_msg.t_x_D.data = T_x_D;
	
	PS_control_efforts_topic_msg.t_y.data = T_y;
	PS_control_efforts_topic_msg.t_y_P.data = T_y_P;
	PS_control_efforts_topic_msg.t_y_I.data = T_y_I;
	PS_control_efforts_topic_msg.t_y_D.data = T_y_D;
	
	PS_control_efforts_topic_msg.m_z.data = M_z;
	PS_control_efforts_topic_msg.m_z_P.data = M_z_P;
	PS_control_efforts_topic_msg.m_z_I.data = M_z_I;
	PS_control_efforts_topic_msg.m_z_D.data = M_z_D;
	PS_control_efforts_topic_pub.publish(PS_control_efforts_topic_msg);  // publish "PS_control_efforts_topic"
} // END OF publish_control_efforts()

// THIS FUNCTION: Updates the state of "propulsion_system" given by "mission_control"
// ACCEPTS: amore::state from "MC_ps_state"
// RETURNS: (VOID)
//=============================================================================================================
void MC_ps_state_update(const amore::state::ConstPtr& msg) 
{
	if (PS_initialization_state_msg.data)
	{
		PS_state = msg->state.data;
		//ROS_INFO("PROPULSION_SYSTEM: PS_state = %i", PS_state);
	}
	if ((!PS_state_ON) && (PS_state == 1))  // if PS_state_ON oneshot is false and the propulsion_system is told to be "ON" by mission_control
	{
		PS_state_ON = true;  // PS_state_ON oneshot is true
		loop_new_goal = loop_count;  // update the loop_new_goal
	}
	else if (PS_state == 0)  // if the propulsion_system is told to be "On standby" by mission_control
	{
		PS_state_ON = false;
	}
} // END OF MC_ps_state_update()

// THIS FUNCTION: Updates the state of "path_planner" given by "mission_control"
// ACCEPTS: amore::state from "MC_pp_state"
// RETURNS: (VOID)
//=============================================================================================================
void MC_pp_state_update(const amore::state::ConstPtr& msg)
{
	if (PS_initialization_state_msg.data)
	{
		PP_state = msg->state.data;
	}
}  // END OF MC_pp_state_update()

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
} // END OF NA_nav_ned_update()

// THIS FUNCTION: Updates the current goal and usv poses for the propulsion_system given by path_planner
// ACCEPTS: propulsion_system from "PP_propulsion_system_topic"
// RETURNS: (VOID)
//=============================================================================================================
void PP_propulsion_system_topic_update(const amore::propulsion_system::ConstPtr& topic) 
{
	// update USV pose
	x_usv_NED = topic->usv_position.x;
	y_usv_NED = topic->usv_position.y;
	psi_usv_NED = topic->usv_psi.data;
	// update goal pose
	x_goal = topic->goal_position.x;
	y_goal = topic->goal_position.y;
	psi_goal = topic->goal_psi.data;
	// reset the integral terms if a new goal is fed
	if ((x_goal != x_goal_prev) || (y_goal != y_goal_prev) || (psi_goal != psi_goal_prev))
	{
		ROS_INFO("PROPULSION_SYSTEM: Resetting integral terms since new goal is aqcuired.");
		loop_new_goal = loop_count;  // update the loop_new_goal
		e_x_total = 0.0;
		e_y_total = 0.0;
		e_xy_total = 0.0;
		e_psi_total = 0.0;
	}
	// update pose error tolerances
	e_xy_allowed = topic->e_xy_allowed.data;
	e_psi_allowed = topic->e_psi_allowed.data;
	drive_config = topic->drive_configuration.data;
}  // END OF PP_propulsion_system_topic_update()

// THIS FUNCTION: Displays the updated gains
// ACCEPTS: (VOID)
// RETURNS: (VOID)
//=============================================================================================================
void display_gains()
{
	ROS_INFO("PROPULSION_SYSTEM:--------GAINS--------");
	// PROPORTIONAL GAINS
	ROS_INFO("PROPULSION_SYSTEM:    Kp_xy  :  %.2f", Kp_x);
	ROS_INFO("PROPULSION_SYSTEM:    Kp_psi :  %.2f", Kp_psi);
	// DERIVATIVE GAINS
	ROS_INFO("PROPULSION_SYSTEM:    Kd_xy  :  %.2f", Kd_x);
	ROS_INFO("PROPULSION_SYSTEM:    Kd_psi :  %.2f", Kd_psi);
	// INTEGRAL GAINS
	ROS_INFO("PROPULSION_SYSTEM:    Ki_xy  :  %.2f", Ki_x);
	ROS_INFO("PROPULSION_SYSTEM:    Ki_psi :  %.2f", Ki_psi);
	ROS_INFO("PROPULSION_SYSTEM:--------GAINS--------\n");
}  // END OF display_gains()

// THIS FUNCTION: Updates the gains
// ACCEPTS: (VOID) 
// RETURNS: (VOID) 
//=============================================================================================================
void update_gains_LL_controller()
{
	if (PS_state == 1)
	{
		// GET ALL GAIN PARAMETERS FROM LAUNCH FILE
		// get P-gains
		ros::param::get("/kp_xy", Kp_x);
		Kp_y = Kp_x; //ros::param::get("/kp_y", Kp_y);
		ros::param::get("/kp_psi", Kp_psi);
		// get I-gains
		ros::param::get("/ki_xy", Ki_x);
		Ki_y = Ki_x; //::param::get("/ki_y", Ki_y);
		ros::param::get("/ki_psi", Ki_psi);
		// get D-gains
		ros::param::get("/kd_xy", Kd_x);
		Kd_y = Kd_x; //ros::param::get("/kd_y", Kd_y);
		ros::param::get("/kd_psi", Kd_psi);
		if (PP_state == 11)  // VRX1: Station-Keeping path_planner
		{
			display_gains();  // DO NOT COMMENT THIS LINE IF YOU WANT TO PRINT GAINS TO USER
		}
	}
}  // END OF update_gains_LL_controller()

// THIS FUNCTION: Calculates pose errors
// ACCEPTS: (VOID)
// RETURNS: (VOID)
//=============================================================================================================
void calculate_pose_errors()
{
	e_x = x_goal - x_usv_NED;  // calculate error in x position
	e_y = y_goal - y_usv_NED;  // calculate error in y position
	e_xy = sqrt(pow(e_x,2.0)+pow(e_y,2.0));  // calculate magnitude of positional error
	e_psi = psi_goal - psi_usv_NED;  // calculate error in heading
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

// THIS FUNCTION: Resets the integral term once the errors become minimal 
//	to avoid overshooting the goal if a huge effort's built up
// ACCEPTS: (VOID) 
// RETURNS: (VOID) 
//=============================================================================================================
void Integral_reset()
{
	if ((float)abs(e_x) > 1.0)  // NOTE: this value may need adjusting
	//if ( ((float)abs(e_x) <= 0.1) || ((float)abs(e_x) > 1.0) )  // PERHAPS TRY THIS INSTEAD
	{
	  e_x_total = 0.0;
	}
	if ((float)abs(e_y) > 1.0)  // NOTE: this value may need adjusting
	//if ( ((float)abs(e_y) <= 0.1) || ((float)abs(e_y) > 1.0) )  // PERHAPS TRY THIS INSTEAD
	{
	  e_y_total = 0.0;
	}
	if ((float)abs(e_xy) > 1.0)
	{
	  e_xy_total = 0.0;
	}
	//if ( ((float)abs(e_xy) <= e_xy_allowed) || ((float)abs(e_xy) > 1.0) )
	//{
	  //e_xy_total = 0.0;
	  //e_x_total = 0.0;
	  //e_y_total = 0.0;
	//}
	//if ((float)abs(e_psi) >= PI/2)  // NOTE: this value may need adjusting 
	//if (((float)abs(e_psi) < 0.05) || (e_xy > 3.0))  // PERHAPS TRY THIS INSTEAD
	if ((float)abs(e_psi) <= e_psi_allowed)
	{
	  e_psi_total = 0.0;
	}
}  // END OF Integral_reset()

// THIS FUNCTION: Checks for saturation of thrust outputs and corrects if so
// ACCEPTS: (VOID)
// RETURNS: (VOID)
//=============================================================================================================
void thrust_saturation_check()
{
	float abs_PM = (float)abs(PM);
	float abs_PB = (float)abs(PB);
	float abs_SM = (float)abs(SM);
	float abs_SB = (float)abs(SB);
	float max_thrust = std::max({abs_PM, abs_PB, abs_SM, abs_SB});
	//ROS_INFO("%f     %f     %f     %f", abs_PM, abs_PB, abs_SM, abs_SB);
	//ROS_INFO("The max thrust is %f\n", max_thrust);
	// check for saturation
	if (max_thrust > 1.0)
	{
		PM_C = PM / max_thrust;
		PB_C = PB / max_thrust;
		SM_C = SM / max_thrust;
		SB_C = SB / max_thrust;
		PM = (int)PM_C * 100;
		PB = (int)PB_C * 100;
		SM = (int)SM_C * 100;
		SB = (int)SB_C * 100;
		ROS_WARN("PROPULSION_SYSTEM:----STANDARDIZED THRUSTS----");
		ROS_WARN("PROPULSION_SYSTEM: PORT_MAIN: %4.2f    PORT_BOW: %4.2f    STBD_MAIN: %4.2f    STBD_BOW: %4.2f\n", PM, PB, SM, SB);
	}
	else
	{
		PM = (int)PM * 100;
		PB = (int)PB * 100;
		SM = (int)SM * 100;
		SB = (int)SB * 100;
	}
}  // END OF thrust_saturation_check()
//.............................................................................................................END OF Functions...............................................................................................................

//..................................................................................................................Main Program..................................................................................................................
int main(int argc, char **argv)
{
	ros::init(argc, argv, "propulsion_system");  // names the program for visual purposes

	// NodeHandles
	ros::NodeHandle nh1, nh2, nh3, nh4, nh7;

	// Subscribers
	// from mission_control
	ros::Subscriber MC_ps_state_sub = nh1.subscribe("MC_ps_state", 1, MC_ps_state_update);
	ros::Subscriber MC_pp_state_sub = nh2.subscribe("MC_pp_state", 1, MC_pp_state_update);
	// from path_planner
	ros::Subscriber PP_propulsion_system_topic_sub = nh3.subscribe("PP_propulsion_system_topic", 1, PP_propulsion_system_topic_update);  // path_planner directions update
	// from navigation_array
	ros::Subscriber NA_nav_ned_sub = nh4.subscribe("NA_nav_ned", 1, NA_nav_ned_update);  // Obtains the USV pose in local NED

	// Publishers
	// to mission_control
	PS_initialization_state_pub = nh7.advertise<std_msgs::Bool>("PS_initialization_state", 1);  // state of initialization
	// to the user
	PS_control_efforts_topic_pub = nh7.advertise<amore::control_efforts>("PS_control_efforts_topic", 1);  // control_efforts

	// Simulation thruster topics 
	ros::Publisher right_thrust_cmd_pub = nh7.advertise<std_msgs::Float32>("/wamv/thrusters/right_thrust_cmd", 10);  // between -1.0 and 1.0, speed to right thruster
	ros::Publisher left_thrust_cmd_pub = nh7.advertise<std_msgs::Float32>("/wamv/thrusters/left_thrust_cmd", 10);  // value between -1.0 and 1.0, speed to left thruster
	ros::Publisher right_thrust_angle_pub = nh7.advertise<std_msgs::Float32>("/wamv/thrusters/right_thrust_angle", 10);  // value between -PI to PI, angle to right thruster
	ros::Publisher left_thrust_angle_pub = nh7.advertise<std_msgs::Float32>("/wamv/thrusters/left_thrust_angle", 10);  // value between -PI to PI, angle to left thruster
	// Simulation variables
	std_msgs::Float32 left_thrust_cmd_msg, right_thrust_cmd_msg, left_thrust_angle_msg, right_thrust_angle_msg;

	// Initialize global variables
	PS_initialization_state_msg.data = false;
	current_time = ros::Time::now();  // sets current time to the time it is now

	ros::Rate loop_rate((int)1/dt);  // sets the frequency for which the program tries to loop per second

	while(ros::ok())  // ros::ok() will be false when the user inputs Ctrl+C
	{
		PROPULSION_SYSTEM_inspector();  // check initialization status and update PS_initialization_state_msg

		if (PS_state == 1)
		{
			ros::spinOnce();  // update subscribers to get most up to date goal and USV poses
			update_gains_LL_controller();  // Update all gain parameters in launch file

			Integral_reset();  // Reset integral term once the errors become minimal 
			calculate_pose_errors();  // calculate the errors in position and heading (e_x, e_y, e_xy, e_psi)

			/* // if within error, have selective gains
			if (e_xy < 1.0) 
			{
				Kp_x = 0.7;
				Kp_y = 0.7;
				Kd_x = 0.2;
				Kd_y = 0.2;
				Ki_x = 0.0;
				Ki_y = 0.0;
			} */

			// position control law
			if ( (loop_count >= loop_new_goal) && (loop_count < (loop_new_goal + 2)) )  // don't include differential term or integration term first 2 loops after being turned ON
			{
				// USE P CONTROLLER SINCE I AND D TERMS ARE NOT YET DEFINED THE LOOP AFTER NEW GOAL IS ACHIEVED
				if (drive_config == 1)  // 1 = PID HP station-keeping
				{
					// effort in x-translation
					T_x_P = Kp_x*e_x;  // P-term
					T_x_I = 0.0;  // I-term
					T_x_D = 0.0;  // D-term
					T_x = T_x_P;  // only P-term
					// effort in y-translation
					T_y_P = Kp_y*e_y;  // P-term
					T_y_I = 0.0;  // I-term
					T_y_D = 0.0;  // D-term
					T_y = T_y_P;  // only P-term
				}
				else if (drive_config == 2)  // 2 = PID HP Differential wayfinding
				{
					// effort in xy-translation
					T_x_P = Kp_x*e_xy;  // P-term
					T_x_I = 0.0;  // I-term
					T_x_D = 0.0;  // D-term
					T_x = T_x_P;  // only P-term
					
					// FILL OUT THE Y-TRANSLATION "EFFORTS" FOR USER FRIENDLINESS
					// effort in y-translation
					T_y_P = 0.0;  // P-term
					T_y_I = 0.0;  // I-term
					T_y_D = 0.0;  // D-term
					T_y = T_y_P + T_y_I + T_y_D;  // PID terms
				}
			}
			else if (loop_count >= (loop_new_goal + 2))
			{
				// USE PID CONTROLLER ONCE I AND D TERMS ARE DEFINED
				if (drive_config == 1)  // 1 = PID HP station-keeping
				{
					// trapezoidal integration of errors for integral term
					e_x_total = e_x_total + ((e_x_prev + e_x)/2.0)*dt;
					e_y_total = e_y_total + ((e_y_prev + e_y)/2.0)*dt;
					// effort in x-translation
					T_x_P = Kp_x*e_x;  // P-term
					T_x_I = Ki_x*e_x_total;  // I-term
					T_x_D = Kd_x*((e_x - e_x_prev)/dt);  // D-term
					T_x = T_x_P + T_x_I + T_x_D;  // PID terms
					// effort in y-translation
					T_y_P = Kp_y*e_y;  // P-term
					T_y_I = Ki_y*e_y_total;  // I-term
					T_y_D = Kd_y*((e_y - e_y_prev)/dt);  // D-term
					T_y = T_y_P + T_y_I + T_y_D;  // PID terms
				}
				else if (drive_config == 2)  // 2 = PID HP Differential wayfinding
				{
					e_xy_total = e_xy_total + ((e_xy_prev + e_xy)/2.0)*dt;  // trapezoidal integration of errors for integral term
					// effort in xy-translation
					T_x_P = Kp_x*e_xy;  // P-term
					T_x_I = Ki_x*e_xy_total;  // I-term
					T_x_D = Kd_x*((e_xy - e_xy_prev)/dt);  // D-term
					T_x = T_x_P + T_x_I + T_x_D;  // PID terms
					
					// FILL OUT THE Y-TRANSLATION "EFFORTS" FOR USER FRIENDLINESS
					// effort in y-translation
					T_y_P = 0.0;  // P-term
					T_y_I = 0.0;  // I-term
					T_y_D = 0.0;  // D-term
					T_y = T_y_P + T_y_I + T_y_D;  // PID terms
				}
			}  // END OF position control law

			// PI WRAP?
			// correct discontinuity in heading error
			if (e_psi < (-PI + 0.1*PI))
			{
				e_psi += 2.0*PI;
			}
			if (e_psi > (PI - 0.1*PI))
			{
				e_psi -= 2.0*PI;
			}

			/* // if within error, have selective gains
			if ((float)abs(e_psi) < 0.1)
			{
				Kp_psi = 0.0;
				Kd_psi = 0.0;
				Ki_psi = 0.0;
			} */

			// orientation control law
			if ( (loop_count >= loop_new_goal) && (loop_count < (loop_new_goal + 2)) )  // don't include differential term or integration term first time through
			{
				// USE P TERM ONLY TO COMPUTE CONTROL EFFORT SINCE I AND D TERMS ARE NOT YET DEFINED THE LOOP AFTER CONTROLLER IS TURNED ON
				// effort in z-rotation
				M_z_P = Kp_psi*e_psi;  // P-term
				M_z_I = 0.0;  // I-term
				M_z_D = 0.0;  // D-term
				M_z = M_z_P;  // only P-term
			}
			else if (loop_count >= (loop_new_goal + 2))
			{
				// USE PID CONTROLLER ONCE I AND D TERMS ARE DEFINED
				e_psi_total = e_psi_total + ((e_psi_prev + e_psi)/2.0)*dt;  // trapezoidal integration of error for integral term
				// effort in z-rotation
				M_z_P = Kp_psi*e_psi;  // P-term
				M_z_I = Ki_psi*e_psi_total;  // I-term
				M_z_D = Kd_psi*((e_psi - e_psi_prev)/dt);  // D-term
				M_z = M_z_P + M_z_I + M_z_D;  // PID terms
			}  // END OF orientation control law

			/* // ROS_INFO("PROPULSION_SYSTEM:---GOAL POSE---");  // UPDATE USER FROM THE CAMELS MOUTH
			// ROS_INFO("PROPULSION_SYSTEM: x_goal: %.2f", x_goal);
			// ROS_INFO("PROPULSION_SYSTEM: y_goal: %.2f", y_goal);
			// ROS_INFO("PROPULSION_SYSTEM: psi_goal: %.2f", psi_goal);
			// ROS_INFO("PROPULSION_SYSTEM:---GOAL POSE---\n");

			// ROS_INFO("PROPULSION_SYSTEM:---USV POSE---");  // UPDATE USER
			// ROS_INFO("PROPULSION_SYSTEM: x_usv: %.2f", x_usv_NED);
			// ROS_INFO("PROPULSION_SYSTEM: y_usv: %.2f", y_usv_NED);
			// ROS_INFO("PROPULSION_SYSTEM: psi_usv: %.2f", psi_usv_NED);
			// ROS_INFO("PROPULSION_SYSTEM:---USV POSE---\n");

			// ROS_INFO("PROPULSION_SYSTEM:--------ERRORS--------");  // UPDATE USER
			// ROS_INFO("PROPULSION_SYSTEM:     e_x   :  %4.2f", e_x);  // x posn. error
			// ROS_INFO("PROPULSION_SYSTEM:     e_y   :  %4.2f", e_y);  // y posn. error
			// ROS_INFO("PROPULSION_SYSTEM:     e_xy  :  %4.2f", e_xy);  // magnitude of posn. error
			// ROS_INFO("PROPULSION_SYSTEM:     e_psi :  %4.2f", e_psi);  // heading error
			// ROS_INFO("PROPULSION_SYSTEM:--------ERRORS--------\n"); */

			publish_control_efforts();  // fill out control_efforts message and publish to "PS_control_efforts_topic"

			/* // ROS_DEBUG("PROPULSION_SYSTEM:-----Control Efforts-----");
			// ROS_DEBUG("PROPULSION_SYSTEM: T_x: %.2f", T_x);
			// ROS_DEBUG("PROPULSION_SYSTEM: T_y: %.2f", T_y);
			// ROS_DEBUG("PROPULSION_SYSTEM: M_z: %.2f", M_z);
			// ROS_DEBUG("PROPULSION_SYSTEM:-----Control Efforts-----\n"); */

			// only control heading if off by more than 60 degrees
			if ((float)abs(e_psi) > 1.047)
			{
				ROS_INFO("PROPULSION_SYSTEM: Zeroing ""efforts"" to fix position since heading is off by more than 60 degrees.");
				T_x = 0.0;
				T_y = 0.0;
			}

			/* // EXPERIMENTING WITH THIS
			// THE FOLLOWING BLOCK SHOULD NOT BE TRIED UNLESS ITS A CALM DAY PERHAPS AND DISTURBANCES ARE MINIMAL
			// if within half a meter only control heading
			if ((float)abs(e_xy) < 0.5) 
			{
				ROS_INFO("PROPULSION_SYSTEM: Zeroing ""efforts"" to fix position since within half meter.");
				T_x = 0.0;
				T_y = 0.0;
			}

			// if errors are small enough, do not try to correct for them
			if ((float)abs(e_x) < 0.1)
			{
				T_x = 0.0;
			}
			if ((float)abs(e_y) < 0.1)
			{
				T_y = 0.0;
			}
			if ((float)abs(e_psi) < 0.1)
			{
				M_z = 0.0;
			} */

			// ALLOCATION to go from control_efforts to thruster commands
			//ROS_INFO("PROPULSION_SYSTEM:-----Dual-azimuthing station-keeping controller-----\n");
			// Convert to USV body-fixed frame from global frame
			T_x_bf = T_x*cos(psi_usv_NED) + T_y*sin(psi_usv_NED);
			T_y_bf = T_y*cos(psi_usv_NED) - T_x*sin(psi_usv_NED);

			// calculate the control allocation outputs
			// f = Transform_pseudoinverse * tau;
			PM = Transform_pseudoinverse[0][0]*T_x_bf + Transform_pseudoinverse[0][1]*T_y_bf + Transform_pseudoinverse[0][2]*M_z;
			PB = Transform_pseudoinverse[1][0]*T_x_bf + Transform_pseudoinverse[1][1]*T_y_bf + Transform_pseudoinverse[1][2]*M_z;
			SM = Transform_pseudoinverse[2][0]*T_x_bf + Transform_pseudoinverse[2][1]*T_y_bf + Transform_pseudoinverse[2][2]*M_z;
			SB = Transform_pseudoinverse[3][0]*T_x_bf + Transform_pseudoinverse[3][1]*T_y_bf + Transform_pseudoinverse[3][2]*M_z;

			// DEBUG INFORMATION ////////////////////////////////////////////////////////////
			// Print thrust control efforts in x and y directions in both the local frame (working frame) and the body-fixed frame (USV frame)
			// ROS_DEBUG("PROPULSION_SYSTEM: BEFORE SWAP TO BODY-FIXED FRAME");
			// ROS_DEBUG("PROPULSION_SYSTEM: T_x_G: %f", T_x);
			// ROS_DEBUG("PROPULSION_SYSTEM: T_y_G: %f", T_y);
			// ROS_DEBUG("PROPULSION_SYSTEM: AFTER SWAP");
			// ROS_DEBUG("PROPULSION_SYSTEM: T_x_bf: %f", T_x_bf);
			// ROS_DEBUG("PROPULSION_SYSTEM: T_y_bf: %f\n", T_y_bf);

			// Print control allocation outputs before saturation check
			ROS_DEBUG("PROPULSION_SYSTEM: BEFORE THRUSTER SATURATION CHECK");
			ROS_DEBUG("PROPULSION_SYSTEM: PM: %f", PM);
			ROS_DEBUG("PROPULSION_SYSTEM: PB: %f", PB);
			ROS_DEBUG("PROPULSION_SYSTEM: SM: %f", SM);
			ROS_DEBUG("PROPULSION_SYSTEM: SB: %f\n", SB);

			// adjust thruster outputs to ensure they are in the expected range
			thrust_saturation_check();
			
			// update previous errors
			e_x_prev = e_x;
			e_y_prev = e_y;
			e_xy_prev = e_xy;
			e_psi_prev = e_psi;
			// update previous goal
			x_goal_prev = x_goal;
			y_goal_prev = y_goal;
			psi_goal_prev = psi_goal;
		}  // END OF if (PS_state == 1)
		else if (PS_state == 0)  // Controller is told to be On standby by command from mission_control
		{
			// set thruster speeds to zero
			PM = 0;
			PB = 0;
			SM = 0;
			SB = 0;
		}  // END OF else if (PS_state == 0)
		else  // if PS_state has not yet been recieved from mission_control
		{
			ROS_WARN("PROPULSION_SYSTEM:---WAITING TO RECIEVE STATE---\n");
		}
		
		// ENTER SERIAL COMMUNICATION PROTOCOL FUNCTION HERE
		// SEND OUT THE THRUSTER COMMANDS NO SLOWER THAN 10 HERTZ
		// set thruster message commands
		left_thrust_angle_msg.data = PB;
		left_thrust_cmd_msg.data = PM;
		right_thrust_angle_msg.data = SB;
		right_thrust_cmd_msg.data = SM;
		// publish thruster message commands
		left_thrust_angle_pub.publish(left_thrust_angle_msg);
		left_thrust_cmd_pub.publish(left_thrust_cmd_msg);
		right_thrust_angle_pub.publish(right_thrust_angle_msg);
		right_thrust_cmd_pub.publish(right_thrust_cmd_msg);
		
		ros::spinOnce();  // update subscribers
		loop_rate.sleep();  // sleep to accomplish set loop_rate
		last_time = current_time;  // update last_time
	}  // END OF while(ros::ok())

	// set thruster speeds to zero
	PM = 0;
	PB = 0;
	SM = 0;
	SB = 0;
	// ENTER SERIAL COMMUNICATION PROTOCOL FUNCTION HERE
	// set thruster message commands
	left_thrust_angle_msg.data = PB;
	left_thrust_cmd_msg.data = PM;
	right_thrust_angle_msg.data = SB;
	right_thrust_cmd_msg.data = SM;
	// publish thruster message commands
	left_thrust_angle_pub.publish(left_thrust_angle_msg);
	left_thrust_cmd_pub.publish(left_thrust_cmd_msg);
	right_thrust_angle_pub.publish(right_thrust_angle_msg);
	right_thrust_cmd_pub.publish(right_thrust_cmd_msg);

	return 0;
}  // END OF main()
//.........................................................................................................END OF Main Program...........................................................................................................