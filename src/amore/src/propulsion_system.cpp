//  Filename:  propulsion_system.cpp
//  Creation Date:  1/31/2022
//  Last Revision Date:  8/17/2022
//  Author(s) [email]:  Brad Hacker [bhacker@lssu.edu]
//                                                  
//  Revisor(s) [Revision Date]:
//  Organization/Institution:  Lake Superior State University RobotX Team AMORE
// 
// ...........................propulsion_system.cpp.......................
//  This code recieves a goal pose to reach. It then calculates the errors and control efforts then uses control allocation to convert the efforts to outputs.in the global NED frame as a usv_pose_msg given by path_planner. 
//  It gets the goal pose by subscribing to the "current_goal_pose" node published by path_planner.
//  This code then uses PID control theory to control the heading and position control efforts of the USV.
//  The control effort is then converted to thrusts and angles on each thruster to be published to "/gazebo".
//  Currently, this code is set up to use a dual-azimuthing drive configuration. Having 4 actuators: 
//  port thrust and angle and starboard thrust and angle to control makes the system over-actuated.
//  For station-keeping three degrees of freedom must be controlled, position on a plane (x,y - long/lat)
//  and the heading of the USV. Since the system is over-actuated, control allocation is used to solve for 
//  the thrusts and azimuthing angles of each thruster.
//
//  Inputs and Outputs of the propulsion_system.cpp file
//				Inputs: ["PP_propulsion_system_topic" - amore/propulsion_system - goal pose (x,y,psi) to reach in local NED frame and current USV pose from path_planner]
//
//				Outputs: "thruster_int_right", "thuster_int_left", "angle_int_right", "angle_int_left" - dual-azimuthing Minn Kota thruster commands

//...............................................................................................Included Libraries and Message Types.........................................................................................
#include "ros/ros.h"
#include "ros/console.h"
#include "time.h"
#include <sstream>
#include <iostream>
#include "math.h"
#include "stdio.h"
#include "nav_msgs/Odometry.h"  // message type used for receiving USV state from navigation_array
#include "std_msgs/Bool.h"  // message type used for communicating initialization status to mission_control
#include "amore/state.h"  // message type used to recieve state of operation from mission_control
#include "amore/propulsion_system.h"  // message type that holds all needed operation information from path_planner
#include "std_msgs/Float32.h"  // message type of thruster commands, and type for control efforts
#include "amore/control_efforts.h"  // message type that holds thrust x, thrust y, and moment z
//......................................................................................End of Included Libraries and Message Types.....................................................................................

//.......................................................................................................................Constants.....................................................................................................................
#define PI 3.14159265
//................................................................................................................End of Constants...............................................................................................................

//.................................................................................................................Global Variables..............................................................................................................
int loop_count = 0;  // loop counter
int loop_count_ON = 0;  // loop count holder for when the the controller is turned on, used to ensure differential and integral terms are not started until they are calculated
bool PS_state_ON = false;  // used to set and reset the above count holder like a oneshot
	
//	STATES CONCERNED WITH "propulsion_system"
int PS_state = 0;
//	0 = On standby
//	1 = Propulsion system ON
int LL_state = 1;
//	1 = PID HP Dual-azimuthing station keeping controller
//	2 = PID HP Differential wayfinding controller
//	3 = PID HP Ackermann controller

float dt = 0.25;  // [s] used for differential term  // MAKE THIS A FUNCTION OF THE LOOP RATE 

float x_goal, y_goal, psi_goal;  // [m, m, radians] desired position and heading
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

// matrix to hold outputs of controlller
//float tau[3][1];

float T_p;  // used to set the port (left) thruster output
float T_s;  // used to set the starboard (right) thruster output
float A_p;  // used to set the port (left) thruster angle
float A_s;  // used to set the starboard (right) thruster angle

// Used for correction of saturated thrust values 
float T_p_C;  // Corrected port (left) thrust output
float T_s_C;  // Corrected starboard (right) thrust output

float Kp_x, Kp_y, Kp_psi;  // Proportional gains
float Kd_x, Kd_y, Kd_psi;  // Differential gains
float Ki_x, Ki_y, Ki_psi;  // Integration gains

// USV system id
float B = 2.0;  // [m] FAU FOUND IT TO BE 2.0
float lx = 2.3;  // [m] bf x-offset of thrusters
float ly = 1.0;  // [m] bf y-offset of thrusters
/* 
float L = 4.6; // [m] length of USV, 16 ft or 4.88 m

// Transformation Matrix to convert from external dynamics to internal dynamics
float Transform[3][4] = {
   {1.0, 0.0, 1.0, 0.0} ,
   {0.0, 1.0, 0.0, 1.0} ,
   {1.0, -2.3, -1.0, -2.3}
};

float Transform_transpose[4][3] = {
   {1.0, 0.0, 1.0} ,
   {0.0, 1.0, -2.3} ,
   {1.0,0.0, -1.0} ,
   {0.0,1.0, -2.3}
};
}; */

// this is the pseudoinverse of the transform matrix from
// forces experienced directly at the thrusters to forces experienced at the USV COG
// Transform_pseudoinverse = Transform_transpose * inv(Transform*Transform_transpose)
float Transform_pseudoinverse[4][3] = {
   {0.5, 1.15, 0.5} ,
   {0.0, 0.5, 0.0} ,
   {0.5, -1.15, -0.5} ,
   {0.0, 0.5, 0.0}
};

// Control Allocation output
float F_xp;  // force in the x-direction on the port side
float F_yp;  // force in the y-direction on the port side
float F_xs;  // force in the x-direction on the starboard side
float F_ys;  // force in the y-direction on the starboard side

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
		loop_count_ON = loop_count;  // update the loop_count_ON
	}
	else if (PS_state == 0)  // if the propulsion_system is told to be "On standby" by mission_control
	{
		PS_state_ON = false;
	}
} // END OF MC_ps_state_update()

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
} // END OF display_gains()

// THIS FUNCTION: Updates the gains
// ACCEPTS: (VOID) 
// RETURNS: (VOID) 
//=============================================================================================================
void update_gains_LL_controller()
{
	if (PS_state == 1)
	{
		// GET ALL GAIN PARAMETERS FROM LAUNCH FILE
		ros::param::get("/Kp_xy_G", Kp_x);
		Kp_y = Kp_x; //ros::param::get("/Kp_y_G", Kp_y);
		ros::param::get("/Kp_psi_G", Kp_psi);
		ros::param::get("/Kd_xy_G", Kd_x);
		Kd_y = Kd_x; //ros::param::get("/Kd_y_G", Kd_y);
		ros::param::get("/Kd_psi_G", Kd_psi);
		ros::param::get("/Ki_xy_G", Ki_x);
		Ki_y = Ki_x; //::param::get("/Ki_y_G", Ki_y);
		ros::param::get("/Ki_psi_G", Ki_psi);
		display_gains();  // DO NOT COMMENT OUT THIS LINE IF YOU WANT TO PRINT GAINS TO USER 
	}
} // END OF update_gains_LL_controller()

// THIS FUNCTION: Resets the integral term once the errors become minimal 
//	to avoid overshooting the goal if a huge effort's built up
// ACCEPTS: (VOID) 
// RETURNS: (VOID) 
//=============================================================================================================
void Integral_reset()
{
	if ( ((float)abs(e_x) < 0.1) || ((float)abs(e_x) > 1.0) )  // NOTE: this value may need adjusting
	{
	  e_x_total = 0.0;
	}
	if ( ((float)abs(e_y) < 0.1) || ((float)abs(e_y) > 1.0) )  // NOTE: this value may need adjusting
	{
	  e_y_total = 0.0;
	}
	if ( ((float)abs(e_xy) < 0.5) || ((float)abs(e_xy) > 1.0) )
	{
	  e_xy_total = 0.0;
	}
	if ((float)abs(e_psi) < 0.2)  // NOTE: this value may need adjusting
	{
	  e_psi_total = 0.0;
	}
} // END OF Integral_reset()

// THIS FUNCTION: Checks that angles are between -PI/2 and PI/2 and corrects accordingly
// ACCEPTS: (VOID)
// RETURNS: (VOID)
//=============================================================================================================
void angle_correction_check()
{
	while ((A_p > PI/2.0) || (A_p < -PI/2.0) || (A_s > PI/2.0) || (A_s < -PI/2.0))
	{
		// Angles to thrusters can only be set between -PI/2 and PI/2
		if (A_p > PI/2.0)
		{
			A_p = A_p - PI;
			T_p = -1.0 * T_p;
		}
		else if (A_p < -PI/2.0)
		{
			A_p = A_p + PI;
			T_p = -1.0 * T_p;
		}
		if (A_s > PI/2.0)
		{
			A_s = A_s - PI;
			T_s = -1.0 * T_s;
		}
		else if (A_s < -PI/2.0)
		{
			A_s = A_s + PI;
			T_s = -1.0 * T_s;
		}
	}
} // END OF angle_correction_check()

// THIS FUNCTION: Checks for saturation of thrust outputs and corrects if so
// ACCEPTS: (VOID) 
// RETURNS: (VOID) 
//=============================================================================================================
void thrust_saturation_check()
{
	// check for saturation
	if (((float)abs(T_p) > 1.0) || ((float)abs(T_s) > 1.0))
	{
		// correct for saturation by normalizing thrust data
		if ((float)abs(T_p) > (float)abs(T_s))
		{
		  // ROS_DEBUG("PROPULSION_SYSTEM: LT IS ISSUE!");
		  T_p_C= T_p / (float)abs(T_p);
		  T_s_C = T_s / (float)abs(T_p);
		}
		else if ((float)abs(T_p) < (float)abs(T_s))
		{
		  // ROS_DEBUG("PROPULSION_SYSTEM: RT IS ISSUE!");
		  T_p_C = T_p / (float)abs(T_s);
		  T_s_C = T_s / (float)abs(T_s);
		}
		else 
		{
		  // ROS_DEBUG("PROPULSION_SYSTEM: Equal");
		  // ROS_DEBUG("PROPULSION_SYSTEM: Divide by : %f\n", (float)abs(T_p));  // displays right thrust value
		  T_p_C = T_p / (float)abs(T_p);
		  T_s_C = T_s / (float)abs(T_p);
		}
		T_p = T_p_C;
		T_s = T_s_C;
		ROS_WARN("PROPULSION_SYSTEM:----CORRECTED THRUSTS----");
		ROS_WARN("PROPULSION_SYSTEM: Port: %4.2f    Stbd: %4.2f\n", T_p, T_s);
	}
} // END OF thrust_saturation_check()
//.............................................................................................................END OF Functions...............................................................................................................

//..................................................................................................................Main Program..................................................................................................................
int main(int argc, char **argv)
{
	ros::init(argc, argv, "propulsion_system");  // names the program for visual purposes

	// NodeHandles
	ros::NodeHandle nh1, nh2, nh3, nh7;

	// Subscribers
	// from mission_control
	ros::Subscriber MC_ps_state_sub = nh1.subscribe("MC_ps_state", 1, MC_ps_state_update);
	// from path_planner
	ros::Subscriber PP_propulsion_system_topic_sub = nh2.subscribe("PP_propulsion_system_topic", 1, PP_propulsion_system_topic_update);  // path_planner directions update
	// from navigation_array
	ros::Subscriber NA_nav_ned_sub = nh3.subscribe("NA_nav_ned", 1, NA_nav_ned_update);  // Obtains the USV pose in local NED

	// Publishers
	// to mission_control
	PS_initialization_state_pub = nh7.advertise<std_msgs::Bool>("PS_initialization_state", 1);  // state of initialization
	// to system interface
	ros::Publisher right_thrust_cmd_pub = nh7.advertise<std_msgs::Float32>("/wamv/thrusters/right_thrust_cmd", 10);  // between -1.0 and 1.0, speed to right thruster
	ros::Publisher left_thrust_cmd_pub = nh7.advertise<std_msgs::Float32>("/wamv/thrusters/left_thrust_cmd", 10);  // value between -1.0 and 1.0, speed to left thruster
	ros::Publisher right_thrust_angle_pub = nh7.advertise<std_msgs::Float32>("/wamv/thrusters/right_thrust_angle", 10);  // value between -PI to PI, angle to right thruster
	ros::Publisher left_thrust_angle_pub = nh7.advertise<std_msgs::Float32>("/wamv/thrusters/left_thrust_angle", 10);  // value between -PI to PI, angle to left thruster
	// to the user
	PS_control_efforts_topic_pub = nh7.advertise<amore::control_efforts>("PS_control_efforts_topic", 1);  // control_efforts

	// Local variables
	std_msgs::Float32 left_thrust_cmd_msg, right_thrust_cmd_msg, left_thrust_angle_msg, right_thrust_angle_msg;
	
	// Initialize global variables
	PS_initialization_state_msg.data = false;
	current_time = ros::Time::now();  // sets current time to the time it is now
	last_time = current_time;  // sets last time to the time it is now
	
	ros::Rate loop_rate((int)1/dt);  // sets the frequency for which the program loops at 100 = 1/100 second  // was 3
	
	while(ros::ok())  // ros::ok() will be false when the user inputs Ctrl+C
	{
		PROPULSION_SYSTEM_inspector();  // check initialization status and update PS_initialization_state_msg

		if (PS_state == 1)
		{
			ros::spinOnce();  // update subscribers to get most up to date goal and USV poses
			update_gains_LL_controller();  // Update all gain parameters in launch file

			Integral_reset();  // Reset integral term once the errors become minimal 

			// determine error in x and y (position)
			e_x = x_goal - x_usv_NED;
			e_y = y_goal - y_usv_NED;
			e_xy = sqrt(pow(e_x,2.0)+pow(e_y,2.0));  // calculate magnitude of positional error

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
			if ( (loop_count >= loop_count_ON) && (loop_count < (loop_count_ON + 2)) )  // don't include differential term or integration term first 2 loops after being turned ON
			{
				// USE P TERM ONLY TO COMPUTE CONTROL EFFORT SINCE I AND D TERMS ARE NOT YET DEFINED THE LOOP AFTER CONTROLLER IS TURNED ON
				//ROS_INFO("PROPULSION_SYSTEM: P Control");
				if (LL_state == 1)  // 1 = PID HP Dual-azimuthing station keeping controller
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
				else if ((LL_state == 2) || (LL_state == 3))  // 2 = PID HP Differential wayfinding controller  // 3 = PID HP Ackermann wayfinding controller
				{
					T_x = Kp_x*e_xy;
				}
			}
			else if (loop_count >= (loop_count_ON + 2))
			{
				// USE PID CONTROLLER ONCE I AND D TERMS ARE DEFINED
				//ROS_INFO("PROPULSION_SYSTEM: PID Control");
				if (LL_state == 1)  // 1 = PID HP Dual-azimuthing station keeping controller
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
				else if ((LL_state == 2) || (LL_state == 3))  // 2 = PID HP Differential wayfinding controller  // 3 = PID HP Ackermann wayfinding controller
				{
					e_xy_total = e_xy_total + ((e_xy_prev + e_xy)/2.0)*dt;  // trapezoidal integration of errors for integral term
					T_x = Kp_x*e_xy+ Kd_x*((e_xy - e_xy_prev)/dt) + Ki_x*e_xy_total;
				}
			}  // END OF position control law 
			
			// make psi_goal the heading to get to the goal position until within 8.0 meters of goal position
			// or if differfential or Ackermann drive configuration
			if ((LL_state == 2) || (LL_state == 3) || (e_xy > 8.0))
			{
				psi_goal = atan2(e_y,e_x);  // [radians] atan2() returns between -PI and PI
			}

			e_psi = psi_goal - psi_usv_NED;  // determine error in psi (heading)
			
			// keep error value between -PI and PI
			while ((e_psi < -PI) || (e_psi >PI))
			{
				// ensure shortest turn is used
				if (e_psi < -PI)
				{
					e_psi = e_psi + 2.0*PI;
				}
				if (e_psi > PI)
				{
					e_psi = e_psi - 2.0*PI;
				}
			}
			
			// PI WRAP?
			// correct discontinuity in heading error
			if (e_psi < (-PI + 0.1*PI))
			{
				e_psi = e_psi + 2.0*PI;
			}
			if (e_psi > (PI - 0.1*PI))
			{
				e_psi = e_psi - 2.0*PI;
			}
			
			// Previous propulsion_system 
			/* // correct discontinuity in heading error
			if (e_psi < ((-2.0*PI) + (0.05*2.0*PI)))
			{
				e_psi = e_psi + 2.0*PI;
			}
			if (e_psi > ((2.0*PI) - (0.05*2.0*PI)))
			{
				e_psi = e_psi - 2.0*PI;
			}

			// ensure shortest turn is used
			if (e_psi < -PI)
			{
				e_psi = e_psi + 2.0*PI;
			}
			if (e_psi > PI)
			{
				e_psi = e_psi - 2.0*PI;
			} */

			/* // if within error, have selective gains
			if ((float)abs(e_psi) < 0.1)
			{
				Kp_psi = 0.0;
				Kd_psi = 0.0;
				Ki_psi = 0.0;
			} */

			// orientation control law
			if ( (loop_count >= loop_count_ON) && (loop_count < (loop_count_ON + 2)) )  // don't include differential term or integration term first time through
			{
				// USE P TERM ONLY TO COMPUTE CONTROL EFFORT SINCE I AND D TERMS ARE NOT YET DEFINED THE LOOP AFTER CONTROLLER IS TURNED ON
				//ROS_INFO("PROPULSION_SYSTEM: P Control");
				// effort in z-rotation
				M_z_P = Kp_psi*e_psi;  // P-term
				M_z_I = 0.0;  // I-term
				M_z_D = 0.0;  // D-term
				M_z = M_z_P;  // only P-term
			}
			else if (loop_count >= (loop_count_ON + 2))
			{
				// USE PID CONTROLLER ONCE I AND D TERMS ARE DEFINED
				//ROS_INFO("PROPULSION_SYSTEM: PID Control");
				// trapezoidal integration of error for integral term
				e_psi_total = e_psi_total + ((e_psi_prev + e_psi)/2.0)*dt;
				// effort in z-rotation
				M_z_P = Kp_psi*e_psi;  // P-term
				M_z_I = Ki_psi*e_psi_total;  // I-term
				M_z_D = Kd_psi*((e_psi - e_psi_prev)/dt);  // D-term
				M_z = M_z_P + M_z_I + M_z_D;  // PID terms
			}  // END OF orientation control law

			// ROS_INFO("PROPULSION_SYSTEM:---GOAL POSE---");  // UPDATE USER
			// ROS_INFO("PROPULSION_SYSTEM: x_goal: %.2f", x_goal);
			// ROS_INFO("PROPULSION_SYSTEM: y_goal: %.2f", y_goal);
			// ROS_INFO("PROPULSION_SYSTEM: psi_goal: %.2f", psi_goal);
			// ROS_INFO("PROPULSION_SYSTEM:---GOAL POSE---\n");

			// ROS_INFO("PROPULSION_SYSTEM:---USV POSE---");  // UPDATE USER
			// ROS_INFO("PROPULSION_SYSTEM: x_usv: %.2f", x_usv_NED);
			// ROS_INFO("PROPULSION_SYSTEM: y_usv: %.2f", y_usv_NED);
			// ROS_INFO("PROPULSION_SYSTEM: psi_usv: %.2f", psi_usv_NED);
			// ROS_INFO("PROPULSION_SYSTEM:---USV POSE---\n");

			ROS_INFO("PROPULSION_SYSTEM:--------ERRORS--------");  // UPDATE USER
			ROS_INFO("PROPULSION_SYSTEM:     e_x   :  %4.2f", e_x);  // x posn. error
			ROS_INFO("PROPULSION_SYSTEM:     e_y   :  %4.2f", e_y);  // y posn. error
			ROS_INFO("PROPULSION_SYSTEM:     e_xy  :  %4.2f", e_xy);  // magnitude of posn. error
			ROS_INFO("PROPULSION_SYSTEM:     e_psi :  %4.2f", e_psi);  // heading error
			ROS_INFO("PROPULSION_SYSTEM:--------ERRORS--------\n");

			// fill out control_efforts message and publish to "PS_control_efforts_topic"
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
			PS_control_efforts_topic_pub.publish(PS_control_efforts_topic_msg);
			
			// ROS_INFO("PROPULSION_SYSTEM:-----Control Efforts-----");
			// ROS_INFO("PROPULSION_SYSTEM: T_x: %.2f", T_x);
			// ROS_INFO("PROPULSION_SYSTEM: T_y: %.2f", T_y);
			// ROS_INFO("PROPULSION_SYSTEM: M_z: %.2f", M_z);
			// ROS_INFO("PROPULSION_SYSTEM:-----Control Efforts-----\n");

			// only control heading if off by more than 60 degrees
			if ((float)abs(e_psi) > 1.047)
			{
				T_x = 0.0;
				T_y = 0.0;
			}

			// EXPERIMENTING WITH THIS
			// if within a meter only control heading
			if (e_xy < 0.4) 
			{
				T_x = 0.0;
				T_y = 0.0;
			}

			// EXPERIMENTING WITH THIS
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
			}
			
			// ALLOCATION to go from control_efforts to thruster commands
			if (LL_state == 1)  // 1 = PID HP Dual-azimuthing station-keeping controller
			{
				// Convert to USV body-fixed frame from global frame
				T_x_bf = T_x*cos(psi_usv_NED) + T_y*sin(psi_usv_NED);
				T_y_bf = T_y*cos(psi_usv_NED) - T_x*sin(psi_usv_NED);

				T_x = T_x_bf;
				T_y = T_y_bf;

				// calculate the control allocation outputs
				// f = Transform_pseudoinverse * tau;
				F_xp = Transform_pseudoinverse[0][0]*T_x + Transform_pseudoinverse[0][1]*T_y + Transform_pseudoinverse[0][2]*M_z;
				F_yp = Transform_pseudoinverse[1][0]*T_x + Transform_pseudoinverse[1][1]*T_y + Transform_pseudoinverse[1][2]*M_z;
				F_xs = Transform_pseudoinverse[2][0]*T_x + Transform_pseudoinverse[2][1]*T_y + Transform_pseudoinverse[2][2]*M_z;
				F_ys = Transform_pseudoinverse[3][0]*T_x + Transform_pseudoinverse[3][1]*T_y + Transform_pseudoinverse[3][2]*M_z;

				T_p = sqrt(pow(F_xp,2.0)+pow(F_yp,2.0));  // calculate magnitude of port thrust
				T_s = sqrt(pow(F_xs,2.0)+pow(F_ys,2.0));  // calculate magnitude of starboard thrust
				A_p = -atan2(F_yp,F_xp);  // calculate angle of port thrust
				A_s = -atan2(F_ys,F_xs);  // calculate angle of starboard thrust

				// DEBUG INFORMATION ////////////////////////////////////////////////////////////
				// Print Proportional, Integral, and Derivative amounts of control effort
				// ROS_DEBUG("PROPULSION_SYSTEM:-----P I D Control Efforts-----");
				// ROS_DEBUG("PROPULSION_SYSTEM: T_x_P: %f", T_x_P);
				// ROS_DEBUG("PROPULSION_SYSTEM: T_x_I: %f", T_x_I);
				// ROS_DEBUG("PROPULSION_SYSTEM: T_x_D: %f\n", T_x_D);
				// ROS_DEBUG("PROPULSION_SYSTEM: T_y_P: %f", T_y_P);
				// ROS_DEBUG("PROPULSION_SYSTEM: T_y_I: %f", T_y_I);
				// ROS_DEBUG("PROPULSION_SYSTEM: T_y_D: %f\n", T_y_D);
				// ROS_DEBUG("PROPULSION_SYSTEM: M_z_P: %f", M_z_P);
				// ROS_DEBUG("PROPULSION_SYSTEM: M_z_I: %f", M_z_I);
				// ROS_DEBUG("PROPULSION_SYSTEM: M_z_D: %f\n", M_z_D);

				// Print thrust control efforts in x and y directions in both the local frame (working frame) and the body-fixed frame (USV frame)
				// ROS_DEBUG("PROPULSION_SYSTEM: BEFORE SWAP TO BODY-FIXED FRAME");
				// ROS_DEBUG("PROPULSION_SYSTEM: T_x_G: %f", T_x);
				// ROS_DEBUG("PROPULSION_SYSTEM: T_y_G: %f", T_y);
				// ROS_DEBUG("PROPULSION_SYSTEM: AFTER SWAP");
				// ROS_DEBUG("PROPULSION_SYSTEM: T_x_bf: %f", T_x_bf);
				// ROS_DEBUG("PROPULSION_SYSTEM: T_y_bf: %f\n", T_y_bf);

				// Print force control effort values in x and y directions on each thruster
				// ROS_DEBUG("PROPULSION_SYSTEM: F_xp: %f", F_xp);
				// ROS_DEBUG("PROPULSION_SYSTEM: F_yp: %f", F_yp);
				// ROS_DEBUG("PROPULSION_SYSTEM: F_xs: %f", F_xs);
				// ROS_DEBUG("PROPULSION_SYSTEM: F_ys: %f\n", F_ys);
			}
			else if (LL_state == 2)  // 2 = PID HP Differential wayfinding controller
			{
				// Calculate torque to thrusters
				T_p = T_x/2.0 + M_z/B;
				T_s = T_x/2.0 - M_z/B;

				// Set thruster angles to zero since it is differential drive
				A_p = 0.0;
				A_s = 0.0;

				/* // display contributions of thrust outputs
				ROS_DEBUG("PROPULSION_SYSTEM: LT_T is: %f", T_x/2.0);  // displays left thrust value
				ROS_DEBUG("PROPULSION_SYSTEM: LT_M is: %f", M_z/B);  // displays right thrust value
				ROS_DEBUG("PROPULSION_SYSTEM: RT_T is: %f", T_x/2.0);  // displays left thrust value
				ROS_DEBUG("PROPULSION_SYSTEM: RT_M is: %f\n", -M_z/B);  // displays right thrust value

				ROS_DEBUG("PROPULSION_SYSTEM: left_thrust_cmd_msg is: %f", T_p);  // displays left thrust value
				ROS_DEBUG("PROPULSION_SYSTEM: right_thrust_cmd_msg is: %f\n", T_s);  // displays right thrust value */
			}
			else if (LL_state == 3)  // 3 = PID HP Ackermann wayfinding controller
			{
				// Use Ackermann drive configuration to set angles to thrusters 
				A_p = atan(lx/(M_z+ly));
				A_s = atan(lx/(M_z-ly));

				// Calculate torque to thrusters
				T_p = T_x/(cos(A_p)+cos(A_s));
				T_s = T_p;
			}

			/* ROS_DEBUG("PROPULSION_SYSTEM:-----BEFORE angle_correction_check()-----");
			ROS_DEBUG("PROPULSION_SYSTEM: Port Thrust: %.2f", T_p);
			ROS_DEBUG("PROPULSION_SYSTEM: Stbd Thrust: %.2f", T_s);
			ROS_DEBUG("PROPULSION_SYSTEM: Port Angle: %.2f", A_p);
			ROS_DEBUG("PROPULSION_SYSTEM: Stbd Angle: %.2f", A_s); */

			angle_correction_check();

			/* ROS_DEBUG("PROPULSION_SYSTEM:-----AFTER angle_correction_check()-----");
			ROS_DEBUG("PROPULSION_SYSTEM: Port Thrust: %.2f", T_p);
			ROS_DEBUG("PROPULSION_SYSTEM: Stbd Thrust: %.2f", T_s);
			ROS_DEBUG("PROPULSION_SYSTEM: Port Angle: %.2f", A_p);
			ROS_DEBUG("PROPULSION_SYSTEM: Stbd Angle: %.2f\n", A_s); */

			thrust_saturation_check();

			// set thruster message commands
			left_thrust_angle_msg.data = A_p;
			left_thrust_cmd_msg.data = T_p;
			right_thrust_angle_msg.data = A_s;
			right_thrust_cmd_msg.data = T_s;
			// publish thruster message commands
			left_thrust_angle_pub.publish(left_thrust_angle_msg);
			left_thrust_cmd_pub.publish(left_thrust_cmd_msg);
			right_thrust_angle_pub.publish(right_thrust_angle_msg);
			right_thrust_cmd_pub.publish(right_thrust_cmd_msg);

			// update previous errors
			e_x_prev = e_x;
			e_y_prev = e_y;
			e_xy_prev = e_xy;
			e_psi_prev = e_psi;
		}  // END OF if (PS_state == 1)
		else if (PS_state == 0)  // Controller is told to be On standby by command from mission_control
		{
			// set thruster speeds to zero
			left_thrust_cmd_msg.data = 0.0;
			right_thrust_cmd_msg.data = 0.0;
			// publish thruster speeds
			left_thrust_cmd_pub.publish(left_thrust_cmd_msg);
			right_thrust_cmd_pub.publish(right_thrust_cmd_msg);
		}  // END OF else if (PS_state == 0)

		ros::spinOnce();  // update subscribers
		loop_rate.sleep();  // sleep to accomplish set loop_rate
		last_time = current_time;  // update last_time
	}  // END OF while(ros::ok())

	// set thruster speeds to zero
	left_thrust_cmd_msg.data = 0.0;
	right_thrust_cmd_msg.data = 0.0;
	// publish thruster speeds
	left_thrust_cmd_pub.publish(left_thrust_cmd_msg);
	right_thrust_cmd_pub.publish(right_thrust_cmd_msg);

	return 0;
}  // END OF main()
//.........................................................................................................END OF Main Program...........................................................................................................