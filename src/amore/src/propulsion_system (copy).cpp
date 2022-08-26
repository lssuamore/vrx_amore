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
//				Inputs: ["current_goal_pose" - amore/usv_pose_msg - goal pose (x,y,psi) to reach in local NED frame]
//				Outputs: "thruster_int_right", "thuster_int_left", "angle_int_right", "angle_int_left" - dual-azimuthing Minn Kota thruster commands


//................................................Included Libraries and Message Types..........................................
#include "ros/ros.h"
#include "ros/console.h"
#include "time.h"
#include <sstream>
#include <iostream>
#include "math.h"
#include "stdio.h"
#include "nav_msgs/Odometry.h"  // message type used for receiving NED USV state from navigation_array
#include "amore/state_msg.h"  // message type used to recieve state of operation from mission_control
#include "std_msgs/Bool.h"
#include "amore/usv_pose_msg.h"  // message type that holds usv position as a geometry_msgs/Point and heading in radians as a Float64
#include "std_msgs/Float32.h"  // message type of thruster commands, and type for control efforts
#include "amore/control_efforts.h"  // message that holds tx,ty, and m_z
//...........................................End of Included Libraries and Message Types....................................


//.................................................................Constants....................................................................
#define PI 3.14159265
//............................................................End of Constants.............................................................


//..............................................................Global Variables............................................................
int loop_count = 0;  // loop counter
int loop_count_ON = 0;  // loop count holder for when the the controller is turned on, used to ensure differential and integral terms are not started until they are calculated
bool PS_state_ON = false;  // used to set and reset the above count holder like a oneshot
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

int LL_state = 1;	//	1 = PID HP Dual-azimuthing station keeping controller	//	2 = PID HP Differential wayfinding controller
float dt = 0.25;				// [s] used for differential term

float x_goal, y_goal, psi_goal;			// [m, m, radians] desired position and heading
float x_usv_NED, y_usv_NED, psi_NED; 		// vehicle position and heading (pose) in NED
float e_x, e_y, e_xy, e_psi;			// current errors between goal pose and usv pose

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

float T_x;					// thrust to set in x-direction in earth-fixed frame
float T_y;					// thrust to set in y-direction in earth-fixed frame
float T_x_bf;					// thrust in x-direction in body-fixed frame
float T_y_bf;					// thrust in y-direction in body-fixed frame
float M_z;					// desired moment around the z-axis

// matrix to hold outputs of controlller
//float tau[3][1];

float T_p;					// used to set the port (left) thruster output
float T_s;					// used to set the starboard (right) thruster output
float A_p;					// used to set the port (left) thruster angle
float A_s;					// used to set the starboard (right) thruster angle

// Used for correction of saturated thrust values 
float T_p_C;					// Corrected port (left) thrust output
float T_s_C;					// Corrected starboard (right) thrust output

float Kp_x, Kp_y, Kp_psi;			// Proportional gains
float Kd_x, Kd_y, Kd_psi;			// Differential gains
float Ki_x, Ki_y, Ki_psi;			// Integration gains

// USV system id
float B = 2.0;							// [m] FAU FOUND IT TO BE 2.0
float lx = 2.3;							// [m] bf x-offset of thrusters
float ly = 1.0;							// [m] bf y-offset of thrusters
/* 
float L = 4.6;					// [m] length of USV, 16 ft or 4.88 m

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
float F_xp;																			// force in the x-direction on the port side
float F_yp;																			// force in the y-direction on the port side
float F_xs;																			// force in the x-direction on the starboard side
float F_ys;																			// force in the y-direction on the starboard side

std_msgs::Bool ps_initialization_status;	// "ps_initialization_state" message
ros::Publisher ps_initialization_state_pub;	// "ps_initialization_state" publisher

amore::control_efforts control_efforts_msg;	// "control_efforts_topic" message
ros::Publisher control_efforts_pub;	// "ps_initialization_state" publisher

ros::Time current_time, last_time;		// creates time variables
//..............................................................End of Global Variables............................................................


//..................................................................Functions.................................................................
// THIS FUNCTION: Updates global current_time, loop_count, and publishes initialization status to "ps_initialization_state"
// ACCEPTS: (VOID)
// RETURNS: (VOID)
// =============================================================================
void PROPULSION_SYSTEM_inspector()
{
	current_time = ros::Time::now();   	// sets current_time to the time it is now
	loop_count += 1;			// increment loop counter
	if (loop_count > 3)
	{
		ps_initialization_status.data = true;
		//ROS_INFO("propulsion_system_initialized -- PS");
	}
	else
	{
		ps_initialization_status.data = false;
		ROS_INFO("!propulsion_system_initialized -- PS");
	}
	ps_initialization_state_pub.publish(ps_initialization_status);	// publish the initialization status of the propulsion_system to "ps_initialization_state"
} // END OF PROPULSION_SYSTEM_inspector()

/////////////////////////////////////////////////////////////////		STATE UPDATERS		///////////////////////////////////////////////////////////////////
// THIS FUNCTION: Updates the state of "navigation_array" given by "mission_control"
// ACCEPTS: navigation_array state_msg from "na_state"
// RETURNS: (VOID)		Updates global variables
// =============================================================================
void na_state_update(const amore::state_msg::ConstPtr& msg)
{
	if (ps_initialization_status.data)
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
	if (ps_initialization_status.data)
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
	if (ps_initialization_status.data)
	{
		PS_state = msg->state.data;
	}
	if ((!PS_state_ON) && (PS_state == 1))
	{
		PS_state_ON = true;
		loop_count_ON = loop_count;
	}
	else if (PS_state == 0)
	{
		PS_state_ON = false;
	}
} // END OF ps_state_update()

// THIS FUNCTION: Updates the state of "perception_array" given by "mission_control"
// ACCEPTS: perception_array state_msg from "pa_state"
// RETURNS: (VOID) Updates global variable
// =============================================================================
void pa_state_update(const amore::state_msg::ConstPtr& msg) 
{
	if (ps_initialization_status.data)
	{
		PA_state = msg->state.data;
	}
} // END OF pa_state_update()
//////////////////////////////////////////////////////////////		STATE UPDATERS END		///////////////////////////////////////////////////////////////////

// THIS FUNCTION: Updates the current NED USV pose converted through the navigation_array
// ACCEPTS: Current NED USV pose and velocities from "nav_ned"
// RETURNS: (VOID)
// =============================================================================
void pose_update(const nav_msgs::Odometry::ConstPtr& odom) 
{
	// Update NED USV pose 
	x_usv_NED = odom->pose.pose.position.x;
	y_usv_NED = odom->pose.pose.position.y;
	psi_NED = odom->pose.pose.orientation.z;
} // END OF pose_update()

// THIS FUNCTION: Updates the goal pose for the propulsion_system given by path_planner
// ACCEPTS: usv_pose_msg from "current_goal_pose"
// RETURNS: (VOID)
// =============================================================================
void goal_pose_update(const amore::usv_pose_msg::ConstPtr& goal) 
{
	// update NED goal position and orientation
	x_goal = goal->position.x;		
	y_goal = goal->position.y;
	psi_goal = goal->psi.data;
} // END OF goal_pose_update()

// THIS FUNCTION: Displays the updated gains
// ACCEPTS: (VOID) 
// RETURNS: (VOID) 
// =====================================================
void display_gains()
{
	// PROPORTIONAL GAINS
	ROS_INFO("Kp_xy is: %.2f --PS", Kp_x);
	ROS_INFO("Kp_psi is: %.2f --PS\n", Kp_psi);
	// DERIVATIVE GAINS
	ROS_INFO("Kd_xy is: %.2f --PS", Kd_x);
	ROS_INFO("Kd_psi is: %.2f --PS\n", Kd_psi);
	// INTEGRAL GAINS
	ROS_INFO("Ki_xy is: %.2f --PS", Ki_x);
	ROS_INFO("Ki_psi is: %.2f --PS\n", Ki_psi);
} // END OF display_gains()

// THIS FUNCTION: Updates the gains
// ACCEPTS: (VOID) 
// RETURNS: (VOID) 
// =====================================================
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
		display_gains();  // DO NOT COMMENT  OUT THIS LINE IF YOU WANT TO PRINT GAINS TO USER 
	}
} // END OF update_gains_LL_controller()

// THIS FUNCTION: Resets the integral term once the errors become minimal 
//	to avoid overshooting the goal if a huge effort's built up
// ACCEPTS: (VOID) 
// RETURNS: (VOID) 
// =====================================================
void Integral_reset()
{
	if ((float)abs(e_x) < 0.1)	// NOTE: this value may need adjusting
	{
	  e_x_total = 0.0;
	}
	if ((float)abs(e_y) < 0.1)	// NOTE: this value may need adjusting
	{
	  e_y_total = 0.0;
	}
	if ((float)abs(e_xy) < 1.0)
	{
	  e_xy_total = 0.0;
	}
	if ((float)abs(e_psi) < 0.5)	// NOTE: this value may need adjusting
	{
	  e_psi_total = 0.0;
	}
} // END OF Integral_reset()

// THIS FUNCTION: Checks that angles are between -PI/2 and PI/2 and corrects accordingly
// ACCEPTS: (VOID)
// RETURNS: (VOID)
// =====================================================
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
// =====================================================
void thrust_saturation_check()
{
	// check for saturation
	if (((float)abs(T_p) > 1.0) || ((float)abs(T_s) > 1.0))
	{
		// correct for saturation by normalizing thrust data
		if ((float)abs(T_p) > (float)abs(T_s))
		{
		  //ROS_DEBUG("LT IS ISSUE!");
		  T_p_C= T_p / (float)abs(T_p);
		  T_s_C = T_s / (float)abs(T_p);
		}
		else if ((float)abs(T_p) < (float)abs(T_s))
		{
		  //ROS_DEBUG("RT IS ISSUE!");
		  T_p_C = T_p / (float)abs(T_s);
		  T_s_C = T_s / (float)abs(T_s);
		}
		else 
		{
		  //ROS_DEBUG("Equal");
		  //ROS_DEBUG("Divide by : %f\n", (float)abs(T_p));      // displays right thrust value
		  T_p_C = T_p / (float)abs(T_p);
		  T_s_C = T_s / (float)abs(T_p);
		}
		T_p = T_p_C;
		T_s = T_s_C;
		ROS_DEBUG("Port Thrust corrected: %.2f --PS", T_p);
		ROS_DEBUG("Stbd Thrust corrected: %.2f --PS\n", T_s);
	}
} // END OF thrust_saturation_check()
//............................................................End of Functions............................................................


int main(int argc, char **argv)
{
	//names the program for visual purposes
	ros::init(argc, argv, "propulsion_system");
	ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

	// NodeHandles
	ros::NodeHandle nh1, nh2, nh3, nh4, nh5, nh6, nh7;

	// Subscribers
	ros::Subscriber na_state_sub = nh1.subscribe("na_state", 1, na_state_update);
	ros::Subscriber pp_state_sub = nh2.subscribe("pp_state", 1, pp_state_update);
	ros::Subscriber ps_state_sub = nh3.subscribe("ps_state", 1, ps_state_update);
	ros::Subscriber pa_state_sub = nh4.subscribe("pa_state", 1, pa_state_update);
	ros::Subscriber nav_NED_sub = nh5.subscribe("nav_ned", 1, pose_update);  // Obtains the USV pose in global NED from mission_control
	ros::Subscriber current_goal_pose_sub = nh6.subscribe("current_goal_pose", 1, goal_pose_update);  // goal pose given by path planners

	// Publishers
	ps_initialization_state_pub = nh7.advertise<std_msgs::Bool>("ps_initialization_state", 1);  // state of initialization
	control_efforts_pub = nh7.advertise<amore::control_efforts>("control_efforts_topic", 1);  // control_efforts
	ros::Publisher stbd_T_pub = nh7.advertise<std_msgs::Float32>("/wamv/thrusters/right_thrust_cmd", 10);  // between -1.0 and 1.0, speed to right thruster
	ros::Publisher port_T_pub = nh7.advertise<std_msgs::Float32>("/wamv/thrusters/left_thrust_cmd", 10);  // value between -1.0 and 1.0, speed to left thruster
	ros::Publisher stbd_A_pub = nh7.advertise<std_msgs::Float32>("/wamv/thrusters/right_thrust_angle", 10);  // value between -PI to PI, angle to right thruster
	ros::Publisher port_A_pub = nh7.advertise<std_msgs::Float32>("/wamv/thrusters/left_thrust_angle", 10);  // value between -PI to PI, angle to left thruster

	// Local variables
	std_msgs::Float32 LT, RT, LA, RA;  // LT is left thrust, RT is right thrust, LA is left thruster angle, RA is right thruster angle

	// Initialize global variables
	ps_initialization_status.data = false;
	current_time = ros::Time::now();  // sets current time to the time it is now
	last_time = current_time;  // sets last time to the time it is now
	
	ros::Rate loop_rate(3);  // sets the frequency for which the program sleeps at 100 = 1/100 second  // was 3
	
	//rosk::ok() will stop when the user inputs Ctrl+C
	while(ros::ok())
	{
		PROPULSION_SYSTEM_inspector();  // check initialization status and update ps_initialization_status

		if (PS_state == 1)
		{
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
			if  (loop_count > (loop_count_ON + 2))  // don't include differential term or integration term first 2 loops after being turned ON
			{
				ROS_INFO("P Control");
				if (LL_state == 1)  // 1 = PID HP Dual-azimuthing station keeping controller
				{
					T_x = Kp_x*e_x;
					T_y = Kp_y*e_y;
				}
				else if ((LL_state == 2) || (LL_state == 3))  // 2 = PID HP Differential wayfinding controller  // 3 = PID HP Ackermann wayfinding controller
				{
					T_x = Kp_x*e_xy;
				}
			}
			else
			{
				ROS_INFO("PID Control");
				if (LL_state == 1)  // 1 = PID HP Dual-azimuthing station keeping controller
				{
					// trapezoidal integration of errors for integral term
					e_x_total = e_x_total + ((e_x_prev + e_x)/2.0)*dt;
					e_y_total = e_y_total + ((e_y_prev + e_y)/2.0)*dt;
					T_x = Kp_x*e_x + Kd_x*((e_x - e_x_prev)/dt) + Ki_x*e_x_total;
					T_y = Kp_y*e_y + Kd_y*((e_y - e_y_prev)/dt) + Ki_y*e_y_total;
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

			e_psi = psi_goal - psi_NED;  // determine error in psi (heading)
			
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
				ROS_INFO("P Control");
				M_z = Kp_psi*e_psi;
			}
			else if (loop_count >= (loop_count_ON + 2))
			{
				ROS_INFO("PID Control");
				e_psi_total = e_psi_total + ((e_psi_prev + e_psi)/2.0)*dt;  // trapezoidal integration of error for integral term
				M_z = Kp_psi*e_psi + Kd_psi*((e_psi - e_psi_prev)/dt) + Ki_psi*e_psi_total;
			}  // END OF orientation control law

			// UPDATES STATUSES TO USER ///////////////////////////////////////////////
			ROS_DEBUG("x_goal: %.2f --PS", x_goal);
			ROS_DEBUG("y_goal: %.2f --PS", y_goal);
			ROS_DEBUG("des_psi: %.2f --PS\n", psi_goal);

			ROS_DEBUG("x_usv: %.2f --PS", x_usv_NED);
			ROS_DEBUG("y_usv: %.2f --PS", y_usv_NED);
			ROS_DEBUG("psi_NED: %.2f --PS\n", psi_NED);

			ROS_DEBUG("e_x: %.2f --PS", e_x);  // x posn. error
			ROS_DEBUG("e_y: %.2f --PS", e_y);  // y posn. error
			ROS_DEBUG("e_xy: %.2f --PS", e_xy);  // magnitude of posn. error
			ROS_DEBUG("e_psi: %.2f --PS\n", e_psi);  // heading error	  
			
			// fill out control_efforts message and publish to "control_efforts_topic"
			control_efforts_msg.t_x.data = T_x;
			control_efforts_msg.t_y.data = T_y;
			control_efforts_msg.m_z.data = M_z;
			control_efforts_pub.publish(control_efforts_msg);
			
			/* ROS_DEBUG("Control Efforts --PS");
			ROS_DEBUG("T_x_G: %.3f --PS", T_x);
			ROS_DEBUG("T_y_G: %.3f --PS", T_y);
			ROS_DEBUG("M_z: %.3f --PS\n", M_z); */

			/* // if within a meter only control heading
			if (e_xy < 1.0) 
			{
				T_x = 0.0;
				T_y = 0.0;
			} */

			// only control heading if heading is off by more than 45 degree
			if ((float)abs(e_psi) > 0.785)
			{
				T_x = 0.0;
				T_y = 0.0;
			}

			/* // if errors are small enough, do not try to correct for them
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
			if (LL_state == 1)				//	1 = PID HP Dual-azimuthing station keeping controller
			{
				// Convert to USV body-fixed frame from global frame
				T_x_bf = T_x*cos(psi_NED) + T_y*sin(psi_NED);
				T_y_bf = T_y*cos(psi_NED) - T_x*sin(psi_NED);
				/* T_x = (float)abs(T_x_bf);
				T_y = (float)abs(T_y_bf); */

				T_x = T_x_bf;
				T_y = T_y_bf;

				// calculate the control allocation outputs
				// f = Transform_pseudoinverse * tau;
				F_xp = Transform_pseudoinverse[0][0]*T_x + Transform_pseudoinverse[0][1]*T_y + Transform_pseudoinverse[0][2]*M_z;
				F_yp = Transform_pseudoinverse[1][0]*T_x + Transform_pseudoinverse[1][1]*T_y + Transform_pseudoinverse[1][2]*M_z;
				F_xs = Transform_pseudoinverse[2][0]*T_x + Transform_pseudoinverse[2][1]*T_y + Transform_pseudoinverse[2][2]*M_z;
				F_ys = Transform_pseudoinverse[3][0]*T_x + Transform_pseudoinverse[3][1]*T_y + Transform_pseudoinverse[3][2]*M_z;

				T_p = sqrt(pow(F_xp,2.0)+pow(F_yp,2.0));	// calculate magnitude of port thrust
				T_s = sqrt(pow(F_xs,2.0)+pow(F_ys,2.0));	// calculate magnitude of starboard thrust
				A_p = -atan2(F_yp,F_xp);									// calculate angle of port thrust
				A_s = -atan2(F_ys,F_xs);									// calculate angle of starboard thrust

				// DEBUG INFORMATION ////////////////////////////////////////////////////////////
				/* // Proportional, Derivative, and Integral amounts of control effort
				ROS_DEBUG("Control Effort Information");
				ROS_DEBUG("T_x_P: %f", Kp_x*e_x);
				ROS_DEBUG("T_x_D: %f", Kd_x*((e_x - e_x_prev)/dt));
				ROS_DEBUG("T_x_I: %f", Ki_x*e_x_total);
				ROS_DEBUG("T_y_P: %f", Kp_y*e_y);
				ROS_DEBUG("T_y_D: %f", Kd_y*((e_y - e_y_prev)/dt));
				ROS_DEBUG("T_y_I: %f", Ki_y*e_y_total);
				ROS_DEBUG("M_z_P: %f", Kp_psi*e_psi);
				ROS_DEBUG("M_z_D: %f", Kd_psi*((e_psi - e_psi_prev)/dt));
				ROS_DEBUG("M_z_I: %f\n", Ki_psi*e_psi_total);

				ROS_DEBUG("BEFORE SWAP TO BODY-FIXED FRAME");
				ROS_DEBUG("T_x_G: %f", T_x);
				ROS_DEBUG("T_y_G: %f", T_y);
				ROS_DEBUG("M_z: %f", M_z);
				ROS_DEBUG("AFTER SWAP");
				ROS_DEBUG("T_x_bf: %f", T_x_bf);
				ROS_DEBUG("T_y_bf: %f\n", T_y_bf);

				// Print f values
				ROS_DEBUG("F_xp: %f", F_xp);
				ROS_DEBUG("F_yp: %f", F_yp);
				ROS_DEBUG("F_xs: %f", F_xs);
				ROS_DEBUG("F_ys: %f\n", F_ys); */
			}
			else if (LL_state == 2)  //	2 = PID HP Differential wayfinding controller
			{
				// Calculate torque to thrusters
				T_p = T_x/2.0 + M_z/B;
				T_s = T_x/2.0 - M_z/B;

				// Set thruster angles to zero since it is differential drive
				A_p = 0.0;
				A_s = 0.0;

				/* // display contributions of output 
				ROS_DEBUG("LT_T is: %f\n", T_x/2.0);	// displays left thrust value
				ROS_DEBUG("LT_M is: %f\n", M_z/B);	// displays right thrust value
				ROS_DEBUG("RT_T is: %f\n", T_x/2.0);	// displays left thrust value
				ROS_DEBUG("RT_M is: %f\n", -M_z/B);	// displays right thrust value

				ROS_DEBUG("LT is: %f\n", T_p);		// displays left thrust value
				ROS_DEBUG("RT is: %f\n", T_s);		// displays right thrust value */
			}
			else if (LL_state == 3)		// 3 = PID HP Ackermann wayfinding controller
			{
				// Use Ackermann drive configuration to set angles to thrusters 
				A_p = atan(lx/(M_z+ly));
				A_s = atan(lx/(M_z-ly));
				
				// Calculate torque to thrusters
				T_p = T_x/(cos(A_p)+cos(A_s));
				T_s = T_p;
			}
			
			/* ROS_DEBUG("Before--------------");
			ROS_DEBUG("Port Thrust: %.2f", T_p);
			ROS_DEBUG("Stbd Thrust: %.2f", T_s);
			ROS_DEBUG("Port Angle: %.2f", A_p);
			ROS_DEBUG("Stbd Angle: %.2f", A_s); */
			
			angle_correction_check();

			/* ROS_DEBUG("After---------------");
			ROS_DEBUG("Port Thrust: %.2f", T_p);
			ROS_DEBUG("Stbd Thrust: %.2f", T_s);
			ROS_DEBUG("Port Angle: %.2f", A_p);
			ROS_DEBUG("Stbd Angle: %.2f\n", A_s); */

			thrust_saturation_check();

			// DEBUG INFORMATION ////////////////////////////////////////////////////////////
			/* // Proportional, Derivative, and Integral amounts of control effort
			ROS_DEBUG("Control Effort Information");
			ROS_DEBUG("T_x_P: %f", Kp_x*e_x);
			ROS_DEBUG("T_x_D: %f", Kd_x*((e_x - e_x_prev)/dt));
			ROS_DEBUG("T_x_I: %f", Ki_x*e_x_total);
			ROS_DEBUG("T_y_P: %f", Kp_y*e_y);
			ROS_DEBUG("T_y_D: %f", Kd_y*((e_y - e_y_prev)/dt));
			ROS_DEBUG("T_y_I: %f", Ki_y*e_y_total);
			ROS_DEBUG("M_z_P: %f", Kp_psi*e_psi);
			ROS_DEBUG("M_z_D: %f", Kd_psi*((e_psi - e_psi_prev)/dt));
			ROS_DEBUG("M_z_I: %f\n", Ki_psi*e_psi_total); */

			/* ROS_DEBUG("BEFORE SWAP TO BODY-FIXED FRAME");
			ROS_DEBUG("T_x_G: %f", T_x);
			ROS_DEBUG("T_y_G: %f", T_y);
			ROS_DEBUG("M_z: %f", M_z);
			ROS_DEBUG("AFTER SWAP");
			ROS_DEBUG("T_x_bf: %f", T_x_bf);
			ROS_DEBUG("T_y_bf: %f\n", T_y_bf); */

			/* // Print f values
			ROS_DEBUG("F_xp: %f", F_xp);
			ROS_DEBUG("F_yp: %f", F_yp);
			ROS_DEBUG("F_xs: %f", F_xs);
			ROS_DEBUG("F_ys: %f\n", F_ys); */

			// only print to thrusters if far enough into loop to have correct calculations
			// for some reason the first 4 times through loop the current pose variables do not update
			// give time to let path planners and pose converters stabilize
			if ((loop_count>10) && (PS_state == 1)) // was at 30 loop_count
			{ 
				LA.data = A_p;
				LT.data = T_p;
				RA.data = A_s;
				RT.data = T_s;
				port_A_pub.publish(LA);
				port_T_pub.publish(LT);
				stbd_A_pub.publish(RA);
				stbd_T_pub.publish(RT);
			}

			// update previous errors
			e_x_prev = e_x;
			e_y_prev = e_y;
			e_xy_prev = e_xy;
			e_psi_prev = e_psi;
		}  // END OF if (PS_state == 1)
		else // (PS_state == 0), Controller is turned off by command from high level
		{
			LA.data = 0.0;
			LT.data = 0.0;
			RA.data = 0.0;
			RT.data = 0.0;
			port_A_pub.publish(LA);
			port_T_pub.publish(LT);
			stbd_A_pub.publish(RA);
			stbd_T_pub.publish(RT);
		}  // END OF if else (PS_state)

		ros::spinOnce();										// update subscribers
		loop_rate.sleep();									// sleep for set loop_rate
		last_time = current_time;						// update last_time
	}
	LT.data = 0.0;
	port_T_pub.publish(LT);
	RT.data = 0.0;	
	stbd_T_pub.publish(RT);

	ros::spinOnce();
	loop_rate.sleep();

	return 0;
}
