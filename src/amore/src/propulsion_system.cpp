//  Filename:						        Task1_SK_Controller.cpp
//  Creation Date:						1/31/2022
//  Last Revision Date:                1/31/2022
//  Author(s) [email]:					Brad Hacker [bhacker@lssu.edu]
//                                                  
//  Revisor(s) [Revision Date]:    
//  Organization/Institution:			Lake Superior State University
// 
// ...........................Task1_SK_Controller.cpp.......................
//  This code takes in a position to reach in the global frame as nav_odom msgs. 
//  It gets this position by subscribing to the "mpp_goal" node created by the Task1_SK_Planner.
//  This controller then uses PID control theory to control the heading and position of the USV.
//  This controller uses a dual-azimuthing drive configuration, which makes it over actuated. 
//   Control allocation is used to solve for the thrusts and azimuthing angles of each thruster.
//
//  Inputs and Outputs of the Task1_SK_Controller.cpp file
//				Inputs: Position (x,y) to reach in global NED frame
//				Outputs: Thruster outputs and angles for left and right thrusters


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
#include "std_msgs/Float32.h"												// thruster commands
//...........................................End of Included Libraries and Message Types....................................


//.................................................................Constants....................................................................
#define PI 3.14159265
//............................................................End of Constants.............................................................


//..............................................................Global Variables............................................................
int loop_count = 0;                                    			// loop counter, first 10 loops used to intitialize subscribers
//float duration = 1000;										// amount of time to finish the path 
float dt = 0.25;														// [s] used for differential term

float x_usv_NED, y_usv_NED, psi_NED; 		// vehicle position and heading (pose) in NED

int PS_state = 0;      											// 0 = On Standby, 1 = LL controller ON

float x_goal, y_goal, psi_goal;							// [m, m, radians] desired position and heading

float e_x, e_y, e_xy, e_psi;								// current errors between goal pose and usv pose

// initialize accumulated total errors for integral term
float e_x_total = 0;
float e_y_total = 0;
float e_psi_total = 0;
// initialize previous errors for calculating differential term
float e_x_prev = 0;
float e_y_prev = 0;
float e_xy_prev = 0;
float e_psi_prev = 0;

float T_x;																// thrust to set in x-direction in earth-fixed frame
float T_y;																// thrust to set in y-direction in earth-fixed frame
float T_x_bf;														// thrust in x-direction in body-fixed frame
float T_y_bf;														// thrust in y-direction in body-fixed frame
float M_z;															// desired moment around the z-axis

// matrix to hold outputs of controlller
//float tau[3][1];

float T_p;																// used to set the port (left) thruster output
float T_s;																// used to set the starboard (right) thruster output
float A_p;															// used to set the port (left) thruster angle
float A_s;															// used to set the starboard (right) thruster angle

// Used for correction of saturated thrust values 
float T_p_C;														// Corrected port (left) thrust output
float T_s_C;														// Corrected starboard (right) thrust output

float Kp_x, Kp_y, Kp_psi;									// Proportional gains
float Kd_x, Kd_y, Kd_psi;									// Differential gains
float Ki_x, Ki_y, Ki_psi;										// Integration gains

/* 
float B = 2.0;														// [m] beam of USV, 8 ft or 2.44 m
float L = 4.6;														// [m] length of USV, 16 ft or 4.88 m

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

std_msgs::Bool ps_initialization_status;							// "ps_initialization_state" message
ros::Publisher ps_initialization_state_pub;						// "ps_initialization_state" publisher

ros::Time current_time, last_time;									// creates time variables
//..............................................................End of Global Variables............................................................


//..................................................................Functions.................................................................
// THIS FUNCTION: Updates and publishes initialization status to "ps_initialization_state"
// ACCEPTS: (VOID)
// RETURNS: (VOID)
// =============================================================================
void PROPULSION_SYSTEM_inspector()
{
	if (loop_count > 10)
	{
		ps_initialization_status.data = true;
		//ROS_INFO("propulsion_system_initialized -- PS");
	}
	else
	{
		ps_initialization_status.data = false;
		ROS_INFO("!propulsion_system_initialized -- PS");
	}
	
	ps_initialization_state_pub.publish(ps_initialization_status);						// publish the initialization status of the propulsion_system to "ps_initialization_state"
} // END OF PROPULSION_SYSTEM_inspector()

// THIS FUNCTION: Updates the state of propulsion_system given by mission_control
// ACCEPTS: state_msg from "ps_state"
// RETURNS: (VOID)
// =============================================================================
void state_update(const amore::state_msg::ConstPtr& msg) 
{
	// do not start anything until subscribers to sensor data are initialized
	if (ps_initialization_status.data)
	{
		PS_state = msg->state.data;
	}
} // END OF state_update()

// THIS FUNCTION: Updates the current NED USV pose converted through the navigation_array
// ACCEPTS: Current NED USV pose and velocities from "nav_ned"
// RETURNS: (VOID)
// =============================================================================
void pose_update(const nav_msgs::Odometry::ConstPtr& odom) 
{
	if (PS_state == 1) // if the propulsion_system is ON
	{
		// Update NED USV pose 
		x_usv_NED = odom->pose.pose.position.x;
		y_usv_NED = odom->pose.pose.position.y;
		psi_NED = odom->pose.pose.orientation.z;
	} // if (PS_state == 1)
} // END OF pose_update()

// THIS FUNCTION: Updates the goal pose for the propulsion_system given by mission_control
// ACCEPTS: usv_pose_msg from "current_goal_pose"
// RETURNS: (VOID)
// =============================================================================
void goal_pose_update(const amore::usv_pose_msg::ConstPtr& goal) 
{
	if (PS_state == 1)						// if the propulsion_system is ON
	{												// update NED goal position and orientation
		x_goal = goal->position.x;		
		y_goal = goal->position.y;
		psi_goal = goal->psi.data;
	}
} // END OF goal_pose_update()

// THIS FUNCTION UPDATES THE GAINS
// ACCEPTS NOTHING (VOID) 
// RETURNS NOTHING (VOID) 
// =====================================================
void update_parameters_function()
{
	// GET ALL PARAMETERS FROM LAUNCH FILE
	ros::param::get("/Kp_xy_G", Kp_x);
	Kp_y = Kp_x; //ros::param::get("/Kp_y_G", Kp_y);
	ros::param::get("/Kp_psi_G", Kp_psi);
	ros::param::get("/Kd_xy_G", Kd_x);
	Kd_y = Kd_x; //ros::param::get("/Kd_y_G", Kd_y);
	ros::param::get("/Kd_psi_G", Kd_psi);
	ros::param::get("/Ki_xy_G", Ki_x);
	Ki_y = Ki_x; //::param::get("/Ki_y_G", Ki_y);
	ros::param::get("/Ki_psi_G", Ki_psi);
} // END OF update_parameters_function()

// THIS FUNCTION DISPLAYS THE UPDATED GAINS
// ACCEPTS NOTHING (VOID) 
// RETURNS NOTHING (VOID) 
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

// THIS FUNCTION: Resets the integral term once the errors become minimal 
//									to avoid overshooting the goal if a huge effort's built up
// ACCEPTS NOTHING (VOID) 
// RETURNS NOTHING (VOID) 
// =====================================================
void Integral_reset()
{
	if ((float)abs(e_x) < 0.1)		// NOTE: this value may need adjusting
	{
	  e_x_total = 0.0;
	}
	if ((float)abs(e_y) < 0.1)		// NOTE: this value may need adjusting
	{
	  e_y_total = 0.0;
	}
	if ((float)abs(e_psi) < 0.5)	// NOTE: this value may need adjusting
	{
	  e_psi_total = 0.0;
	}
} // END OF Integral_reset()

// THIS FUNCTION: Checks for saturation of thrust outputs and corrects if so
// ACCEPTS NOTHING (VOID) 
// RETURNS NOTHING (VOID) 
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
  ros::NodeHandle nh1, nh2, nh3, nh4, nh5, nh6, nh7, nh8, nh9, nh10;
  
  // Subscribers
  ros::Subscriber ps_state_sub = nh1.subscribe("ps_state", 1, state_update);																// current position converted to NED
  ros::Subscriber nav_NED_sub = nh2.subscribe("nav_ned", 1, pose_update);															// current pose converted to NED
  ros::Subscriber current_goal_pose_sub = nh3.subscribe("current_goal_pose", 1, goal_pose_update);				// goal pose given by path planners
  
  // Publishers
  ps_initialization_state_pub = nh6.advertise<std_msgs::Bool>("ps_initialization_state", 1);										// state of initialization
  ros::Publisher stbd_T_pub = nh7.advertise<std_msgs::Float32>("/wamv/thrusters/right_thrust_cmd", 10);			// float value between -1.0 and 1.0, speed to right thruster
  ros::Publisher port_T_pub = nh8.advertise<std_msgs::Float32>("/wamv/thrusters/left_thrust_cmd", 10);				// float value between -1.0 and 1.0, speed to left thruster
  ros::Publisher stbd_A_pub = nh9.advertise<std_msgs::Float32>("/wamv/thrusters/right_thrust_angle", 10);		// float value between -PI to PI, angle to right thruster
  ros::Publisher port_A_pub = nh10.advertise<std_msgs::Float32>("/wamv/thrusters/left_thrust_angle", 10);			// float value between -PI to PI, angle to left thruster
  
  // Local variables
  std_msgs::Float32 LT, RT, LA, RA;									// LT is left thrust, RT is right thrust, LA is left thruster angle, RA is right thruster angle
  
  // Initialize global variables
  ps_initialization_status.data = false;
  current_time = ros::Time::now();										// sets current time to the time it is now
  last_time = current_time;												// sets last time to the time it is now
  
  //sets the frequency for which the program sleeps at. 10=1/10 second
  ros::Rate loop_rate(3);

  //rosk::ok() will stop when the user inputs Ctrl+C
  while(ros::ok())
  {
	  current_time = ros::Time::now();													// update current_time
	  
	  PROPULSION_SYSTEM_inspector();											// check initialization status and update ps_initialization_status
	  
	  if (PS_state == 1)
	  {
		  update_parameters_function();			// Update all parameters in launch: gain values
		  //display_gains();									// UNCOMMENT IF YOU WANT TO PRINT GAINS TO USER 
		  
		  Integral_reset();										// Reset integral term once the errors become minimal 
		  
		  // determine error in x and y (position)
		  e_x = x_goal - x_usv_NED;
		  e_y = y_goal - y_usv_NED;
		  
		  // calculate magnitude of positional error
		  e_xy = sqrt(pow(e_x,2.0)+pow(e_y,2.0));
		  
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
		  if (loop_count<6) // don't include differential term or integration term first time through		!!!!!!!!!!!!!!!!!!! WHATS THE 6 DOING?
		  {
			  T_x = Kp_x*e_x;
			  T_y = Kp_y*e_y;
		  }
		  else
		  {
			  // trapezoidal integration of errors for integral term
			  e_x_total = e_x_total + ((e_x_prev + e_x)/2.0)*dt; 
			  e_y_total = e_y_total + ((e_y_prev + e_y)/2.0)*dt;
			  /* ROS_DEBUG("E_X_Total = %f\n", e_x_total);
			  ROS_DEBUG("E_Y_Total = %f\n", e_y_total); */
			  
			  T_x = Kp_x*e_x + Kd_x*((e_x - e_x_prev)/dt) + Ki_x*e_x_total;
			  T_y = Kp_y*e_y + Kd_y*((e_y - e_y_prev)/dt) + Ki_y*e_y_total; 
		  }
		  
		  // Give desired heading at goal once within 5.0 meters
		  if (e_xy > 10.0)
		  {
			  psi_goal = atan2(e_y,e_x);       // [radians] desired heading to travel to goal location
		  }
		  
		  e_psi = psi_goal - psi_NED;
		  
		  // correct discontinuity in heading error
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
		  }
		  
		  /* // if within error, have selective gains
		  if ((float)abs(e_psi) < 0.1)
		  {
			  Kp_psi = 0.0;
			  Kd_psi = 0.0;
			  Ki_psi = 0.0;
		  } */
		  
		  // orientation control law
		  if (loop_count<6) // don't include differential term or integration term first time through
		  {
			  M_z = Kp_psi*e_psi;
		  }
		  else
		  {
			  // trapezoidal integration of error for integral term
			  e_psi_total = e_psi_total + ((e_psi_prev + e_psi)/2.0)*dt; 
			  //ROS_DEBUG("E_PSI_Total = %f\n", e_psi_total);
			  
			  M_z = Kp_psi*e_psi + Kd_psi*((e_psi - e_psi_prev)/dt) + Ki_psi*e_psi_total;
		  }
		  
		  // UPDATES STATUSES TO USER ///////////////////////////////////////////////
		  ROS_DEBUG("x_goal: %.2f --PS", x_goal);
		  ROS_DEBUG("y_goal: %.2f --PS", y_goal);
		  ROS_DEBUG("des_psi: %.2f --PS\n", psi_goal);
		  
		  ROS_DEBUG("x_usv: %.2f --PS", x_usv_NED);
		  ROS_DEBUG("y_usv: %.2f --PS", y_usv_NED);
		  ROS_DEBUG("psi_NED: %.2f --PS\n", psi_NED);
		  
		  ROS_DEBUG("e_x: %.2f --PS", e_x);                             // x posn. error
		  ROS_DEBUG("e_y: %.2f --PS", e_y);                             // y posn. error
		  ROS_DEBUG("e_xy: %.2f --PS", e_xy);                             // magnitude of posn. error
		  ROS_DEBUG("e_psi: %.2f --PS\n", e_psi);                        // heading error	  
		  
		  ROS_DEBUG("Control Efforts --PS");
		  ROS_DEBUG("T_x_G: %.3f --PS", T_x);
		  ROS_DEBUG("T_y_G: %.3f --PS", T_y);
		  ROS_DEBUG("M_z: %.3f --PS\n", M_z);
		  
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
		  
		  T_p = sqrt(pow(F_xp,2.0)+pow(F_yp,2.0));                 // calculate magnitude of port thrust
		  T_s = sqrt(pow(F_xs,2.0)+pow(F_ys,2.0));                 // calculate magnitude of starboard thrust
		  A_p = -atan2(F_yp,F_xp);                                              // calculate angle of port thrust
		  A_s = -atan2(F_ys,F_xs);                                              // calculate angle of starboard thrust
		  
		  /* ROS_DEBUG("Before--------------");
		  ROS_DEBUG("Port Thrust: %.2f", T_p);
		  ROS_DEBUG("Stbd Thrust: %.2f", T_s);
		  ROS_DEBUG("Port Angle: %.2f", A_p);
		  ROS_DEBUG("Stbd Angle: %.2f", A_s); */
		  
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
		  if ((loop_count>30) && (PS_state == 1))                                                 
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
	  } // if (PS_state == 1)
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
	  }
	  
	  loop_count += 1;
	  ros::spinOnce();
	  loop_rate.sleep();
  }
  LT.data = 0.0;
  port_T_pub.publish(LT);
  RT.data = 0.0;	
  stbd_T_pub.publish(RT);

  ros::spinOnce();
  loop_rate.sleep();

  return 0;
}
