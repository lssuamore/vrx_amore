//  Filename:						        PID_HP_controller.cpp
//  Creation Date:						12/4/2021
//  Last Revision Date:                
//  Author(s) [email]:					Brad Hacker [bhacker@lssu.edu]
//                                                  Shaede Perzanowksi [sperzanowski1@lssu.edu]
//  Revisor(s) [Revision Date]:    
//  Organization/Institution:			Lake Superior State University
// 
// ...........................PID_HP_controller.cpp.......................
//  This code takes in a position to reach in the global frame as nav_odom msgs. 
//  It gets this position by subscribing to the "mpp_goal" node created by the path planner.
//  This controller then uses a PID controller to control the heading and position of the USV.
//  This controller uses a differential drive configuration.
//
//  Inputs and Outputs of the PID_HP_controller.cpp file
//				Inputs: Position (x,y) to reach in global NED frame
//				Outputs: Thrust outputs for left and right thrusters

// Includes all of the ROS libraries needed
#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "ros/console.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3Stamped.h"
#include <sstream>
#include <iostream>
#include "stdio.h"
#include "time.h"

// include necessary message libraries
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"

// DEFINING GLOBAL VARIABLES
#define PI 3.14159265
// ---------------------------------------------------------------------------------------------------------------------
bool goal = false;            // goal = false means goal has not been reached, goal = true means reached, starts out as false
int loop_count = 0;            // used to keep track of loop, first 10 loops are used to just intitialize the subscribers
float duration = 1000;      // amount of time to finish the path 
float dt = 0.25;                   // [s] used for differential term
float B = 2.44;                  // [m] 8 ft or 2.44 m

float x_usv_ENU;           // this variable is updated as the WAM-V x position in real time by transfering lat and long 
float y_usv_ENU;           // this variable is updated as the WAM-V y position in real time by transfering lat and long
float x_usv_NED;           // this variable is updated as the WAM-V x position in NED
float y_usv_NED;           // this variable is updated as the WAM-V y position in NED

// q0 - q3 are current orientation in quaternion form
float q1;            
float q2;
float q3;
float q0;
float psi_ENU;                          // this variable is updated as the WAM-V z-orientation in real time through the compass
float psi_NED;                // this variable is updated as the WAM-V z-orientation in NED

float ON_OFF = 0.0;         // if this variable is a 0.0, the controller is OFF (will not print to thrusters), if it is a 1.0, the controller is ON (will print to thrusters)

/* // harcoded goal
float x_goal = 29.85;                  // this variable is updated as the WAM-V x goal position through the PID_pub
float y_goal = 8.975;                  // this variable is updated as the WAM-V y goal position through the PID_pub */

// goal position
float x_goal;
float y_goal;

float psi_desired = 3.0;       // [radians] desired heading to hold at goal location

float psiD;                       // this is calculated as the desired z-orientation 

float e_x;                        // this is calculated as the error between the desired x-pos. and the actual x-pos.
float e_y;                        // this is calculated as the error between the desired y-pos. and the actual y-pos.
float e_xy;                      // this is calculated as the error between the desired xy-pos. and the actual xy-pos.
float e_psi;                     // this is calculated as the error between the desired z-orientation and the actual z-orientation

float e_xy_total = 0;      // used for I term
float e_psi_total = 0;     // used for I term
float e_xy_prev = 0;     // holds previous xy error
float e_psi_prev = 0;    // holds previous heading error

float T_x;              // this is continuously calculated as the thrust to set in x-direction
float M_z;             // this is calculated as the desired moment around the z-axis

float T_port;          // this is used to set the port (left) thruster
float T_stbd;         // this is used to set the starboard (right) thruster

// Used for correction of saturated thrust values 
float T_port_C;          // Corrected port (left) thrust output
float T_stbd_C;         // Corrected starboard (right) thrust output

// Proportional gains
float Kp_xy = 0.7;               // Position proportional gain in x-direction
float Kp_psi = 8.0;              // z-orientation proportional gain
// Differential gains
float Kd_xy = 1.0;               // Position differential gain in x-direction
float Kd_psi = 12.0;              // z-orientation differential gain
// Integration gains
float Ki_xy = 0.0;                // Position integration gain in x-direction
float Ki_psi = 0.0;               // z-orientation integration gain

void pose_update(const nav_msgs::Odometry::ConstPtr& odom) 
{
	x_usv_ENU = odom->pose.pose.position.x;
	y_usv_ENU = odom->pose.pose.position.y;
	
	q1 = odom->pose.pose.orientation.x;
	q2 = odom->pose.pose.orientation.y;
	q3 = odom->pose.pose.orientation.z;
	q0 = odom->pose.pose.orientation.w;
	
	// convert current heading from quaternion into radians
	psi_ENU = atan2((2.0*(q3*q0 + q1*q2)) , (1 - 2*(pow(q2,2.0) + pow(q3,2.0)))); // orientation off x-axis
	
	// convert from ENU to NED
    x_usv_NED = y_usv_ENU;
	y_usv_NED = x_usv_ENU;
	psi_NED = PI/2.0 - psi_ENU;
	// adjust current heading back within -PI and PI
	if (psi_NED < -PI)
	{
   	  psi_NED = psi_NED + 2*PI;
	}
	if (psi_NED > PI)
	{
	  psi_NED = psi_NED - 2*PI;
	} 
}

void goal_pose_update(const nav_msgs::Odometry::ConstPtr& goal) 
{
	x_goal = goal->pose.pose.position.x;
	y_goal = goal->pose.pose.position.y;
	ON_OFF = goal->pose.pose.position.z;
}

// THIS FUNCTION UPDATES THE GAINS -------
// ACCEPTS NOTHING (VOID) ------------------------------------------------------
// RETURNS NOTHING (VOID) ------------------------------------------------------
// =============================================================================
void parameters_function()
{
	// GET ALL PARAMETERS FROM LAUNCH FILE
	ros::param::get("/Kp_xy_G", Kp_xy);
	ros::param::get("/Kp_psi_G", Kp_psi);
	ros::param::get("/Kd_xy_G", Kd_xy);
	ros::param::get("/Kd_psi_G", Kd_psi);
	ros::param::get("/Ki_xy_G", Ki_xy);
	ros::param::get("/Ki_psi_G", Ki_psi);
	ROS_INFO("Kp_xy is: %f\n", Kp_xy);
	ROS_INFO("Kp_psi is: %f\n", Kp_psi);
	ROS_INFO("Kd_xy is: %f\n", Kd_xy);
	ROS_INFO("Kd_psi is: %f\n", Kd_psi);
	ROS_INFO("Ki_xy is: %f\n", Ki_xy);
	ROS_INFO("Ki_psi is: %f\n", Ki_psi);
}

int main(int argc, char **argv)
{
  //names the program for visual purposes
  ros::init(argc, argv, "PID_HP_controller");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
  
  // set up NodeHandles
  ros::NodeHandle nh3;  // this node handle is used for subscribing to geonav_odom which provides position in x, y
  ros::NodeHandle nh4;  // this node handle is used for subscribing to the desired pose, which is published by the high level controller
  ros::NodeHandle nh5;  // this node handle is used for publishing thrust information to starboard side 
  ros::NodeHandle nh6;  // this node handle is used for publishing thrust information to port side
  ros::NodeHandle nh7;  // this node handle is used for publishing thrust angle to starboard side 
  ros::NodeHandle nh8;  // this node handle is used for publishing thrust angle to port side
  
  // start publishers and subscribers
  ros::Subscriber nav_sub = nh3.subscribe("geonav_odom", 1, pose_update);                                                     // subscribes to geonav_odom
  ros::Subscriber pose_sub = nh4.subscribe("mpp_goal", 1, goal_pose_update);                                                  // subscribes to mpp_goal
  ros::Publisher stbd_T_pub = nh5.advertise<std_msgs::Float32>("/wamv/thrusters/right_thrust_cmd", 10);     // publishes float value between -1.0 and 1.0 to right thruster
  ros::Publisher port_T_pub = nh6.advertise<std_msgs::Float32>("/wamv/thrusters/left_thrust_cmd", 10);        // publishes float value between -1.0 and 1.0 to left thruster
  ros::Publisher stbd_A_pub = nh7.advertise<std_msgs::Float32>("/wamv/thrusters/right_thrust_angle", 10);     // publishes float value between -PI to PI, angle to right thruster
  ros::Publisher port_A_pub = nh8.advertise<std_msgs::Float32>("/wamv/thrusters/left_thrust_angle", 10);        // publishes float value between -PI to PI, angle to left thruster
  
  // local variables //////////////////////////////////////////////////////////////////////////////////////////////////////////////
  std_msgs::Float32 LT, RT, LA, RA;            // LT is left thrust, RT is right thrust, LA is left thruster angle, RA is right thruster angle 
  ros::Time current_time, last_time;  // creates time variables
  current_time = ros::Time::now();   // sets current time to the time it is now
  last_time = ros::Time::now();        // sets last time to the time it is now
  
  //sets the frequency for which the program sleeps at. 4=1/4th second
  ros::Rate loop_rate(4);
  ros::spinOnce();                             // initialize position

  //rosk::ok() will stop when the user inputs Ctrl+C
  while(ros::ok() && !goal)
  {
	  // Update gain values
	  parameters_function();
	  
	  current_time = ros::Time::now(); // update time
	  
	  if ((loop_count > 40) && (ON_OFF == 1.0))
	  { 
		  // if within tolerance, reset integral term 
		  if ((float)abs(e_xy) < 1.0)
    	  {
	    	  e_xy_total = 0.0;
	      }
	      if ((float)abs(e_psi) < 1.0)
	      {
		      e_psi_total = 0.0;
	      }
		  
	      // determine error in x and y (position)
		  ROS_DEBUG("x_goal: %f\n", x_goal);
		  ROS_DEBUG("the x is: %f\n", x_usv_NED);       // displays current x posn.
		  ROS_DEBUG("y_goal: %f\n", y_goal);
		  ROS_DEBUG("the y is: %f\n", y_usv_NED);       // displays current y posn.
	      e_x = x_goal - x_usv_NED;                                                // calculate error in x position
	      e_y = y_goal - y_usv_NED;                                                // calculate error in y position
    	  e_xy = sqrt(pow(e_x,2.0)+pow(e_y,2.0));                           // calculate magnitude of positional error
		  
    	  /* ROS_DEBUG("the x is: %f\n", x_usv_NED);       // displays current x posn.
    	  ROS_DEBUG("the e_x is: %f\n", e_x);       // displays current x posn. error
          ROS_DEBUG("the y is: %f\n", y_usv_NED);       // displays current y posn.
          ROS_DEBUG("the e_y is: %f\n", e_y);       // displays current y posn. error
	      ROS_DEBUG("the e_xy is: %f\n", e_xy);   // displays magnitude of positional error */

     	  // position control law
	      if (loop_count == 11) // don't include differential term or integration term first time through
	      {
    		  T_x = Kp_xy*e_xy;
	      }
	      else
    	  {
	    	  e_xy_total = e_xy_total + ((e_xy_prev + e_xy)/2.0)*dt; // trapezoidal integration of errors for integral term
	    	  T_x = Kp_xy*e_xy+ Kd_xy*((e_xy - e_xy_prev)/dt) + Ki_xy*e_xy_total;
	      }

	      // heading is based off positional error and is given with respect to the sway (x direction)
	      // CCW is positive
	      psiD = atan2(e_y, e_x);  // get desired orientation
		  
	      /* // once within desired error, quit controller
	      if (e_xy < 2.0)
    	  {
	    	  ROS_INFO("Goal Reached!");                                   // inform user goal has been reached
	    	  goal =true;
	 	  } */
		  
		  e_psi = psiD - psi_NED;
		  
		  // correct discontinuity in heading error
		  if (e_psi < ((-2.0*PI)+(0.2*2.0*PI)))
		  {
			  psiD = psiD + 2.0*PI;
			  e_psi = psiD - psi_NED;
		  }
		  if (e_psi > ((2.0*PI)-(0.2*2.0*PI)))
		  {
			  psiD = psiD - 2.0*PI;
			  e_psi = psiD - psi_NED;
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
		  
		  ROS_DEBUG("psi is: %f\n", psi_NED);                            // displays current heading
		  ROS_DEBUG("psiD is: %f\n", psiD);                                 // displays desired heading
		  ROS_DEBUG("the e_psi is: %f\n", e_psi);                       // displays heading error

		  // orientation control law
		  if (loop_count == 11) // don't include differential term or integration term first time through
		  {
			  M_z = Kp_psi*e_psi;
		  }
		  else
		  {
			  e_psi_total = e_psi_total + ((e_psi_prev + e_psi)/2.0)*dt; // trapezoidal integration of errors for integral term
			  M_z = Kp_psi*e_psi + Kd_psi*((e_psi - e_psi_prev)/dt) + Ki_psi*e_psi_total;
		  }

		  // update previous errors
		  e_xy_prev = e_xy; 
		  e_psi_prev = e_psi;
		  
		  // if heading error is bigger than 14.3 degrees, control heading only
		  if ((float)abs(e_psi) > 0.4)
		  {
			  T_x = 0.0;
		  }
		  
		  // Calculate torque to thrusters
		  T_port = T_x/2.0 + M_z/B;
		  T_stbd = T_x/2.0 - M_z/B;
		  
		  // display contributions of output 
		  ROS_WARN("LT_T is: %f\n", T_x/2.0);            // displays left thrust value
		  ROS_WARN("LT_M is: %f\n", M_z/B);          // displays right thrust value
		  ROS_WARN("RT_T is: %f\n", T_x/2.0);            // displays left thrust value
		  ROS_WARN("RT_M is: %f\n", -M_z/B);          // displays right thrust value

		  ROS_DEBUG("LT is: %f\n", T_port);            // displays left thrust value
		  ROS_DEBUG("RT is: %f\n", T_stbd);          // displays right thrust value
		  
		  // check for saturation, if so standardize back within allowable output range
		  if (((float)abs(T_port) > 1.0) || ((float)abs(T_stbd) > 1.0))
		  {
			  // correct for saturation by normalizing thrust data
			  if ((float)abs(T_port) > (float)abs(T_stbd))
			  {
				  //ROS_DEBUG("LT IS ISSUE!");
				  T_port_C= T_port / (float)abs(T_port);
				  T_stbd_C = T_stbd / (float)abs(T_port);
			  }
			  else if ((float)abs(T_port) < (float)abs(T_stbd))
			  {
				  //ROS_DEBUG("RT IS ISSUE!");
				  T_port_C = T_port / (float)abs(T_stbd);
				  T_stbd_C = T_stbd / (float)abs(T_stbd);
			  }
			  else 
			  {
				  //ROS_DEBUG("Equal");
				  //ROS_DEBUG("Divide by : %f\n", (float)abs(T_port));          // displays right thrust value
				  T_port_C = T_port / (float)abs(T_port);
				  T_stbd_C = T_stbd / (float)abs(T_port);
			  }
			  T_port = T_port_C;
			  T_stbd = T_stbd_C;
			  ROS_DEBUG("LT after correction is: %f\n", T_port);            // displays left thrust value
		      ROS_DEBUG("RT after correction is: %f\n", T_stbd);          // displays right thrust value
		  }
		  
		  /* // make less aggressive by only allowing half the output possible
		  if (((float)abs(T_port) > 0.5) || ((float)abs(T_stbd) > 0.5))
		  {
			  // cut outputs in half
			  T_port_C= T_port / 2.0;
			  T_stbd_C = T_stbd / 2.0;
			  T_port = T_port_C;
			  T_stbd = T_stbd_C;
			  ROS_DEBUG("LT after extra correction is: %f\n", T_port);            // displays left thrust value
		      ROS_DEBUG("RT after extra correction is: %f\n", T_stbd);          // displays right thrust value
		  } */
		  
		  /* // ensure thrusters are within bounds [-1.0 - 1.0]
		  if (T_port < -1.0)
		  {
			  T_port = -1.0;
		  }
		  if (T_port > 1.0)
		  {
			  T_port = 1.0;
		  }
		  if (T_stbd < -1.0)
		  {
			  T_stbd = -1.0;
		  }
		  if (T_stbd > 1.0)
		  {
			  T_stbd = 1.0;
		  } */
		  
		  if (ON_OFF == 0.0)
		  {
			  LT.data = 0.0;
			  RT.data = 0.0;
		  }
		  else
		  {
			  LT.data = T_port;
			  RT.data = T_stbd;
		  }
		  // Differential Drive
		  LA.data = 0.0;
		  RA.data = 0.0;
		  /* ROS_WARN("LT = %f\n", LT.data);
		  ROS_WARN("RT = %f\n", RT.data); */
		  port_A_pub.publish(LA);
		  port_T_pub.publish(LT);
		  stbd_A_pub.publish(RA);
	      stbd_T_pub.publish(RT);
	  } // if (loop_count>10)
		  
	  last_time = current_time; 
	  ros::spinOnce();
	  loop_rate.sleep();
	  loop_count = loop_count+1;
  } // while(ros::ok() && !goal)
  LT.data = 0.0;
  port_T_pub.publish(LT);
  RT.data = 0.0;	
  stbd_T_pub.publish(RT);

  ros::spinOnce();
  loop_rate.sleep();

  return 0;
}


