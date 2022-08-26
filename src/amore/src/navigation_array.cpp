//  Filename:  navigation_array.cpp
//  Creation Date:  1/22/2022
//  Last Revision Date:  8/23/2022
//  Author(s) [email]:  Taylor Lamorie [tlamorie@lssu.edu]
//                                                  
//  Revisor(s) [Revision Date]: Shaede Perzanowski [sperzanowski1@lssu.edu], Brad Hacker [bhacker@lssu.edu]
//  Organization/Institution:  Lake Superior State University RobotX Team AMORE
//
//...................................................About navigation_array.cpp.....................................................................
// Used to convert USV pose from geodetic position from GPS and quarternion orientation from the IMU.
// Interacts with the geonav_transform package to convert lat/long coordinates to ENU. Then this code has a function
// that converts to a NED (North-x, East-y, Down-z) coordinate system that is more user friendly to work with in the maritime field.
// Subscribes to all sensors to get information to fill message needed for geonav_transform package. It's purposes 
// is to provide the system with NED pose information.

// Inputs and Outputs of the navigation_array.cpp file
//		Inputs: {"MC_na_state" - state from mission_control}, {"geonav_odom" - geonav_transform package topic containing the transform of geodetic position}
//			{"/wamv/sensors/gps/gps/fix", "/wamv/sensors/gps/gps/fix_velocity"- VRX GPS gives USV position in World Geodetic System and linear velocities}
//			{"/wamv/sensors/imu/imu/data" - VRX IMU gives USV 3D heading and angular velocities}
//
//		Outputs: "NA_initialization_state", "nav_odom", "NA_nav_ned"
	
//...............................................................................................Included Libraries and Message Types.........................................................................................
#include "ros/ros.h"
#include "ros/console.h"
#include "time.h"
#include <sstream>
#include <iostream>
#include "math.h"
#include "stdio.h"
#include "amore/state.h"  // message type used to recieve state of operation from mission_control
#include "std_msgs/Bool.h"  // message type used for communicating initialization status to mission_control
#include "sensor_msgs/NavSatFix.h"  // message type of lat and long coordinates given by the GPS
#include "geometry_msgs/Vector3Stamped.h"  // message type of linear velocities given by the GPS
#include "sensor_msgs/Imu.h"  // message type of quaternion orientation and angular velocities given by the IMU
#include "nav_msgs/Odometry.h"  // message type used for sending USV state
//......................................................................................End of Included Libraries and Message Types.....................................................................................

//.......................................................................................................................Constants.....................................................................................................................
#define PI 3.14159265
//................................................................................................................End of Constants...............................................................................................................

//.................................................................................................................Global Variables..............................................................................................................
int loop_count = 0;  // loop counter

//	STATES CONCERNED WITH "navigation_array"
int NA_state = 0;
//	0 = On standby
//	1 = USV NED state converter

double latitude, longitude, altitude;  // geodetic coordinates
float vx, vy, vz;  // linear velocities
float qx, qy, qz, qw;  // quaternion orientation
float omega_x, omega_y, omega_z;  // angular velocities

double xNED, yNED, zNED;  // NED position
float vxNED, vyNED, vzNED;  // NED linear velocities
float q1NED, q2NED, q3NED, q0NED;  // NED quarternion orientation
float phiNED, thetaNED, psiNED;  // NED euler orientation
float omega_xNED, omega_yNED, omega_zNED;  // NED angular velocities

std_msgs::Bool NA_initialization_state_msg;  // "NA_initialization_state" message
ros::Publisher NA_initialization_state_pub;  // "NA_initialization_state" publisher

nav_msgs::Odometry nav_odom_msg, NA_nav_ned_msg;  // "nav_odom" and "NA_nav_ned" messages, respectively
ros::Publisher nav_odom_pub;  // "nav_odom" publisher
ros::Publisher NA_nav_ned_pub;  // "NA_nav_ned" publisher

ros::Time current_time, last_time;  // creates time variables
//..........................................................................................................End of Global Variables........................................................................................................

//.....................................................................................................................Functions.......................................................................................................................
// THIS FUNCTION: Updates global current_time, loop_count, and publishes initialization status to "NA_initialization_state"
// ACCEPTS: (VOID)
// RETURNS: (VOID)
//=============================================================================================================
void NAVIGATION_ARRAY_inspector()
{
	current_time = ros::Time::now();  // sets current_time to the time it is now
	loop_count += 1;  // increment loop counter
	if (loop_count > 3)
	{
		NA_initialization_state_msg.data = true;
		//ROS_INFO("NAVIGATION_ARRAY: navigation_array_initialized");
	}
	else
	{
		NA_initialization_state_msg.data = false;
		//ROS_INFO("NAVIGATION_ARRAY: !navigation_array_initialized");
	}
	NA_initialization_state_pub.publish(NA_initialization_state_msg);  // publish the initialization status of the navigation_array to "NA_initialization_state"
} // END OF NAVIGATION_ARRAY_inspector()

// THIS FUNCTION: Updates the state of "navigation_array" given by "mission_control"
// ACCEPTS: amore::state from "MC_na_state"
// RETURNS: (VOID)
//=============================================================================================================
void MC_na_state_update(const amore::state::ConstPtr& msg)
{
	if (NA_initialization_state_msg.data)
	{
		NA_state = msg->state.data;
		//ROS_INFO("NAVIGATION_ARRAY: NA_state = %i", NA_state);
	}
} // END OF MC_na_state_update()

// THIS FUNCTION: Updates the USV position in geodetic coordinates 
// ACCEPTS: sensor_msgs::NavSatFix from "/wamv/sensors/gps/gps/fix"
// RETURNS: (VOID)
//=============================================================================================================
void GPS_Position_update(const sensor_msgs::NavSatFix::ConstPtr& gps_msg)
{
	latitude = gps_msg->latitude; //sets latitude from gps
	longitude = gps_msg->longitude; //sets longitude from gps
	altitude = gps_msg->altitude; //sets altitude rom gps
} // END OF GPS_Position_update()

// THIS FUNCTION: Updates the USV linear velocities
// ACCEPTS: geometry_msgs::Vector3Stamped from "/wamv/sensors/gps/gps/fix_velocity"
// RETURNS: (VOID)
//=============================================================================================================
void GPS_Velocity_update(const geometry_msgs::Vector3Stamped::ConstPtr& vel_msg) 
{
	// gather the velocity, which is in NWU, wrt the GPS sensor (which I think is almost the USV origin)
	vx = vel_msg->vector.x;
	vy = vel_msg->vector.y;
	vz = vel_msg->vector.z;
} // END OF GPS_Velocity_update()

// THIS FUNCTION: Updates the USV quaternion orientation and angular velocities
// ACCEPTS: sensor_msgs::Imu from "/wamv/sensors/imu/imu/data"
// RETURNS: (VOID)
//=============================================================================================================
 void IMU_processor(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
	// gather orientation quaternion
	qx = imu_msg->orientation.x;
	qy = imu_msg->orientation.y;
	qz = imu_msg->orientation.z;
	qw = imu_msg->orientation.w;
	
	// gather body-fixed angular velocity
	omega_x = imu_msg->angular_velocity.x;
	omega_y = imu_msg->angular_velocity.y;
	omega_z = imu_msg->angular_velocity.z;
} // END OF IMU_processor()

// THIS FUNCTION: Converts the current pose from ENU -> NED
// ACCEPTS: nav_msgs::Odometry from "geonav_odom"
// RETURNS: (VOID)
//=============================================================================================================
void NED_Func(const nav_msgs::Odometry::ConstPtr& enu_state)
{
	// Variables
	float q1, q2, q3, q0, phi, theta, psi;
	
	// Convert the position to NED from ENU
	xNED = enu_state->pose.pose.position.y;
	yNED = enu_state->pose.pose.position.x;
	zNED = -(enu_state->pose.pose.position.z);
	// Convert from quaternion into radians
	q1 = enu_state->pose.pose.orientation.x;
	q2 = enu_state->pose.pose.orientation.y;
	q3 = enu_state->pose.pose.orientation.z;
	q0 = enu_state->pose.pose.orientation.w;
	phi = atan2((2.0*(q1*q0 + q3*q2)) , (1.0 - 2.0*(pow(q1,2.0) + pow(q2,2.0)))); 
	theta = asin(2.0*(q0*q2 - q1*q3));
	psi = atan2((2.0*(q3*q0 + q1*q2)) , (1.0 - 2.0*(pow(q2,2.0) + pow(q3,2.0))));  // orientation off x-axis
	// Convert the orientation to NED from ENU
	phiNED = theta;
	thetaNED = phi;
	psiNED = PI/2.0 - psi;
	// Adjust psiNED back within -PI and PI
	if (psiNED < -PI)
	{
		psiNED = psiNED + 2.0*PI;
	}
	if (psiNED > PI)
	{
		psiNED = psiNED - 2.0*PI;
	}
	// Convert the linear velocity to NED from NWU
	vxNED = vx;
	vyNED = -vy;
	vzNED = -vz;
	// Convert the angular velocity to NED
} // END OF NED_Func()

// THIS FUNCTION: Fills out nav_odom_msg and publishes to "nav_odom"
// ACCEPTS: (VOID)
// RETURNS: (VOID)
//=============================================================================================================
void nav_odom_publish()
{
	// Fill the odometry header for nav_odom_msg
	nav_odom_msg.header.seq +=1;  // sequence number
	nav_odom_msg.header.stamp = current_time;  // sets stamp to current time
	nav_odom_msg.header.frame_id = "odom";  // header frame
	nav_odom_msg.child_frame_id = "base_link";  // child frame

	// Fill the pose
	nav_odom_msg.pose.pose.position.x = longitude;
	nav_odom_msg.pose.pose.position.y = latitude;
	nav_odom_msg.pose.pose.position.z = altitude;
	nav_odom_msg.pose.pose.orientation.x = qx;
	nav_odom_msg.pose.pose.orientation.y = qy;
	nav_odom_msg.pose.pose.orientation.z = qz;
	nav_odom_msg.pose.pose.orientation.w = qw;
	nav_odom_msg.pose.covariance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

	// Fill the velocities
	nav_odom_msg.twist.twist.linear.x = vx;
	nav_odom_msg.twist.twist.linear.y = vy;
	nav_odom_msg.twist.twist.linear.z = vz;
	nav_odom_msg.twist.twist.angular.x = omega_x;
	nav_odom_msg.twist.twist.angular.y = omega_y;
	nav_odom_msg.twist.twist.angular.z = omega_z;
	nav_odom_msg.twist.covariance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

	// publish the state to be transformed to ENU
	nav_odom_pub.publish(nav_odom_msg);
} // END OF nav_odom_publish()
//.............................................................................................................END OF Functions...............................................................................................................

//..................................................................................................................Main Program..................................................................................................................
int main(int argc, char **argv)
{
	// names the program for visual purposes
	ros::init(argc, argv, "navigation_array");
  
	// NodeHandles
	ros::NodeHandle nh1, nh2, nh3, nh4, nh5, nh6;
  
	// Subscribers
	// from mission_control
	ros::Subscriber MC_na_state_sub = nh1.subscribe("MC_na_state", 1, MC_na_state_update);  // navigation_array state published by mission_control
	// from system interface with gazebo or physical sensors
	ros::Subscriber gpspos_sub = nh2.subscribe("/wamv/sensors/gps/gps/fix", 1, GPS_Position_update);  // subscribes to GPS position
	ros::Subscriber gpsvel_sub = nh3.subscribe("/wamv/sensors/gps/gps/fix_velocity", 1, GPS_Velocity_update);  // subscribes to GPS velocity
	ros::Subscriber imu_sub = nh4.subscribe("/wamv/sensors/imu/imu/data", 1, IMU_processor);  // subscribes to IMU to get headings
	// from geonav_transform package
	ros::Subscriber geonav_odom_sub = nh5.subscribe("geonav_odom", 1, NED_Func);  // HOPEFULLY I GET RID OFF THIS PACKAGE AND WE DO THE TRANSFORMATION OURSELVES
	
	// Publishers
	// to mission_control
	NA_initialization_state_pub = nh6.advertise<std_msgs::Bool>("NA_initialization_state", 1);  // publisher for state of initialization
	nav_odom_pub = nh6.advertise<nav_msgs::Odometry>("nav_odom", 1);  // USV state publisher, this sends the current state to be converted to "nav_odom", so geonav_transform package can publish the ENU conversion to "geonav_odom"
	NA_nav_ned_pub = nh6.advertise<nav_msgs::Odometry>("NA_nav_ned", 1);  // USV NED state publisher
	
	// Initialize global variables
	NA_initialization_state_msg.data = false;
	current_time = ros::Time::now();  // sets current time to the time it is now
	last_time = current_time;  // sets last time to the time it is now
	  
	// sets the frequency for which the program loops at 10 = 1/10 second
	ros::Rate loop_rate(20);  // {Hz} GPS update rate: 20, IMU update rate: 100

	while(ros::ok())
	{
		NAVIGATION_ARRAY_inspector();  // check, update, and publish NA_initialization_state_msg
		
		//  NA_state LEGEND
		//	0 = On standby
		//	1 = USV NED state converter
		switch(NA_state)
		{
			case 0:  // On standby
				ros::spinOnce();  // update subscribers
				loop_rate.sleep();  // sleep to accomplish set loop_rate
				last_time = current_time;  // update last_time
				break;
				
			case 1:  // USV NED state converter
				ros::spinOnce();  // update subscribers to get updated USV state from GPS and IMU sensors
				
				nav_odom_publish();  // fills out nav_odom_msg and publishes to "nav_odom"  -- UNCOMMENT THIS ASAP
				
				ros::spinOnce();  // update subscribers to get updated NED converted pose
				
				// Fill NA_nav_ned_msg, the USV state in NED
				// Fill the header
				NA_nav_ned_msg.header.seq +=1;  // sequence number
				NA_nav_ned_msg.header.stamp = current_time;  // sets stamp to current time
				NA_nav_ned_msg.header.frame_id = "odom";  // header frame
				NA_nav_ned_msg.child_frame_id = "base_link";  // child frame
				
				// Fill the USV pose in NED
				NA_nav_ned_msg.pose.pose.position.x = xNED;
				NA_nav_ned_msg.pose.pose.position.y = yNED;
				NA_nav_ned_msg.pose.pose.position.z = zNED;
				NA_nav_ned_msg.pose.pose.orientation.x = phiNED;
				NA_nav_ned_msg.pose.pose.orientation.y = thetaNED;
				NA_nav_ned_msg.pose.pose.orientation.z = psiNED;
				NA_nav_ned_msg.pose.pose.orientation.w = 1.23456;  // not used
				NA_nav_ned_msg.pose.covariance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
				
				// Fill the USV velocities in NED
				NA_nav_ned_msg.twist.twist.linear.x = vxNED;
				NA_nav_ned_msg.twist.twist.linear.y = vyNED;
				NA_nav_ned_msg.twist.twist.linear.z = vzNED;
				NA_nav_ned_msg.twist.twist.angular.x = omega_x;
				NA_nav_ned_msg.twist.twist.angular.y = omega_y;
				NA_nav_ned_msg.twist.twist.angular.z = omega_z;
				NA_nav_ned_msg.twist.covariance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
				
				// publish the USV state in NED
				NA_nav_ned_pub.publish(NA_nav_ned_msg);
				
				loop_rate.sleep();  // sleep to accomplish set loop_rate
				last_time = current_time;  // update last_time
				break;
				
			default:
				loop_rate.sleep();  // sleep to accomplish set loop_rate
				last_time = current_time;  // update last_time
				break;
		}
	}  // END OF while(ros::ok())
	
	return 0;
}  // END OF main()
//.........................................................................................................END OF Main Program...........................................................................................................