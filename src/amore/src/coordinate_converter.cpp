//  Filename:  coordinate_converter.cpp
//  Creation Date:  8/23/2022
//  Last Revision Date:  8/23/2022
//  Author(s) [email]:  Brad Hacker [bhacker@lssu.edu]
//                                                  
//  Revisor(s) [Revision Date]:  Brad Hacker [bhacker@lssu.edu]
//  Organization/Institution:  Lake Superior State University RobotX Team AMORE
//
//...................................................About coordinate_converter.cpp.....................................................................
// Used to interact with the geonav_transform package. This package is used to convert lat/long coordinates
// to a NED (North-x East-y Down-z) coordinate system that is more user friendly to work with. This code
// subscribes to all sensors to get information to fill message needed for geonav_transform package. It's purposes 
// are to provide the system with NED pose information, as well as to subscribe to task goal poses and convert
// them to NED to be published for planner to subscribe to.

// Inputs and Outputs of the coordinate_converter.cpp file
//		Inputs: {"MC_cc_state",  "MC_pp_state" - states from mission_control}, {"/vrx/station_keeping/goal", VRX Tasks 1, 2, and 4 goal poses to be converted through geonav_transform}
//			{"geonav_odom" - geonav_transform package topic containing the transform of geodetic position}, {"PP_USV_pose_update_state" - status of whether or not the path_planner has gotten the updated USV state from navigation_array}
//
//		Outputs: "CC_initialization_state", "CC_goal_poses_publish_state", "nav_odom", "CC_goal_poses_ned", "CC_animals_ned"

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
#include "nav_msgs/Odometry.h"  // message type used for sending NED USV state from coordinate_converter
#include "geographic_msgs/GeoPoseStamped.h"  // message type published by VRX Task 1
#include "geographic_msgs/GeoPath.h"  // message type published by VRX Task 2 & 4
#include "geometry_msgs/Point.h"  // message type used for building "CC_goal_poses_ned" message
#include "amore/NED_poses.h"  // message type created to hold an array of the converted WF goal waypoints w/ headings and the number of goal poses
#include "geometry_msgs/PointStamped.h"  // message type used for building "CC_animals_ned" message
#include "amore/NED_objects.h"  // message type used for "CC_animals_ned" message
//......................................................................................End of Included Libraries and Message Types.....................................................................................

//.......................................................................................................................Constants.....................................................................................................................
#define PI 3.14159265
//#define CONFIRM 2  // count of loops to hold the same data point for before sending next pose to convert
//................................................................................................................End of Constants...............................................................................................................

//.................................................................................................................Global Variables..............................................................................................................
int loop_count = 0;  // loop counter

//	STATES CONCERNED WITH "coordinate_converter"
//	0 = On standby
//	1 = VRX1: Station-Keeping NED goal pose converter
//	2 = VRX2: Wayfinding NED goal pose converter
//	4 = VRX4: Wildlife NED animals converter
int CC_state;

double latitude, longitude, altitude;  // geodetic coordinates
float vx, vy, vz;  // linear velocities
float qx, qy, qz, qw;  // quaternion orientation
float omega_x, omega_y, omega_z;  // angular velocities

double xNED, yNED, zNED;  // NED position
float vxNED, vyNED, vzNED;  // NED linear velocities
float q1NED, q2NED, q3NED, q0NED;  // NED quarternion orientation
float phiNED, thetaNED, psiNED;  // NED euler orientation
float omega_xNED, omega_yNED, omega_zNED;  // NED angular velocities

std_msgs::Bool CC_initialization_state_msg;  // "CC_initialization_state" message
ros::Publisher CC_initialization_state_pub;  // "CC_initialization_state" publisher

nav_msgs::Odometry nav_odom_msg;  // "nav_odom" message
ros::Publisher nav_odom_pub;  // "nav_odom" publisher

ros::Time current_time, last_time;  // creates time variables

int loops_published = 0;  // count of loops to hold the same data point for before sending next
int point_num = 0;  // used to keep track of the point being converted
int goal_poses_quantity = -1;  // used to keep track of the number of goal poses to convert

float qx_goal[100], qy_goal[100], qz_goal[100], qw_goal[100];  // goal pose array headings in quarternion form
float goal_lat[100], goal_long[100];  // goal pose array lat/long coordinates
float NED_x_goal[100], NED_y_goal[100], NED_psi_goal[100];  // NED converted goal pose array

bool lat_lon_goal_recieved = false;  // false means goal waypoint poses in lat and long coordinates have not been recieved
bool lat_lon_pose_sent = false;  // false means the current point has not been published to nav_odom for conversion
bool pose_converted_NED = false;  // false means current goal pose has not been converted from lat/long to NED, this becomes true when lat_lon_pose_sent and its ENU conversion has been subscribed to and then converted with the NED_Func
bool goal_poses_converted_NED = false;  // false means goal poses have not been converted from lat/long to NED

std_msgs::Bool CC_goal_poses_publish_state_msg;  // "CC_goal_poses_publish_state" message; false means goal NED waypoints have not been published
ros::Publisher CC_goal_poses_publish_state_pub;  // "CC_goal_poses_publish_state" publisher for whether NED converted waypoints have been published

amore::NED_poses goal_poses_ned_msg;  // "CC_goal_poses_ned" message
ros::Publisher CC_goal_poses_ned_pub;  // "CC_goal_poses_ned" publisher of goal_poses_ned_msg which holds NED conversion of poses

//  TASK 4 variables
std::string Animal[100];  // string array to hold the names of the animals
amore::NED_objects CC_animals_ned_msg;  // "CC_animals_ned" message used to publish animal locations w/ IDs
ros::Publisher CC_animals_ned_pub;  // "CC_animals_ned" publisher

bool usv_NED_pose_updated = false;  // false means usv NED pose has not been acquired from navigation_array by the path_planner  // TASK 4

//	STATES CONCERNED WITH "path_planner"
int PP_state;
//	0 = On standby
//	1 = VRX1: Station-Keeping
//	2 = VRX2: Wayfinding
//	4 = VRX4: Wildlife Encounter and Avoid
//	5 = VRX5: Channel Navigation, Acoustic Beacon Localization and Obstacle Avoidance
//	6 = VRX6: Scan and Dock and Deliver
//..........................................................................................................End of Global Variables........................................................................................................

//.....................................................................................................................Functions.......................................................................................................................
// THIS FUNCTION: Updates global current_time, loop_count, and publishes initialization status to "CC_initialization_state"
// ACCEPTS: (VOID)
// RETURNS: (VOID)
//=============================================================================================================
void COORDINATE_CONVERTER_inspector()
{
	current_time = ros::Time::now();  // sets current_time to the time it is now
	ros::spinOnce();
	loop_count += 1;  // increment loop counter
	if (loop_count > 3)
	{
		CC_initialization_state_msg.data = true;
		//ROS_INFO("COORDINATE_CONVERTER: coordinate_converter_initialized");
	}
	else
	{
		CC_initialization_state_msg.data = false;
		//ROS_INFO("COORDINATE_CONVERTER: !coordinate_converter_initialized");
	}
	CC_initialization_state_pub.publish(CC_initialization_state_msg);  // publish the initialization status of the coordinate_converter to "CC_initialization_state"
	CC_goal_poses_publish_state_pub.publish(CC_goal_poses_publish_state_msg);  // publish whether NED converted waypoints have been published
}  // END OF COORDINATE_CONVERTER_inspector()

// THIS FUNCTION: Updates the state of "coordinate_converter" given by "mission_control"
// ACCEPTS: amore::state from "MC_cc_state"
// RETURNS: (VOID)
//=============================================================================================================
void MC_cc_state_update(const amore::state::ConstPtr& msg)
{
	if (CC_initialization_state_msg.data)
	{
		CC_state = msg->state.data;
		//ROS_INFO("COORDINATE_CONVERTER: CC_state = %i", CC_state);
	}
}  // END OF MC_cc_state_update()

// THIS FUNCTION: Updates the state of "path_planner" given by "mission_control"
// ACCEPTS: amore::state from "MC_pp_state"
// RETURNS: (VOID)
//=============================================================================================================
void MC_pp_state_update(const amore::state::ConstPtr& msg)
{
	if (CC_initialization_state_msg.data)
	{
		PP_state = msg->state.data;
	}
}  // END OF MC_pp_state_update()

// THIS FUNCTION: Converts the current pose from ENU -> NED
// ACCEPTS: nav_msgs::Odometry from "geonav_odom"
// RETURNS: (VOID)
//=============================================================================================================
void NED_Func(const nav_msgs::Odometry::ConstPtr& enu_state)
{
	if ((!goal_poses_converted_NED) && (lat_lon_pose_sent))  // if all the goal poses have not been converted yet AND the current goal to be converted has been sent
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
		
		pose_converted_NED = true;  // true means lat_lon_pose_sent and its ENU conversion has been subscribed to and then converted with the NED_Func
	}  // END OF if ((!goal_poses_converted_NED) && (lat_lon_pose_sent))
}  // END OF NED_Func()

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
}  // END OF nav_odom_publish()

// THIS FUNCTION: Fills out position and orientation of pose to be converted
// ACCEPTS: (VOID)
// RETURNS: (VOID)
//=============================================================================================================
void goal_convert_update()
{
	latitude = goal_lat[point_num];
	longitude = goal_long[point_num];
	altitude = 0.0;  // [m] sets altitude to 0
	
	// set velocity to zero since it isn't pertinent
	vx = 0.0;
	vy = 0.0;
	vz = 0.0;
	
	// set orientation quaternion
	qx = qx_goal[point_num];
	qy = qy_goal[point_num];
	qz = qz_goal[point_num];
	qw = qw_goal[point_num];
	
	// set body-fixed angular velocity to zero since it isn't pertinent
	omega_x = 0.0;
	omega_y = 0.0;
	omega_z = 0.0;
}  // END OF goal_convert_update()

// THIS FUNCTION: Subscribes to get VRX1: Station-Keeping goal pose
// ACCEPTS: geographic_msgs::GeoPoseStamped from "/vrx/station_keeping/goal"
// RETURNS: (VOID)
//=============================================================================================================
void VRX_T1_goal_update(const geographic_msgs::GeoPoseStamped::ConstPtr& goal)
{
	if ((!lat_lon_goal_recieved) && (CC_state != 0))  // if goal waypoint poses in lat and long coordinates have not been recieved and the coordinate_converter is ON
	{
		goal_lat[point_num] = goal->pose.position.latitude;
		goal_long[point_num] = goal->pose.position.longitude;
		qx_goal[point_num] = goal->pose.orientation.x;
		qy_goal[point_num] = goal->pose.orientation.y;
		qz_goal[point_num] = goal->pose.orientation.z;
		qw_goal[point_num] = goal->pose.orientation.w;
		goal_poses_quantity = 1;
		lat_lon_goal_recieved = true;  // true means goal waypoint poses in lat and long coordinates have been recieved

		// UPDATE USER OF STATUSES
		std::cout << "\n";
		ROS_INFO("COORDINATE_CONVERTER: GOAL POSE ACQUIRED FROM VRX TASK 1");
		ROS_INFO("COORDINATE_CONVERTER: goal_lat: %.2f    goal_long: %.2f\n", goal_lat[point_num], goal_long[point_num]);
	}
}  // END OF VRX_T1_goal_update()

// THIS FUNCTION: Subscribes to get VRX2: Wayfinding goal poses
// ACCEPTS: geographic_msgs::GeoPath from "/vrx/wayfinding/waypoints"
// RETURNS: (VOID)
//=============================================================================================================
void VRX_T2_goal_update(const geographic_msgs::GeoPath::ConstPtr& goal)
{
	if ((!lat_lon_goal_recieved) && (CC_state != 0))  // if goal waypoint poses in lat and long coordinates have not been recieved and the coordinate_converter is ON
	{
		int i = 0;
		current_time = goal->header.stamp;  // used to check to see if there are more poses in message
		while ((goal->poses[i].header.stamp == current_time))
		{
			goal_lat[i] = goal->poses[i].pose.position.latitude;
			goal_long[i] = goal->poses[i].pose.position.longitude;
			qx_goal[i] = goal->poses[i].pose.orientation.x;
			qy_goal[i] = goal->poses[i].pose.orientation.y;
			qz_goal[i] = goal->poses[i].pose.orientation.z;
			qw_goal[i] = goal->poses[i].pose.orientation.w;
			i++;
		}
		goal_poses_quantity = i;
		lat_lon_goal_recieved = true;  // true means goal waypoint poses in lat and long coordinates have been recieved

		// UPDATE USER OF STATUSES
		std::cout << "\n";
		ROS_INFO("COORDINATE_CONVERTER: GOAL POSES ACQUIRED FROM VRX TASK 2");
		ROS_INFO("COORDINATE_CONVERTER: Quantity of goal poses: %i\n", goal_poses_quantity);
	}
}  // END OF VRX_T2_goal_update()

// THIS FUNCTION: Subscribes to get VRX4: Wildlife Encounter and Avoid animal poses
// ACCEPTS: geographic_msgs::GeoPath from "/vrx/wildlife/animals"
// RETURNS: (VOID)
//=============================================================================================================
void VRX_T4_goal_update(const geographic_msgs::GeoPath::ConstPtr& goal)
{
	if ((!lat_lon_goal_recieved) && (CC_state != 0))  // if goal waypoint poses in lat and long coordinates have not been recieved and the coordinate_converter is ON
	{
		int i = 0;
		current_time = goal->header.stamp;  // used to check to see if there are more poses in message
		while ((goal->poses[i].header.stamp == current_time))
		{
			Animal[i] = goal->poses[i].header.frame_id;  // Getting which is the 1st animal
			goal_lat[i] = goal->poses[i].pose.position.latitude;
			goal_long[i] = goal->poses[i].pose.position.longitude;
			qx_goal[i] = goal->poses[i].pose.orientation.x;
			qy_goal[i] = goal->poses[i].pose.orientation.y;
			qz_goal[i] = goal->poses[i].pose.orientation.z;
			qw_goal[i] = goal->poses[i].pose.orientation.w;
			i++;
		}
		goal_poses_quantity = i;
		lat_lon_goal_recieved = true;  // true means goal waypoint poses in lat and long coordinates have been recieved
		
		// UPDATE USER OF STATUSES
		// std::cout << "\n";
		// ROS_INFO("COORDINATE_CONVERTER: GOAL POSES ACQUIRED FROM VRX TASK 4");
		// ROS_INFO("COORDINATE_CONVERTER: Quantity of goal poses (should be 3): %i", goal_poses_quantity);
		// for (int i = 0; i < goal_poses_quantity; i++)
		// {
			// if (i < (goal_poses_quantity-1))
			// {
				// ROS_INFO("COORDINATE_CONVERTER: Animal: %s    lat: %4.9f    long: %4.9f", Animal[i].c_str(), goal_lat[i], goal_long[i]);
			// }
			// else  // if its the last pose print with the next line (\n) to make the monitor display nice
			// {
				// ROS_INFO("COORDINATE_CONVERTER: Animal: %s    lat: %4.9f    long: %4.9f\n", Animal[i].c_str(), goal_lat[i], goal_long[i]);
			// }
		// }
	}
}  // END OF VRX_T4_goal_update()

// THIS FUNCTION: Checks to see if the path_planner has gotten the updated USV state 
// ACCEPTS: std_msgs::Bool from "PP_USV_pose_update_state"
// RETURNS: (VOID)
//=============================================================================================================
void PP_USV_pose_update_state_update(const std_msgs::Bool status)
{
	usv_NED_pose_updated = status.data;
}  // END OF PP_USV_pose_update_state_update()

// THIS FUNCTION: Publishes VRX1 or VRX2 goal poses in local NED convention to "CC_goal_poses_ned"
// ACCEPTS: (VOID)
// RETURNS: (VOID)
//=============================================================================================================
void CC_goal_poses_ned_publish()
{
	std::cout << "\n";
	ROS_INFO("COORDINATE_CONVERTER: PUBLISHING ARRAY OF GOAL POSES WRT LOCAL NED FRAME");  // UPDATE USER
	goal_poses_ned_msg.poses.clear();
	goal_poses_ned_msg.quantity = goal_poses_quantity;  // publish quantity of poses so the high level control knows
	for (int i = 0; i < goal_poses_quantity; i++)
	{
		// fill pose, then push_back to build array
		geometry_msgs::Point pose;
		pose.x = NED_x_goal[i];
		pose.y = NED_y_goal[i];
		//pose.position.z = 0.0;
		pose.z = NED_psi_goal[i];
		goal_poses_ned_msg.poses.push_back(pose);
		// UPDATE USER
		if (i < (goal_poses_quantity-1))
		{
			ROS_INFO("COORDINATE_CONVERTER: pose: %i    x: %.2f    y: %.2f    psi: %.2f", i, pose.x, pose.y, pose.z);
		}
		else  // if its the last pose print with the next line (\n) to make the monitor display nice
		{
			ROS_INFO("COORDINATE_CONVERTER: pose: %i    x: %.2f    y: %.2f    psi: %.2f\n", i, pose.x, pose.y, pose.z);
		}
	}
	CC_goal_poses_ned_pub.publish(goal_poses_ned_msg);  // publish the converted to local NED convention poses
}  // END OF CC_goal_poses_ned_publish()

// THIS FUNCTION: Publishes VRX4 animal poses in local NED convention w/ IDs to "CC_animals_ned"
// ACCEPTS: (VOID)
// RETURNS: (VOID)
//=============================================================================================================
void CC_animals_ned_publish()
{
	ROS_INFO("COORDINATE_CONVERTER: PUBLISHING ARRAY OF ANIMALS WRT LOCAL NED FRAME");  // UPDATE USER
	CC_animals_ned_msg.objects.clear();
	CC_animals_ned_msg.quantity = goal_poses_quantity;  // publish quantity of poses so the path planner knows
	for (int i=0; i<goal_poses_quantity; i++)
	{
		// fill animal, then push_back to build array
		geometry_msgs::PointStamped animal;  // publisher message type
		animal.header.seq +=1;  // sequence number
		animal.header.stamp = current_time;  // sets stamp to current time
		animal.header.frame_id = Animal[i].c_str();  // header frame
		animal.point.x = NED_x_goal[i];
		animal.point.y = NED_y_goal[i];
		CC_animals_ned_msg.objects.push_back(animal);
		// UPDATE USER
		// if (i < (goal_poses_quantity-1))
		// {
			// ROS_INFO("COORDINATE_CONVERTER: Animal: %s    x: %4.2f    y: %4.2f", CC_animals_ned_msg.objects[i].header.frame_id.c_str(), CC_animals_ned_msg.objects[i].point.x, CC_animals_ned_msg.objects[i].point.y);
		// }
		// else  // if its the last pose print with the next line (\n) to make the monitor display nice
		// {
			// ROS_INFO("COORDINATE_CONVERTER: Animal: %s    x: %4.2f    y: %4.2f\n", CC_animals_ned_msg.objects[i].header.frame_id.c_str(), CC_animals_ned_msg.objects[i].point.x, CC_animals_ned_msg.objects[i].point.y);
		// }
	}
	CC_animals_ned_pub.publish(CC_animals_ned_msg);  // publish left and right buoy locations, respectively, in array "NED_buoys"
}  // END OF CC_animals_ned_publish()
//.............................................................................................................END OF Functions...............................................................................................................

//..................................................................................................................Main Program..................................................................................................................
int main(int argc, char **argv)
{
	// names the program for visual purposes
	ros::init(argc, argv, "coordinate_converter");

	// NodeHandles
	ros::NodeHandle nh1, nh2, nh3, nh4, nh5, nh6;

	// Subscribers
	// from mission_control
	ros::Subscriber MC_cc_state_sub = nh1.subscribe("MC_cc_state", 1, MC_cc_state_update);  // updates coordinate_converter state published by mission_control
	ros::Subscriber MC_pp_state_sub = nh2.subscribe("MC_pp_state", 1, MC_pp_state_update);  // updates path_planner state published by mission_control
	// from geonav_transform package
	ros::Subscriber geonav_odom_sub = nh3.subscribe("geonav_odom", 100, NED_Func);  // HOPEFULLY I GET RID OFF THIS PACKAGE AND WE DO THE TRANSFORMATION OURSELVES
	// from VRX Tasks
	ros::Subscriber VRX_T1_goal_sub; // = nh4.subscribe("/vrx/station_keeping/goal", 1, VRX_T1_goal_update);  // subscriber for goal pose given by VRX TASK 1
	ros::Subscriber VRX_T2_goal_sub; // = nh4.subscribe("/vrx/wayfinding/waypoints", 100, VRX_T2_goal_update);  // subscriber for goal waypoints (poses) given by VRX TASK 2
	ros::Subscriber VRX_T4_goal_sub;  // = nh4.subscribre("vrx/wildlife/animals/poses",100, VRX_T4_goal_update);  // subscriber for VRX TASK 4
	// from path_planner
	ros::Subscriber PP_USV_pose_update_state_sub = nh5.subscribe("PP_USV_pose_update_state", 1, PP_USV_pose_update_state_update);  // subscriber for path_planner usv pose update status

	// Publishers
	// to mission_control
	CC_initialization_state_pub = nh6.advertise<std_msgs::Bool>("CC_initialization_state", 1);  // publisher for state of initialization
	CC_goal_poses_publish_state_pub = nh6.advertise<std_msgs::Bool>("CC_goal_poses_publish_state", 1);  // publisher for whether goal poses have been converted to NED and have been published
	nav_odom_pub = nh6.advertise<nav_msgs::Odometry>("nav_odom", 1);  // USV state publisher, this sends the current state to be converted to "nav_odom", so geonav_transform package can publish the ENU conversion to "geonav_odom"
	CC_goal_poses_ned_pub = nh6.advertise<amore::NED_poses>("CC_goal_poses_ned", 1);  // goal poses for VRX1 or VRX2 converted to NED publisher
	CC_animals_ned_pub = nh6.advertise<amore::NED_objects>("CC_animals_ned", 1);  // current animal IDs with respective locations for planner to use to generate path for VRX4

	// Initialize global variables
	CC_goal_poses_publish_state_msg.data = false;  // false means goal poses have not been converted to NED and published
	CC_initialization_state_msg.data = false;
	current_time = ros::Time::now();  // sets current time to the time it is now
	last_time = current_time;  // sets last time to the time it is now

	// sets the frequency for which the program loops at 10 = 1/10 second
	ros::Rate loop_rate(10);  // {Hz} geonav_transform rate: 10

	while(ros::ok())
	{
		COORDINATE_CONVERTER_inspector();  // check, update, and publish CC_initialization_state_msg
		
		//  CC_state LEGEND
		//	0 = On standby
		//	1 = VRX1: Station-Keeping NED goal pose converter
		//	2 = VRX2: Wayfinding NED goal pose converter
		//	4 = VRX4: Wildlife NED animals converter
		switch(CC_state)
		{
			case 0:  // On standby
				goal_poses_converted_NED = false;  // reset for next times task conversion
				// if path_planner is in VRX4: Wildlife Encounter and Avoid and the usv pose has been updated in the path_planner, but
				// the goal_poses_published_state has not been reset
				if ((PP_state == 4) && (usv_NED_pose_updated) && (CC_goal_poses_publish_state_msg.data))
				{
					// reset for next times task conversion
					CC_goal_poses_publish_state_msg.data = false;  // reset goal_poses_published_state
					//ROS_INFO("COORDINATE_CONVERTER: CC_goal_poses_publish_state_msg SHOULD BE RESET");
				}
				break;

			case 1:  // VRX1: Station-Keeping NED goal pose converter
				VRX_T1_goal_sub = nh4.subscribe("/vrx/station_keeping/goal", 1, VRX_T1_goal_update);  // subscriber for goal pose given by VRX TASK 1
				break;

			case 2:  // VRX2: Wayfinding NED goal pose converter
				VRX_T2_goal_sub = nh4.subscribe("/vrx/wayfinding/waypoints", 1, VRX_T2_goal_update);  // subscriber for goal poses given by VRX TASK 2
				break;

			case 4:  // VRX4: Wildlife NED animals converter
				VRX_T4_goal_sub = nh4.subscribe("/vrx/wildlife/animals/poses", 1, VRX_T4_goal_update);  // subscriber for goal poses given by VRX TASK 4
				break;

			default:
				break;
		}
		
		if ((CC_state == 1) || (CC_state == 2) || (CC_state == 4))
		{
			if (!goal_poses_converted_NED)  // if all the current goal poses have not been converted to the local NED frame
			{
				// while(!lat_lon_goal_recieved)  // while goal poses in lat and long coordinates have not been recieved
				// {
					// ros::spinOnce();
					// loop_rate.sleep();  // sleep to accomplish set loop_rate
				// }
				if (!lat_lon_goal_recieved)  // if goal poses in lat and long coordinates have not been recieved
				{
					ros::spinOnce();  // cycle the task topic subscriber to get the goal poses in geodetic coordinates
				}
				else if (lat_lon_goal_recieved)  // if goal poses in lat and long coordinates have been recieved
				{
					if (!lat_lon_pose_sent)  // if the current point to convert has not been sent
					{
						goal_convert_update();  // update nav_odom_msg variables to goal poses
						nav_odom_publish();  // fills nav_odom_msg and publishes to "nav_odom"
						lat_lon_pose_sent = true;
					}
					else if ((lat_lon_pose_sent) && (pose_converted_NED))  // if the current point to convert has been sent to be converted, AND its ENU conversion has been subscribed to and converted to NED
					{
						// Update goal pose array in NED units
						if (point_num < goal_poses_quantity)  // if not all the poses have been converted yet
						{
							NED_x_goal[point_num] = xNED;
							NED_y_goal[point_num] = yNED;
							NED_psi_goal[point_num] = psiNED;
							
							/* loops_published += 1;
							if (loops_published == CONFIRM)
							{
								loops_published = 0;  // reset loops_published counter
								point_num += 1;  // used to keep track of the point being converted
								lat_lon_pose_sent = false;
							} */
							point_num += 1;  // used to keep track of the point being converted
						}
						if (point_num == goal_poses_quantity)  // if all coordinates have been converted
						{
							point_num = 0;  // reset point_num to begin at 0
							//lat_lon_goal_recieved = false;  // reset for next times task conversion
							goal_poses_converted_NED = true;  // true means goal poses have been converted from lat/long to NED
							//ROS_INFO("COORDINATE_CONVERTER: GOAL POSES HAVE BEEN CONVERTED, PUBLISHING NEXT");
						}
						// reset for next pose's conversion
						pose_converted_NED = false;
						lat_lon_pose_sent = false;
					}  // END OF else if ((lat_lon_pose_sent) && (pose_converted_NED))
				}  // else if (lat_lon_goal_recieved)
			}  //  if (!goal_poses_converted_NED)
			else if ((goal_poses_converted_NED) && (!CC_goal_poses_publish_state_msg.data))  // if the goal poses have been converted to NED convention and have not been published
			{
				if ((CC_state == 1) || (CC_state == 2))
				{
					CC_goal_poses_ned_publish();  // CC_goal_poses_ned_pub.publish(goal_poses_ned_msg);
				}
				else if (CC_state == 4)
				{
					CC_animals_ned_publish();  // CC_animals_ned_pub.publish(CC_animals_ned_msg);
				}
				CC_goal_poses_publish_state_msg.data = true;
			}  // else if ((goal_poses_converted_NED) && (!CC_goal_poses_publish_state_msg.data))
		}  // if ((CC_state == 1) || (CC_state == 2) || (CC_state == 4))
		
		//ros::spinOnce();  // update subscribers
		loop_rate.sleep();  // sleep to accomplish set loop_rate
	}  // END OF while(ros::ok())
	
	return 0;
}  // END OF main()
//.........................................................................................................END OF Main Program...........................................................................................................