// Filename:								stereo_vis.cpp
// Creation Date:						12/04/2021
// Last Revision Date:
// Author(s) [email]:					Shaede Perzanowski [sperzanowski1@lssu.edu]
//												Brad Hacker [bhacker@lssu.edu]
// Revisor(s) [Revision Date]:
// Organization/Institution:			Lake Superior State University

//...................................................About stereo_vis.cpp.....................................................................
// The stereo_vis.cpp file is used to access image data from the on-board camera sensors of the USV. 
// The program uses OpenCV to detect a red and a green buoy, and then it uses perspective geometry
// to find the location of each buoy with respect to the USV.

// Inputs and Outputs of the stereo_vis.cpp file
//				Inputs: Camera sensor data from forward facing cameras.
//				Outputs: Buoy locations (published data) and classifications (visual display)


//................................................Included Libraries and Message Types..........................................
#include "ros/ros.h"
#include "time.h"
#include "image_transport/image_transport.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "iostream"
#include "stdio.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include <vector>
#include <string>
#include "std_msgs/Float32.h"
#include "geometry_msgs/PoseArray.h"
//...........................................End of Included Libraries and Message Types....................................

//..............................................................Namespaces................................................................
using namespace cv;
using namespace std;
//........................................................End of Namespaces...........................................................

//.................................................................Constants..................................................................
#define BASELINE 0.2				// baseline between camera sensors [m]
#define Umax 1279.0					// maximum pixel index for u (a pictures's x)
#define Umin 0.0						// minimum pixel index for u (a pictures's x)
//............................................................End of Constants.............................................................

//..............................................................Global Variables............................................................
int loop_count = 0;                                    					// loop counter, first 10 loops used to intitialize subscribers

int hue_low = 0;				//__ For HSV sliders, used to "tune" the masks
int hue_high = 179;		//   |
int sat_low = 0;				//   |
int sat_high = 255;			//   |
int value_low = 0;			//   |
int value_high = 255; 		//__|
// Create color scalars for the masks
cv::Scalar green_low = cv::Scalar(59, 180, 25);  //70 //59 now includes the totem //30
cv::Scalar green_high = cv::Scalar(100, 255, 255); 
cv::Scalar blue_water_low = cv::Scalar(92, 0, 0);   
cv::Scalar blue_water_high = cv::Scalar(148, 255, 255);
cv::Scalar red_low = cv::Scalar(130, 200, 60);  //bitwise add //200 //60
cv::Scalar red_high = cv::Scalar(179, 255, 255); //sometimes detects red mutliple times
cv::Scalar red_low1 = cv::Scalar(0, 80, 82);  //bitwise add
cv::Scalar red_high1 = cv::Scalar(1, 255, 255); //sometimes detects red mutliple times
// Two masks, used to show the effects of the color masks and BLOB analysis
cv::Mat mask;
cv::Mat mask_new;
cv::Mat red_mask; //____mask images for each colour
cv::Mat red_mask1; // 	|	
cv::Mat green_mask; //	|
cv::Mat red_res; //			|
cv::Mat green_res; //___|
// three vectors to hold BLOB data
std::vector <std::vector <cv::Point>> contours;
std::vector <std::vector <cv::Point>> contours1;
std::vector <std::vector <cv::Point>> contours2;
// Creat three images to hold the raw data from the sensors
cv::Mat org_img;
cv::Mat org_img1;
cv::Mat org_img2;
float height; //_ height and width of image 
float width;  //_|
float area; //area of contour for a given BLOB
string color;
// Two vectors to hold the centroids of the BLOBs
vector<Point2f> MC(5); // For left image
vector<Point2f> MC1(5); // For right image
int left_blob_cnt = 0; // Holds the count of BLOBs from left camera
int right_blob_cnt = 0; // Holds the count of BLOBs from left camera

// Look-up-table for finding accurate lateral translation of buoy from the USV
float lat_scale_LUT[10][9] = {												  // 7m	8m	9m	10m	11m	12m	13m	14m	15m
	{1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1, 1.000, 1.000}, // 0m
	{1.838, 1.610, 1.439, 1.290, 1.176, 1.075, 1, 0.930, 0.735}, // 1m		columns are forward longitudinal translation
	{0.947, 0.826, 0.727, 0.658, 0.597, 0.543, 1, 0.468, 0.431}, // 2m
	{0.624, 0.544, 0.489, 0.438, 0.397, 0.365, 1, 0.313, 0.287}, // 3m		rows are lateral translation, either direction
	{0.472, 0.410, 0.363, 0.325, 0.295, 0.273, 1, 0.233, 0.218}, // 4m
	{1.000, 0.331, 0.291, 0.261, 0.236, 0.214, 1, 0.186, 0.175}, // 5m
	{1.000, 1.000, 0.245, 0.218, 0.196, 0.181, 1, 0.153, 0.143}, // 6m
	{1.000, 1.000, 1.000, 0.188, 0.171, 0.156, 1, 0.131, 0.122}, // 7m
	{1.000, 1.000, 1.000, 1.000, 0.147, 0.135, 1, 0.114, 0.106}, // 8m
	{1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1, 0.101, 0.097}  // 9m
};
string Rstring = "_`_`_`_`_ Right Buoy _`_`_`_`_  {V}"; // Right Buoy ID
string Lstring = "_`_`_`_`_ Left Buoy _`_`_`_`_  {V}"; // Left Buoy ID
string buoy_ID;
float LX, LY, RX, RY; // Variables for the coordinates of the buoys w.r.t USV
std_msgs::Float32 lx, ly, rx, ry; // message type for the coordinates of the buoys w.r.t USV

geometry_msgs::PoseArray left_right_buoy_array_msg;		// message used to hold and publish buoy locations
ros::Publisher left_right_buoy_array_pub;								// "left_right_buoy_array" publisher

ros::Time current_time, last_time;											// creates time variables

//........................................................End of Global Variables........................................................

//..................................................................Functions.................................................................

// THIS FUNCTION: Finds the best focal length of the camera sensors, based off a LUT
// ACCEPTS: the estimated longitudinal translation to the buoy (zz)
// RETURNS: the corrected focal length (focal_length)
// =============================================================================
float fpLUT(float zz)
{
	float focal_length;
	zz = round(zz - 6.0);				// use longitudinal distance to find entry from LUT
	switch(int(zz))
	{
		case 0:								// <7m
			focal_length = 865.0;
			break;
		case 1:								// 7m
			focal_length = 855.0;
			break;
		case 2:								// 8m
			focal_length = 846.0;
			break;
		case 3:								// 9m
			focal_length = 839.2;
			break;
		case 4:								// 10m
			focal_length = 837.0;
			break;
		case 5:								// 11m
			focal_length = 829.0;
			break;
		case 6:								// 12m
			focal_length = 826.1;
			break;
		case 7:								// 13m
			focal_length = 823.0;
			break;
		case 8:								// 14m
			focal_length = 821.0;
			break;
		case 9:								// 15m
			focal_length = 815.9;
			break;
		default:
			focal_length = 845.0;
	}
	return focal_length;
} // end of fpLUT()

// THIS FUNCTION: Determines the initial lateral gain to multiply the first lateral disparity calculation by, based off a LUT
// ACCEPTS: the approximated lateral translation to the buoy (latzz)
// RETURNS: the lateral gain (gainer)
// =============================================================================
float lateral_gainLUT(float latzz)
{
	float gainer;
	latzz = abs(round(latzz));
	// This LUT is referenced to a longitudinal translation of 13m
	switch(int(latzz))
	{
		case 0:
			gainer = 62.5;
			break;
		case 1:
			gainer = 62.5;
			break;
		case 2:
			gainer = 62.5;
			break;
		case 3:
			gainer = 62.5;
			break;
		case 4:
			gainer = 62.5;
			break;
		case 5:
			gainer = 62.5;
			break;
		case 6:
			gainer = 61.86;
			break;
		case 7:
			gainer = 62.5;
			break;
		case 8:
			gainer = 63.0;
			break;
		case 9:
			gainer = 63.38;
			break;
		default:
			gainer = 62.5;
	}
	return gainer;
} // end of lateral_gainLUT()

// THIS FUNCTION: Determines the final lateral gain to multiply the lateral disparity calculation by, based off a LUT (lat_scale_LUT)
// ACCEPTS: the approximated lateral translation (lat_z) and longitudinal (long_z) translation to the buoy
// RETURNS: final lateral gain (lats)
// =============================================================================
float lateral_scale(float lat_z, float long_z)
{
	float lats = 1.0, lat_z_prev, error_s;
	int r, c;
	c = int(round(long_z) - 7.0);							// Use the longitudinal translation to determine the column of the LUT to look at
	if(c<0)														//_ Do not let c break the LUT columns
	{																//	|
		c = 0;													//	|
	}																//	|
	if(c>8)														//	|
	{																//	|
		c = 8;													//	|
	}																//_|
	lat_z_prev = lat_z;
	lat_z = 1.0/lat_z;
	if(c==6)
	{
		return lats;
	}
	for(r=1;r<10;r++)
	{
		if(lat_z>=lat_scale_LUT[r][c])
		{
			lats = float(r)*lat_scale_LUT[r][c];
			break;
		}
	}
	return lats;
} // end of lateral_scale()

// THIS FUNCTION: 
// ACCEPTS: centroid coordinates
// RETURNS: nothing
// =============================================================================
void DisparityFunc()
{
	float d[left_blob_cnt];						// The disparity between camera images [pixels]
	float f_p = 820.0;							// experimental focal length [pixels]
	float z[left_blob_cnt];						// Distance along USV surge to the projection of the point [m]
	float z_0;										// Used to obtain more accurate estimate of z [m]
	float s_left[left_blob_cnt], s_right[left_blob_cnt];	// Distances from centrod to each image's interior vertical [pixels]
	float ds_left[left_blob_cnt];				// The "lateral disparity" for each camera image [pixels]
	float ds_right[left_blob_cnt];			// The "lateral disparity" for each camera image [pixels]
	float lat_z_left[left_blob_cnt];			// Distance along USV sway to the projection of the point [m]
	float lat_z_right[left_blob_cnt];		// Distance along USV sway to the projection of the point [m]
	float lat_z_avg[left_blob_cnt];
	float g_left = 62.5;
	float g_right = 62.5;
	float k_lat = -1.0;
	float ds_lat = -1.0;
	for(int i=0; i<left_blob_cnt; i++)
	{
		d[i] = MC[i].x - MC1[i].x;
		z_0 = (f_p*BASELINE)/d[i];
		f_p = fpLUT(z_0);
		z[i] = (f_p*BASELINE)/d[i];
		
		s_left[i] = MC[i].x - (Umax/2.0);
		s_right[i] = MC1[i].x - (Umax/2.0);
		ds_left[i] = (z[i]*s_left[i])/f_p;
		ds_right[i] = (z[i]*s_right[i])/f_p;
		ds_lat = (ds_left[i]+ds_right[i])/2.0;
		lat_z_left[i] = g_left*((z[i]*ds_left[i])/f_p - 0.001);
		lat_z_right[i] = g_right*((z[i]*ds_right[i])/f_p + 0.002);
		g_left = lateral_gainLUT(lat_z_left[i]);
		g_right = lateral_gainLUT(lat_z_right[i]);
		lat_z_left[i] = g_left*((z[i]*ds_left[i])/f_p - 0.001);
		lat_z_right[i] = g_right*((z[i]*ds_right[i])/f_p + 0.002);
		lat_z_avg[i] = (lat_z_left[i] + lat_z_right[i])/2.0;
		k_lat = lateral_scale(lat_z_avg[i], z[i]);
		lat_z_avg[i] = lat_z_avg[i]*k_lat;
		
		if(isfinite(lat_z_avg[i])&&isfinite(z[i]))
		{
			if(i==0)
			{
				buoy_ID = Rstring;
				RX = z[i];
				RY = lat_z_avg[i];
			}
			else
			{
				buoy_ID = Lstring;
				LX = z[i];
				LY = lat_z_avg[i];
			}
			//ROS_INFO(buoy_ID);
			ROS_INFO("________vvvvvvvvvvv  {V}  vvvvvvvvvvv___________________");
			ROS_INFO("                 Left Camera                            |");
			ROS_INFO("               u: %.0f [pixels]|", MC[i].x);
			ROS_INFO("               v: %.0f [pixels]", MC[i].y);
			ROS_INFO(".................Right Camera...........................|");
			ROS_INFO("               u: %.0f [pixels]", MC1[i].x);
			ROS_INFO("               v: %.0f [pixels]", MC1[i].y);
			ROS_INFO("            Disparity: %.0f [pixels]", d[i]);
			ROS_INFO("         Lateral Disparity:  %.0f [pixels]", ds_lat);
			ROS_INFO("Longitudinal Translation from USV (x): %.3f [m]", z[i]);
			ROS_INFO("  Lateral Translation from USV (y): %.3f [m]", lat_z_avg[i]);
			ROS_INFO("________^^^^^^^^^^^  {V}  ^^^^^^^^^^^___________________|\n");
			/* printf("u: %.0f [pixels]   |||   %.0f [pixels]\n", MC[i].x, MC1[i].x);
			printf("v: %.0f [pixels]   |||   %.0f [pixels]\n", MC[i].y, MC1[i].y);
			printf("Disparity: %.0f [pixels]\n             Lateral Disparity:  %.0f [pixels]\n", d[i], d[i]);
			printf("Translation from USV (x,y): (%.3f [m] , %.3f [m])\n\n", z[i], lat_z_avg[i]); */
		}
		else
		{
			ROS_ERROR("Non-finite Disparity Calculation. {V}");
		}
	}
	left_blob_cnt = 0;
} // end of DisparityFunc()

// THIS FUNCTION: 
// ACCEPTS: 
// RETURNS: nothing
// =============================================================================
void cameraCallBack(const sensor_msgs::ImageConstPtr& camera_msg)
{
	try 
	{
		org_img = cv_bridge::toCvShare(camera_msg, "bgr8") -> image;		//converts message camera_msg into bgr8 type image
		//cv::imshow("Left Camera", org_img); //converts the ros image to bgr type
		Mat background;
		Mat grey_frame;
		Mat threshold;
		Mat f_con;
		Mat imgHSV;
		org_img.copyTo(background);
		cv::cvtColor(org_img, imgHSV, cv::COLOR_BGR2HSV);
		
		cv::inRange(imgHSV, red_low, red_high, red_mask); 
		cv::inRange(imgHSV, red_low1, red_high1, red_mask1); 
		red_mask = red_mask + red_mask1;
		
		cv::Mat mask_t;
		cv::inRange(imgHSV, red_low, red_high, red_mask); 
		cv::inRange(imgHSV, green_low, green_high, green_mask);  


		cv::medianBlur(red_mask, red_mask, 3);
		cv::medianBlur(green_mask, green_mask, 3);
		
		mask_t = green_mask + red_mask;
		
		cv::bitwise_and(org_img, org_img, mask=red_mask, red_res);
		cv::bitwise_and(org_img, org_img, mask=mask, green_res);
		
		// Finds contour around coloured BLOBS and draws a rectangle around the object
		cv::findContours(red_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
		for (size_t i=0; i < contours.size(); ++i)
		{
			cv::Rect boundRect = cv::boundingRect(contours[i]);
			cv::rectangle(background, boundRect.tl(), boundRect.br(), cv::Scalar(0, 0, 255), 3);
			cv::putText(background, "red",(boundRect.br(), boundRect.tl()), 2, 1, (255, 0, 0));
		}
		
 		cv::findContours(green_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
		for (size_t i=0; i < contours.size(); ++i)
		{
			cv::Rect boundRect = cv::boundingRect(contours[i]);
			cv::rectangle(background, boundRect.tl(), boundRect.br(), cv::Scalar(0, 255, 0), 3);
			cv::putText(background, "green",(boundRect.br(), boundRect.tl()), 2, 1, (255, 0, 0));
		}
		
		cv::SimpleBlobDetector::Params blob;
		
//		blob.minThreshold = 240;
//		blob.maxThreshold = 255;
		blob.filterByArea = true;
		blob.minArea = 20;
		blob.filterByCircularity = false;
//		blob.minCircularity = 0.1;
		blob.filterByConvexity = false;
//		blob.minConvexity = 0.87;
		blob.filterByInertia = false;
//		blob.minInertiaRatio = 0.01;
//		 cv::SimpleBlobDetector detector(blob);

		Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(blob);

		std::vector <cv::KeyPoint> keypoints;
		cv::bitwise_not(mask_t, mask_new); //black BLOB on white background
		detector->detect(mask_new, keypoints);

		if (keypoints.size() == 0) //nothing was found
		{
			ROS_ERROR("No Object Detected by Left Camera. {V}");
		}
		else 
		{			
			cv::findContours(mask_t, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
			
			// Find the moments of each contour 
			vector<Moments> mu(contours.size());
			for( int i = 0; i<contours.size(); i++ )
			{
				mu[i] = cv::moments(contours[i], false);
			}
			
			// Find the centroid of each BLOB based on the contour moments
			// ...important note: centroids change depending on scaling
			vector<Point2f> mc(contours.size());
			for( int i = 0; i<contours.size(); i++)
			{
				mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );
				MC[i] = mc[i];
				left_blob_cnt = left_blob_cnt + 1;
			}
			DisparityFunc();
			
			// Place a small circle centered at the centroid of the BLOB
			for( int i = 0; i<contours.size(); i++ )
			{
				Scalar color = Scalar(128,0,0);
				circle(mask_t, mc[i], 4, color, 1, 8, 0 );
			} 
		}
		
		// Find corners of each BLOB
		std::vector< cv::Point2f > corners;
		cv::Mat mask;
		int maxCorners = 10;
		double qualityLevel = 0.75;
		double minDistance = 10.0;
		int blockSize = 3;
		bool useHarrisDetector = false;
		double k = 0.04;
		corners.reserve(maxCorners);
 		cv::goodFeaturesToTrack(mask_t, corners, maxCorners, qualityLevel, minDistance, mask, blockSize, useHarrisDetector, k );
		// Place a small circle centered at every corner of each BLOB
		for( size_t i = 0; i < corners.size(); i++ )
		{
			cv::circle(mask_t, corners[i], 7, cv::Scalar(255, 255, 0), -1 );
		} 
		
		cv::imshow("Left Camera Updated", background);   
		//cv::imshow("Left Camera Features", mask_t); 
		cv::waitKey(30);
	}
	catch (cv_bridge::Exception& e) //looks for errors 
	{
		ROS_ERROR("Left Camera could not convert from '%s' to 'bgr8'. {V}", camera_msg -> encoding.c_str()); //prints out the encoding string
	}
	
} // end of cameraCallBack()

// THIS FUNCTION: 
// ACCEPTS: 
// RETURNS: nothing
// =============================================================================
void cameraCallBack1(const sensor_msgs::ImageConstPtr& camera_msg1) {

	try 
	{
		org_img1 = cv_bridge::toCvShare(camera_msg1, "bgr8") -> image;		//converts message camera_msg into bgr8 type image
		//cv::imshow("Right Camera", org_img1); //converts the ros image to bgr type
		Mat background1;
		Mat grey_frame1;
		Mat threshold1;
		Mat f_con1;
		Mat imgHSV1;
		org_img1.copyTo(background1);
		cv::cvtColor(org_img1, imgHSV1, cv::COLOR_BGR2HSV);
		
		cv::inRange(imgHSV1, red_low, red_high, red_mask); 
		cv::inRange(imgHSV1, red_low1, red_high1, red_mask1); 
		red_mask = red_mask + red_mask1;
		
		cv::Mat mask_t1;
		cv::inRange(imgHSV1, red_low, red_high, red_mask); 
		cv::inRange(imgHSV1, green_low, green_high, green_mask);  


		cv::medianBlur(red_mask, red_mask, 3);
		cv::medianBlur(green_mask, green_mask, 3);
		
		mask_t1 = green_mask + red_mask;
		
		cv::bitwise_and(org_img1, org_img1, mask=red_mask, red_res);
		cv::bitwise_and(org_img1, org_img1, mask=mask, green_res);
		
		// Finds contour around coloured BLOBS and draws a rectangle around the object
		cv::findContours(red_mask, contours1, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
		for (size_t i=0; i < contours1.size(); ++i)
		{
			cv::Rect boundRect = cv::boundingRect(contours1[i]);
			cv::rectangle(background1, boundRect.tl(), boundRect.br(), cv::Scalar(0, 0, 255), 3);
			cv::putText(background1, "red",(boundRect.br(), boundRect.tl()), 2, 1, (255, 0, 0));
		}
		
 		cv::findContours(green_mask, contours1, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
		for (size_t i=0; i < contours1.size(); ++i)
		{
			cv::Rect boundRect = cv::boundingRect(contours1[i]);
			cv::rectangle(background1, boundRect.tl(), boundRect.br(), cv::Scalar(0, 255, 0), 3);
			cv::putText(background1, "green",(boundRect.br(), boundRect.tl()), 2, 1, (255, 0, 0));
		}
		
		cv::SimpleBlobDetector::Params blob1;
		
//		blob1.minThreshold = 240;
//		blob1.maxThreshold = 255;
		blob1.filterByArea = true;
		blob1.minArea = 20;
		blob1.filterByCircularity = false;
//		blob1.minCircularity = 0.1;
		blob1.filterByConvexity = false;
//		blob1.minConvexity = 0.87;
		blob1.filterByInertia = false;
//		blob1.minInertiaRatio = 0.01;
//		 cv::SimpleBlobDetector detector(blob);

		Ptr<cv::SimpleBlobDetector> detector1 = cv::SimpleBlobDetector::create(blob1);

		std::vector <cv::KeyPoint> keypoints1;
		cv::bitwise_not(mask_t1, mask_new); //black BLOB on white background
		detector1->detect(mask_new, keypoints1);

		if (keypoints1.size() == 0) //nothing was found
		{
			ROS_ERROR("Nothing Detected by Right Camera. {V}");
		}
		else 
		{			
			cv::findContours(mask_t1, contours1, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
			
			// Find the moments of each contour 
			vector<Moments> mu1(contours1.size());
			for( int i = 0; i<contours1.size(); i++ )
			{
				mu1[i] = cv::moments(contours1[i], false);
			}
			
			// Find the centroid of each BLOB based on the contour moments
			// ...important note: centroids change depending on scaling
			vector<Point2f> mc1(contours1.size());
			for( int i = 0; i<contours1.size(); i++)
			{
				mc1[i] = Point2f( mu1[i].m10/mu1[i].m00 , mu1[i].m01/mu1[i].m00 );
				MC1[i] = mc1[i];
				right_blob_cnt = right_blob_cnt + 1;
			}
			// DisparityFunc() should not be used in this subroutine      {SSP, 12/04/2021}
			
			// Place a small circle centered at the centroid of the BLOB
			for( int i = 0; i<contours1.size(); i++ )
			{
				Scalar color = Scalar(128,0,0);
				circle(mask_t1, mc1[i], 4, color, 1, 8, 0 );
			} 
		}
		
		// Find corners of each BLOB
		std::vector< cv::Point2f > corners1;
		cv::Mat mask1;
		int maxCorners = 10;
		double qualityLevel = 0.75;
		double minDistance = 10.0;
		int blockSize = 3;
		bool useHarrisDetector = false;
		double k = 0.04;
		corners1.reserve(maxCorners);
 		cv::goodFeaturesToTrack(mask_t1, corners1, maxCorners, qualityLevel, minDistance, mask1, blockSize, useHarrisDetector, k );
		// Place a small circle centered at every corner of each BLOB
		for( size_t i = 0; i < corners1.size(); i++ )
		{
			cv::circle(mask_t1, corners1[i], 7, cv::Scalar(255, 255, 0), -1 );
		} 
		cv::imshow("Right Camera Updated", background1); 
		//cv::imshow("Right Camera Features", mask_t1); 
	}
	catch (cv_bridge::Exception& e) //looks for errors 
	{
		ROS_ERROR("Right Camera could not convert from '%s' to 'bgr8'. {V}", camera_msg1 -> encoding.c_str()); //prints out the encoding string
	}
	
} // end of cameraCallBack1()

// THIS FUNCTION: Fills out left_right_buoy_array_msg and publishes to "left_right_buoy_array" for mission_control
// ACCEPTS: Nothing. Uses global variable buoy locations
// RETURNS: (VOID)
// =============================================================================
void buoy_array_publish()
{
	geometry_msgs::Point L, R;
	
	if(LY < RY)
	{
		L.x = LX;
		L.y = LY;
		R.x = RX;
		R.y = RY;
	}
	else
	{
		L.x = RX;
		L.y = RY;
		R.x = LX;
		R.y = LY;
	}
	
	left_right_buoy_array_msg.poses.clear();
	left_right_buoy_array_msg.poses[0].push_back(L);
	left_right_buoy_array_msg.poses[1].push_back(R);

	left_right_buoy_array_pub.publish(left_right_buoy_array_msg);		// publish left and right buoy locations, respectively, in array "left_right_buoy_array"
} // END OF buoy_array_publish()
//............................................................End of Functions............................................................


//...............................................................Main Program..............................................................
int main(int argc, char **argv)
{
	ros::init(argc, argv, "stereo_vis");
	
	// Initializations
	/* cv::namedWindow("Left Camera"); //creates new windows for each camera
	cv::namedWindow("Right Camera"); */
	cv::namedWindow("Left Camera Updated", cv::WINDOW_AUTOSIZE);
	//cv::namedWindow("Left Camera Features", cv::WINDOW_AUTOSIZE);
	cv::namedWindow("Right Camera Updated", cv::WINDOW_AUTOSIZE);
	//cv::namedWindow("Right Camera Features", cv::WINDOW_AUTOSIZE);
  
	// NodeHandles
	ros::NodeHandle nh1, nh2, nh3;
	
	// Subscribers
	image_transport::ImageTransport it1(nh1); //transports the images from the subscriber
	image_transport::ImageTransport it2(nh2);
	image_transport::Subscriber camera_sub = it1.subscribe("/wamv/sensors/cameras/front_left_camera/image_raw", 1, cameraCallBack); //front left camera
	image_transport::Subscriber camera_sub1 = it2.subscribe("/wamv/sensors/cameras/front_right_camera/image_raw", 1, cameraCallBack1); //front rightcamera
	
	// Publishers
	left_right_buoy_array_pub = nh3.advertise<geometry_msgs::PoseArray>("left_right_buoy_array", 1);						// current left and right buoy locations for planner to use to generate path
	
	// Initialize simulation time
	ros::Time::init();
	
	// Initialize global variables
	current_time = ros::Time::now();						// sets current time to the time it is now
	last_time = current_time;									// sets last time to the time it is now
  
	// Set the loop sleep rate
	ros::Rate loop_rate(0.5);

	ros::spinOnce();
	/* cv::destroyWindow("Left Camera"); //destroys the new windows
	cv::destroyWindow("Right Camera"); */
	cv::destroyWindow("Left Camera Updated");
	//cv::destroyWindow("Left Camera Features");
	cv::destroyWindow("Right Camera Updated");
	//cv::destroyWindow("Right Camera Features");
	
	while(ros::ok())
	{
		current_time = ros::Time::now();						// sets current time to the time it is now
		
		if(loop_count>=10)
		{
			buoy_array_publish();									// fill and publish left_right_buoy_array message 
		}

		ros::spinOnce();
		loop_rate.sleep();
		last_time = current_time;
		loop_count += 1;
	}
	
	return 0;
} // end of main()
//............................................................End of Main Program...........................................................