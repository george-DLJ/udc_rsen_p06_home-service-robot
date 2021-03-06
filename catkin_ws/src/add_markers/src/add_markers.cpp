/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* 2020.08.17 I am using this code following Part 06 project "Home Service Robot" instructions on UDACITY Robot Software Engineering nanodegree as basis to the exercise. */

/* 2020.08.21 UPDATE:
   After review I've followed the rubric recomendation to use amcl pose instead of odom to get
   more accuracy between robot pose and markers position.   
   * #include <nav_msgs/Odometry.h> is not necessary any more and has been 
   replaced by #include <nav_msgs/Odometry.h> 
   
   * on robot_location_callback() the message type has been also replaced with amcl_pose 
     messages (geometry_msgs::PoseWithCovarianceStamped).
	
   * it was observed that amcl pose messages stop comming when robot stops moving. 
     therefore the logic has been slightly modified.	    
*/

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
//#include <nav_msgs/Odometry.h> 
#include <geometry_msgs/PoseWithCovarianceStamped.h> 

// Define a tolerance threshold to compare double values
const double positionTolerance = 0.1;//meters
const double moveTolerance = 0.0001;//meters
// Predefined pick up and drop off zones:
const std::vector<double> robot_pickup_position{ -2.2, 2.4, 0.0 }; //x, y, z
const std::vector<double> robot_dropoff_position{ -0.3, -0.8, 0.0 }; //x, y, z

// Define global vector of last position:
std::vector<double> robot_last_position{ 0.0, 0.0, 0.0 };
// state values
bool markerPickedUp = false;
bool markerDroppedOff = false;
bool moving_state = false;


//global variable marker
visualization_msgs::Marker marker;
ros::Publisher marker_pub;

void initMarker()
{
// Set our initial shape type to be a cube
    uint32_t shape = visualization_msgs::Marker::CUBE; 
  
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "add_markers";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;


    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

// 1. Publish the marker at PICKUP ZONE:
   ROS_INFO("Publishing Marker on PICK UP zone:");    
   // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.4;
    marker.scale.y = 0.4;
    marker.scale.z = 0.4;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration(); //5 seconds
}

void setMarkerPositionXY(double posx, double posy)
{
	marker.pose.position.x = posx;
    	marker.pose.position.y = posy;
	marker.action = visualization_msgs::Marker::ADD;
}

void hideMarker()
{	ros::Rate r(1);
 	marker.action = visualization_msgs::Marker::DELETE;
	marker_pub.publish(marker);
}

void showMarker()
{
	ros::Rate r(1);
	marker_pub.publish(marker);
    	r.sleep();
}

void pickUpMarker()
{
    // simulate pickup
    ROS_INFO("Picking Marker (5 seconds)");
    ros::Duration(5.0).sleep(); 
    
    // Hiding marker:
    ROS_INFO("Hiding Marker");
    hideMarker();
}

void dropOffMarker()
{
    // simulate drop off
    ROS_INFO("Droping Off Marker (5 seconds)");
    ros::Duration(5.0).sleep(); 	
    // Change marker  location:
    setMarkerPositionXY(robot_dropoff_position[0], robot_dropoff_position[1]);	
    // showMarker on dropOffZone
    showMarker();
}

bool robotOnPickupPosition( std::vector<double> robot_current_position)
{
	if (fabs(robot_current_position[0] - robot_pickup_position[0]) < positionTolerance && fabs(robot_current_position[1] - robot_pickup_position[1]) < positionTolerance)
	{
        markerPickedUp = true;
	ROS_INFO("MARKER PICKED UP (Position(x,y,z): [%f], [%f], [%f]", robot_current_position[0],robot_current_position[1], robot_current_position[2]);
		return true;
	}
	return false;
}

bool robotOnDropOffPosition( std::vector<double> robot_current_position)
{
	if (markerPickedUp && fabs(robot_current_position[0] - robot_dropoff_position[0]) < positionTolerance && fabs(robot_current_position[1] - robot_dropoff_position[1]) < positionTolerance)
	{
        	markerDroppedOff = true;
		ROS_INFO("MARKER Dropped Off (Position(x,y,z): [%f], [%f], [%f]", robot_current_position[0],robot_current_position[1], robot_current_position[2]);
		return true;
	}
	return false;
}

void robot_location_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    std::vector<double> robot_current_position = {msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z};
  // DEBUG:
  // ROS_INFO("robot_current_position-> x: [%f], y: [%f], z: [%f]", robot_current_position[0],robot_current_position[1], robot_current_position[2]); 

	if (!markerPickedUp && robotOnPickupPosition(robot_current_position))
	{
	   pickUpMarker();
	} 
	else if (markerPickedUp && !markerDroppedOff && robotOnDropOffPosition(robot_current_position))
	{
	   dropOffMarker();
	}
	else
	{
	   if(robot_last_position != robot_current_position)
	   {
		robot_last_position = robot_current_position;
		ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", robot_current_position[0], robot_current_position[1], robot_current_position[2] );
	   }
	}
}

int main( int argc, char** argv )
{
  ROS_INFO("START add_markers.cpp");
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
 
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // Subscribe to odometry values
  ROS_INFO("Subscribing to odom:");
  //ros::Subscriber odomSubscriber = n.subscribe("odom", 10, robot_location_callback);

  // PROBLEM: pick_objects (/map) and add_markers (/odom) use different references that change.
  // Solution 1: subscribe to amcl instead of odom
  ROS_INFO("Subscribing to amcl (amcl_pose)");
  ros::Subscriber sub_amcl = n.subscribe("amcl_pose", 10, robot_location_callback);

   // Create/init marker
   initMarker();
   setMarkerPositionXY(robot_pickup_position[0],robot_pickup_position[1]);

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    marker_pub.publish(marker);
    r.sleep();

   // Read pose values until robot reaches pickup zone.
   ROS_INFO("Read Pose messages ros::spin()");  
   ros::spin();
}
