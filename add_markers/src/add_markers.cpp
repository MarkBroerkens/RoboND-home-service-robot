/*
 * MIT License
 * 
 * Copyright (c) 2010, Willow Garage, Inc.
 * Copyright (c) 2021, Mark Brörkens
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>

geometry_msgs::PoseStamped goal_;
bool showMarker_ = false;

  
  ros::Publisher marker_pub;
  ros::Subscriber goal_sub;
  ros::Subscriber showMarker_sub;

void updateMarker();

void showMarkerCallback(const std_msgs::Bool& showMarker)
{
  ROS_INFO("show marker: %s ", showMarker.data ? "true" : "false");
  showMarker_ =  showMarker.data;
  updateMarker();
}


void goalCallback(const geometry_msgs::PoseStamped& goal)
{
  ROS_INFO("current goal (x,y): %f, %f", goal.pose.position.x, goal.pose.position.y);
  goal_ =  goal;
  updateMarker();
}

void updateMarker()
{
    if (marker_pub.getNumSubscribers() < 1)
    {
      ROS_WARN_ONCE("Please create a subscriber to the marker");
    }
    else 
    {
          visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "add_markers";
    marker.id = 0;

   
    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    if (showMarker_)
    {
      // Set the marker type.
      marker.type = visualization_msgs::Marker::SPHERE;;

      marker.action = visualization_msgs::Marker::ADD;
            // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
      marker.pose = goal_.pose;

      // Set the scale of the marker -- 1x1x1 here means 1m on a side
      marker.scale.x = 1.0;
      marker.scale.y = 1.0;
      marker.scale.z = 1.0;

      // Set the color -- be sure to set alpha to something non-zero!
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0;

      marker.lifetime = ros::Duration();
    } 
    else 
    {
      marker.action = visualization_msgs::Marker::DELETE;
    }
    


    marker_pub.publish(marker);
    
  }
}


int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  goal_sub = n.subscribe("/move_base/current_goal", 1000, goalCallback);
  showMarker_sub = n.subscribe("/showMarker", 1000, showMarkerCallback);

  ros::spin();  
}
