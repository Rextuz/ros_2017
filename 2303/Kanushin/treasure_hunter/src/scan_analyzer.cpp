#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>


class LaserScanToPointCloud{

public:
  int oneMarker;
  ros::Publisher pubMarker;
  ros::Subscriber subOdom;
  nav_msgs::Odometry odmsg;
  visualization_msgs::Marker marker;
  int count;
  ros::NodeHandle n_;
  laser_geometry::LaserProjection projector_;
  tf::TransformListener listener_;
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
  tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;
  ros::Publisher scan_pub_;

  LaserScanToPointCloud(ros::NodeHandle n) : 
    n_(n),
    laser_sub_(n_, "base_scan", 10),
    laser_notifier_(laser_sub_,listener_, "base_laser_link", 10)
  {
    laser_notifier_.registerCallback(
      boost::bind(&LaserScanToPointCloud::scanCallback, this, _1));
    laser_notifier_.setTolerance(ros::Duration(0.01));
    scan_pub_ = n_.advertise<sensor_msgs::PointCloud>("/my_cloud",1);
    subOdom = n.subscribe("odom", 100,&LaserScanToPointCloud::callback, this);
    pubMarker = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
   count = 0;
   oneMarker = 0;
  }

void callback(const nav_msgs::Odometry::ConstPtr& msg)
{
   odmsg = *msg;
   std::cout<<"POSE X - "<<odmsg.pose.pose.position.x<<std::endl;
}
void addMarker(float x, float y)
{
        marker.header.frame_id = "base_link";
        marker.header.stamp = ros::Time::now();//ros::Time::now();
        
        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        marker.ns = "markersTen";
        marker.id = count;
        count++;
        
        // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
        // marker.type = shape;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        
        // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
      
        marker.pose.position.x = x;//odmsg.pose.pose.position.x;
        marker.pose.position.y = y;//odmsg.pose.pose.position.y;

        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        
        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;
        
        // Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        marker.lifetime = ros::Duration();
       
	
	pubMarker.publish(marker);

      
}

  void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
  {
    int count = 0;

    sensor_msgs::PointCloud cloud;
    try
    {
        projector_.transformLaserScanToPointCloud(
          "base_laser_link",*scan_in, cloud,listener_);
    }
    catch (tf::TransformException& e)
    {
        std::cout << e.what();
        return;
    }

    int i = 0;
    float x;
    float y;
    for(  i = 0; i < 100; i++)
    {
       if (scan_in->intensities[i] == 2) 
       {
	std::cout<<"I see the treasure"<<std::endl;
        x = cloud.points[i].x;
        y = cloud.points[i].y;
	oneMarker = 1;
	break;
       }
    } 
    if (oneMarker == 1)
    {
	addMarker(x,y);
	oneMarker = 0;
        ros::Duration(10).sleep();
    } 
  }
};

int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "my_scan_to_cloud");
  ros::NodeHandle n;
  LaserScanToPointCloud lstopc(n);
  
  ros::spin();
  
  return 0;
}
