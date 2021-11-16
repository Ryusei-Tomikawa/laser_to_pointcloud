#include <iostream>
#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"
 
class LaserScanToPointCloud
{
 
 public:
    ros::NodeHandle n_;
    laser_geometry::LaserProjection projector_;
    tf::TransformListener listener_;
    message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
    tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;
    ros::Publisher scan_pub_;
    ros::Publisher scan_pub_2;
 
  LaserScanToPointCloud(ros::NodeHandle n) : n_(n), laser_sub_(n_, "scan", 10), laser_notifier_(laser_sub_,listener_, "map", 10)
  {
    laser_notifier_.registerCallback(
    boost::bind(&LaserScanToPointCloud::scanCallback, this, _1));
    laser_notifier_.setTolerance(ros::Duration(0.01));
    scan_pub_ = n_.advertise<sensor_msgs::PointCloud>("/laserscan_to_pointcloud", 1); //PoinbtCloud
    scan_pub_2 = n_.advertise<sensor_msgs::PointCloud2>("/laserscan_to_pointcloud2",1); //PointCloud2
  }
   
  void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
  {
    sensor_msgs::PointCloud cloud;
    sensor_msgs::PointCloud2 cloud2;
    try
    {
      projector_.transformLaserScanToPointCloud(
       "map",*scan_in, cloud,listener_); //PointCloud

       projector_.transformLaserScanToPointCloud(
       "map",*scan_in, cloud2,listener_); //PointCloud2
    }
    catch (tf::TransformException& e)
    {
        std::cout << e.what();
        return;
    }
       
   // Do something with cloud.
   
    scan_pub_.publish(cloud);
    scan_pub_2.publish(cloud2);
   
  }
};
   
int main(int argc, char** argv)
{
     
     ros::init(argc, argv, "lasercan_to_pointcloud");
     ros::NodeHandle n;
     LaserScanToPointCloud lstopc(n);

     std::cout << "laserScanをPointCloudに変換します" << std::endl;
     std::cout << "laserScanをPointCloud2に変換します" << std::endl;
     
     ros::spin();
     
     return 0;
}
