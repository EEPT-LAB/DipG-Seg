
/*****************************************************************************************
This demo node is for the application running on the topic "/pointcloud"
Before running this node, you should make sure that the topic "/pointcloud" is published coorectly. 

----------------------------------------------MOST INMPORTANTLY!!!---------------------------------
you should check the configuration of your LiDAR sensor, which should be registered in the ./include/projection_params.h file. 

Then, you can run this node as follows in your terminal:
```bash
source {the path of your catkin workspace}/devel/setup.bash
roscore
roslaunch dipgseg demo.launch
```
Also, this node would publish the ground and non-ground pointclouds to the topic "/ground" and "/non_ground" respectively.
Thus, you can visualize the results in rviz by importing the rviz config file in DipG-Seg/rviz/reprj.rviz.
*****************************************************************************************/

#include "time_utils.h"
#include "kitti_loader.h"
#include "evaluate.h"
#include "dipgseg.h"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
using namespace KittiLoader;

bool verbose = false;
ros::Publisher ground_pub;
ros::Publisher non_ground_pub;

void callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, DIPGSEG::Dipgseg& dipgseg){
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::fromROSMsg(*cloud_msg, cloud);
    pcl::PointCloud<pcl::PointXYZI> cloud_ground, cloud_non_ground;
    
    dipgseg.segment_ground(cloud, cloud_ground, cloud_non_ground);
    double time_all = dipgseg.get_whole_time();
    double time_seg = dipgseg.get_seg_time();
    if(verbose){
      printf("-------------time_all: %f, time_seg: %f\n", time_all, time_seg);
      printf("cloud size: %ld, cloud_ground size: %ld, cloud_non_ground size: %ld\n", cloud.size(), cloud_ground.size(), cloud_non_ground.size());
    }

    sensor_msgs::PointCloud2 ground_msg, non_ground_msg;

    pcl::toROSMsg(cloud_ground, ground_msg);
    ground_msg.header.frame_id = "laser_link";
    ground_msg.header.stamp = ros::Time::now();
    ground_pub.publish(ground_msg);

    pcl::toROSMsg(cloud_non_ground, non_ground_msg);
    non_ground_msg.header.frame_id = "laser_link";
    non_ground_msg.header.stamp = ros::Time::now();
    non_ground_pub.publish(non_ground_msg);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "demo_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);
    nh.param<bool>("verbose", verbose, false);
    ground_pub= nh.advertise<sensor_msgs::PointCloud2>("/ground", 10);
    non_ground_pub= nh.advertise<sensor_msgs::PointCloud2>("/non_ground", 10);
    DIPGSEG::Dipgseg dipgseg;
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/pointcloud", 1, boost::bind(callback, _1, dipgseg));
    printf("-----------------------waiting for pointcloud...----------------------\n");
    ros::spin();
    return 0;
}