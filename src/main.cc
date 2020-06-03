#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <assert.h>
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/registration/ndt.h>
#include <unordered_map>
#include <numeric>
#include <algorithm>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include "GridmapKDE.h"



ros::Publisher marker_pub;
ros::Publisher points_pub;

pcl::PointCloud<pcl::PointXYZ>::Ptr LoadPcdFile(std::string file_name)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::cout << "try to read " << file_name << " ." << std::endl;
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd \n");
        abort();
    }
    std::cout << "read " << file_name << " success!" << std::endl;
    return cloud;
}


int main(int argc, char **argv)
{


    ros::init(argc, argv, "trav");
    ros::NodeHandle nh;
    ros::Publisher  map3d_pub = nh.advertise<sensor_msgs::PointCloud2 >("cloud", 100, true);

    assert(argc >= 2);
    std::cout << "pcd file:" << argv[1] << std::endl;
    auto cloud_raw = LoadPcdFile(argv[1]);
    BGK::GridmapKDE gridmap(0.1);
    gridmap.setInput(cloud_raw);
    auto cloud = gridmap.getCloud();
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);
    output.header.frame_id = "map";
    map3d_pub.publish(output);
    std::cout<<"OK:"<<cloud->points.size()<<std::endl;
    ros::spin();
    // Finish
    ;
}
