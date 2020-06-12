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
#include "BGKnode.h"

#include <chrono>

ros::Publisher points_pub;



int main(int argc, char **argv)
{
    ros::init(argc, argv, "trav");
    ros::NodeHandle nh;

    BGK::GridmapKDE* grid = new BGK::GridmapKDE(0.1);
    BGK::BGKnode node(nh, grid);
    
    ros::spin();

    //ros::init(argc, argv, "trav");
    //ros::NodeHandle nh;
    //ros::Publisher  map3d_pub = nh.advertise<sensor_msgs::PointCloud2 >("cloud", 100, true);


    /*
    assert(argc >= 2);
    std::cout << "pcd file:" << argv[1] << std::endl;
    auto cloud_raw = LoadPcdFile(argv[1]);
    BGK::GridmapKDE gridmap(0.1);
    auto start = std::chrono::system_clock::now(); 
    gridmap.setInput(cloud_raw);
    auto end = std::chrono::system_clock::now();
    double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count();
    std::cout<<elapsed<<" millisec"<<std::endl;
    auto cloud = gridmap.getCloud();
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);
    output.header.frame_id = "map";
    map3d_pub.publish(output);
    std::cout<<"OK:"<<cloud->points.size()<<std::endl;
    ros::spin();
    */
    // Finish
    ;
}
