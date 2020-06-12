

#ifndef BGK_ROS_H_
#define BGK_ROS_H_

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <message_filters/time_synchronizer.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <eigen_conversions/eigen_msg.h>

#include "GridmapKDE.h"

namespace BGK
{
     typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped,
                                                             sensor_msgs::PointCloud2>
         syncPolicy;

     class BGKnode
     {
     public:
          void callbackPoints(const sensor_msgs::PointCloud2ConstPtr &points_msg){
               pcl::PointCloud<pcl::PointXYZI> tmp;
               pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());

               Eigen::Matrix4f t_localizer(Eigen::Matrix4f::Identity());
               //tf::Transform transform;

               pcl::fromROSMsg(*points_msg, tmp);
               Eigen::Affine3d pose;
               pose.setIdentity();
               //tf::poseMsgToEigen(pose_msg->pose, pose);

               pcl::transformPointCloud(tmp, *transformed_scan_ptr, pose);

               gridmap_->update(transformed_scan_ptr);
               auto cloud = gridmap_->getCloud();
               sensor_msgs::PointCloud2 output;
               pcl::toROSMsg(*cloud, output);
               output.header.frame_id = "map";

               map3d_pub_.publish(output);
               std::cout << "out\n";
          }


          void callback(const geometry_msgs::PoseStampedConstPtr &pose_msg,
                        const sensor_msgs::PointCloud2ConstPtr &points_msg)
          {

               pcl::PointCloud<pcl::PointXYZI> tmp;
               pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());

               Eigen::Matrix4f t_localizer(Eigen::Matrix4f::Identity());
               //tf::Transform transform;

               pcl::fromROSMsg(*points_msg, tmp);
               Eigen::Affine3d pose;
               pose.setIdentity();
               tf::poseMsgToEigen(pose_msg->pose, pose);

               pcl::transformPointCloud(tmp, *transformed_scan_ptr, pose);
               
               gridmap_->update(transformed_scan_ptr);
               auto cloud = gridmap_->getCloud();
               sensor_msgs::PointCloud2 output;
               pcl::toROSMsg(*cloud, output);
               output.header.frame_id = "map";

               
               map3d_pub_.publish(output);
               std::cout<<"out\n";
               


               /*

               gridmap_->setInput(cloud_raw);
               auto end = std::chrono::system_clock::now();
               double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
               std::cout << elapsed << " millisec" << std::endl;
               auto cloud = gridmap.getCloud();
               sensor_msgs::PointCloud2 output;
               pcl::toROSMsg(*cloud, output);
               output.header.frame_id = "map";
               map3d_pub.publish(output);
               std::cout << "OK:" << cloud->points.size() << std::endl;
               */
          }

          BGKnode(ros::NodeHandle nh, GridmapKDE *gridmap)
              : gridmap_(gridmap), nh_(nh),q_size_(100)
          {
               map3d_pub_ = nh.advertise<sensor_msgs::PointCloud2 >("cloud", 100, true);
               std::string pose_topic("/current_pose");
               std::string points_topic("/points_raw");
               ros::NodeHandle nh_;
               pose_sub_ = new message_filters::Subscriber<geometry_msgs::PoseStamped>(nh_, pose_topic, q_size_);
               points_sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, points_topic, q_size_);
               sync_ = new message_filters::Synchronizer<syncPolicy>(syncPolicy(q_size_), *pose_sub_, *points_sub_);
               sync_->registerCallback(boost::bind(&BGKnode::callback, this, _1, _2));

               //ros::Subscriber sub = nh_.subscribe("/tran_points", 1000, &BGKnode::callbackPoints, this);

               

          }

     private:
          message_filters::Subscriber<geometry_msgs::PoseStamped> *pose_sub_;
          message_filters::Subscriber<sensor_msgs::PointCloud2> *points_sub_;
          message_filters::Synchronizer<syncPolicy> *sync_;
          GridmapKDE *gridmap_;
          ros::NodeHandle nh_;
          const int q_size_;
          ros::Publisher  map3d_pub_;

     };

     //The callback method
} // namespace BGK

#endif
