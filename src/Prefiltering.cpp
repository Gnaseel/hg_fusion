#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>

#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <std_msgs/Bool.h>
#include "std_msgs/Float32.h"
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include "cmath"
using namespace std;
using PointT = pcl::PointXYZI;
// standard
float P_first_x=1.8;
float P_first_y=1;

float P_second_x=1.8;
float P_second_y=-1;

float P_third_x=2.8;
float P_third_y=-1;

float P_fourth_x=2.8;
float P_fourth_y=1;

ros::Publisher pub_filtered;
// void cloud_cb(pcl::PCLPointCloud2 input)
pcl::PCLPointCloud2 cloud_cb(pcl::PCLPointCloud2ConstPtr input)
{

  pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
  
  pcl::fromPCLPointCloud2(*input,*cloud_filtered);
  
//   pcl::fromROSMsg(*input, *cloud_filtered);

  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  tree->setInputCloud(cloud_filtered);
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> ECE;
  ECE.setClusterTolerance(0.4); // 1m
  ECE.setMinClusterSize(15); // 몇 개부터 한 군집?
  ECE.setMaxClusterSize(10000); // 몇 개까지 한 군집?
  ECE.setSearchMethod(tree);
  ECE.setInputCloud(cloud_filtered);
  ECE.extract(cluster_indices);

    pcl::PCLPointCloud2 output;
  int j = 0;
  pcl::PointCloud<PointT> TotalCloud;
  std::vector<pcl::PointCloud<PointT>> arr;
  arr.clear();
  for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it, j++)
  {
    for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
    {
      PointT pt = cloud_filtered->points[*pit];
      PointT pt2;

      pt2.x = pt.x, pt2.y = pt.y, pt2.z = pt.z;
      pt2.intensity = (float)(j + 1);

      TotalCloud.push_back(pt2);
      
    }
  }

  pcl::toPCLPointCloud2(TotalCloud, output);

//   sensor_msgs::PointCloud2 output;
//   pcl_conversions::fromPCL(cloud_p, output);

//   output.header.frame_id = "velodyne";
    return output;

}
//--------------------- FILTER APPLY----------------
pcl::PCLPointCloud2 roi_filter(pcl::PCLPointCloud2ConstPtr cloudPtr)
{
  pcl::PCLPointCloud2 output;
  pcl::CropBox<pcl::PCLPointCloud2> cropFilter;
  cropFilter.setInputCloud(cloudPtr);
  cropFilter.setMin(Eigen::Vector4f(-5, -5, -0.5, 0));
  cropFilter.setMax(Eigen::Vector4f(5.0, 5, 4.5, 0));
  cropFilter.filter(output);
//roi_pub.publish(output);
  return output;
}
pcl::PCLPointCloud2 voxelGrid(pcl::PCLPointCloud2ConstPtr cloudPtr)
{
  pcl::PCLPointCloud2 output;
  pcl::VoxelGrid<pcl::PCLPointCloud2> VG;
  VG.setInputCloud(cloudPtr);
  VG.setLeafSize(0.1f, 0.1f, 0.1f);
  VG.filter(output);
//vg_pub.publish(output);
  return output;
}

void applyFilter(const sensor_msgs::PointCloud2ConstPtr& msg){
    sensor_msgs::PointCloud2 output;
    pcl::PCLPointCloud2* source = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr sourcePtr(source);
    pcl_conversions::toPCL(*msg, *source);
    pcl::PCLPointCloud2* cloud= new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    *cloud=*source;
    *cloud = roi_filter(cloudPtr);  //ROI 필터 적용
    *cloud = voxelGrid(cloudPtr);   //VoxelGrid 적용
    *cloud = cloud_cb(cloudPtr);

    pcl_conversions::fromPCL(*cloud, output);
    output.header.frame_id = "velodyne";

    pub_filtered.publish(output);

    // obstacle_detect(*cloud);        //물체 가ㅁ지
}
//-------------------------------------------------------
int main(int argc, char ** argv)
{
    ros::init(argc, argv, "Prefiltering");
    ros::NodeHandle nh;
    ros::Subscriber vlp_sub = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 10, applyFilter);
    pub_filtered = nh.advertise<sensor_msgs::PointCloud2> ("/velodyne_filtered", 50);

    ros::Rate r(100);


    while(ros::ok()){
        ros::spinOnce();
        r.sleep();
    }
}