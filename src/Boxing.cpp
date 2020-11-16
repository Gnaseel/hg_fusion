#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include<visualization_msgs/Marker.h>
#include<visualization_msgs/MarkerArray.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>    
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

using namespace std;
ros::Publisher marker_pub;
class Box{
    public:
    float minX;
    float minY;
    float minZ;
    float maxX;
    float maxY;
    float maxZ;
    Box(){
        minX=minY=minZ = 9999;
        maxX=maxY=maxZ = -9999;
    }
};
void sizeCheck(Box* pBox,float x, float y, float z){
    if(x < pBox->minX) pBox->minX = x;
    if(y < pBox->minY) pBox->minY = y;
    if(z < pBox->minZ) pBox->minZ = z;

    if(x > pBox->maxX) pBox->maxX = x;
    if(y > pBox->maxY) pBox->maxY = y;
    if(z > pBox->maxZ) pBox->maxZ = z;
}
void boxingCallback(const sensor_msgs::PointCloud2ConstPtr& msg){
    pcl::PointCloud<pcl::PointXYZI>::Ptr pCloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg,*pCloud);
    pcl::PointCloud<pcl::PointXYZI>::iterator iter;


    Box box[50];
    int arrayLength=0;
    for(iter = pCloud->begin();iter!=pCloud->end();iter++){
        int intensity = iter->intensity;
        if(intensity>arrayLength) arrayLength=intensity;
        sizeCheck(&box[intensity], iter->x, iter->y, iter->z);
    }




    visualization_msgs::MarkerArray markerArray;

    for(int i=0;i<=arrayLength;i++){
        cout<<"IDX = "<<i<<endl;
        cout<<"X = "<<box[i].minX<<"  ,  "<<box[i].maxX<<endl;
        cout<<"Y = "<<box[i].minY<<"  ,  "<<box[i].maxY<<endl;
        cout<<"Z = "<<box[i].minZ<<"  ,  "<<box[i].maxZ<<endl;
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/velodyne";
        marker.header.stamp = ros::Time::now();
        marker.ns = "box";
        marker.id = i;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = (box[i].minX + box[i].maxX)/2;
        marker.pose.position.y = (box[i].minY + box[i].maxY)/2;
        marker.pose.position.z = (box[i].minZ + box[i].maxZ)/2;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = box[i].maxX - box[i].minX;
        marker.scale.y = box[i].maxY - box[i].minY;
        marker.scale.z = box[i].maxZ - box[i].minZ;
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 0.4;
        marker.lifetime = ros::Duration();
        markerArray.markers.push_back(marker);
    }

    
    marker_pub.publish(markerArray);
    return;
}
int main(int argc, char** argv){
    ros::init(argc, argv, "Boxing");
    ros::NodeHandle nh;
    ros::Subscriber clustered_sub = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_filtered", 10, boxingCallback);
    marker_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker", 1);
    ros::Rate r(100);
    while(ros::ok()){
        ros::spinOnce();
        r.sleep();
    }

}