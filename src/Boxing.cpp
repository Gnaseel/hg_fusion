#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include<visualization_msgs/Marker.h>
#include<visualization_msgs/MarkerArray.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>    
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <list>
#include <string>
using namespace std;
class Box{
    public:
    float minX;
    float minY;
    float minZ;
    float maxX;
    float maxY;
    float maxZ;
    int index;
    Box(){
        minX=minY=minZ = 9999;
        maxX=maxY=maxZ = -9999;
        index=-1;
    }
};

int objectIndex=1;
list<Box> preBoxes;
ros::Publisher marker_pub;
ros::Publisher markerText_pub;
bool getIOU(Box sor, Box tar, int temp){
    cout<<sor.minX<<"  "<<sor.maxX<<"  "<<tar.minX<<"  "<<tar.maxX<<"  "<<endl;
    cout<<sor.minY<<"  "<<sor.maxY<<"  "<<tar.minY<<"  "<<tar.maxY<<"  "<<endl;
    cout<<sor.minZ<<"  "<<sor.maxZ<<"  "<<tar.minZ<<"  "<<tar.maxZ<<"  "<<endl;

    float dx, dy, dz;
    dx=dy=dz=0;
    float threshold=0;
    if (sor.maxX <= tar.minX || tar.maxX <= sor.minX ){
        dx=0;
        cout<<"XZERO   ";
        cout<<sor.minX<<"  "<<sor.maxX<<"  "<<tar.minX<<"  "<<tar.maxX<<"  ";
    }
    else if(sor.minX <= tar.minX&& tar.minX <= sor.maxX)
        dx = sor.maxX - tar.minX;
    else if (sor.minX <= tar.minX && tar.maxX <= sor.maxX)
        dx = tar.maxX - tar.minX;
    else if (tar.minX <= sor.minX && sor.maxX <= tar.maxX)
        dx = sor.maxX - sor.minX;
    else if (sor.minX <= tar.maxX && tar.maxX <= sor.maxX)
        dx = tar.maxX - sor.minX;
    else{
        cout<<"XXXXXX WHY?????????????"<<endl;
        for(;;);
    }
    if (sor.maxY <= tar.minY || tar.maxY <= sor.minY ){
        dy=0;
        cout<<"YZERO  ";

    }
    else if(sor.minY <= tar.minY&& tar.minY <= sor.maxY)
        dy = sor.maxY - tar.minY;
    else if (sor.minY <= tar.minY && tar.maxY <= sor.maxY)
        dy = tar.maxY - tar.minY;
    else if (tar.minY <= sor.minY && sor.maxY <= tar.maxY)
        dy = sor.maxY - sor.minY;
    else if (sor.minY <= tar.maxY && tar.maxY <= sor.maxY)
        dy = tar.maxY - sor.minY;
    else{
        cout<<"bbbb"<<endl;
        // cout<<sor.maxY<<"  ";
        cout<<"YYYYYYYYYYY WHY?????????????";
        for(;;);
    }

    if (sor.maxZ <= tar.minZ || tar.maxZ <= sor.minZ ){
        dz=0;
        cout<<"ZZERO  ";

    }
    else if(sor.minZ <= tar.minZ && tar.minZ <= sor.maxZ)
        dz = sor.maxZ - tar.minZ;
    else if (sor.minZ <= tar.minZ && tar.maxZ <= sor.maxZ)
        dz = tar.maxZ - tar.minZ;
    else if (tar.minZ <= sor.minZ && sor.maxZ <= tar.maxZ)
        dz = sor.maxZ - sor.minZ;
    else if (sor.minZ <= tar.maxZ && tar.maxZ <= sor.maxZ)
        dz = tar.maxZ - sor.minZ;
    else{
        cout<<"ZZZZZZZZ WHY?????????????"<<endl;
        cout<<sor.minZ<<"  "<<sor.maxZ<<"  "<<tar.minZ<<"  "<<tar.maxZ<<"  ";
        for(;;);
    }

    float sor_vol = (sor.maxX-sor.minX)*(sor.maxY-sor.minY)*(sor.maxZ-sor.minZ);
    if(sor_vol<0)
    {
        cout<<"WHY?????????????"<<endl;
        for(;;);
    }
    float tar_vol = (tar.maxX-tar.minX)*(tar.maxY-tar.minY)*(tar.maxZ-tar.minZ);

    
    // cout<<"sor volume = "<<sor_vol<<endl;
    // cout<<"tar volume = "<<tar_vol<<endl;
    // cout<<"IOU = "<<dx*dy*dz<<endl;
    if (sor_vol*threshold < dx*dy*dz){
        // cout<<"sor Index = "<<sor.index<<endl;
        cout<<"  sor Index = "<<sor.index<<"  Temp Index = "<<temp;
        cout<<"  sor volume = "<<sor_vol;
        cout<<"  tar volume = "<<tar_vol;
        cout<<"  IOU = "<<dx*dy*dz<<endl;
        return true;
    }else{
        cout<<"sor Index = "<<sor.index<<"  Temp Index = "<<temp;
        cout<<"  sor volume = "<<sor_vol;
        cout<<"  tar volume = "<<tar_vol;
        cout<<"  IOU = "<<dx*dy*dz<<endl;
        // cout<<"XXXXXXXxxX"<<endl;
    }

    return false;
}
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
    visualization_msgs::MarkerArray markerTextArray;
    cout<<"-------------------------------------------------------"<<endl;
    cout<<"SIZE = "<<arrayLength<<endl;
    // for(int i=1;i<=arrayLength;i++){
    //     cout<<"IDX = "<<i<<endl;
    //     cout<<box[i].minX<<"  "<<box[i].maxX<<endl;
    //     cout<<box[i].minY<<"  "<<box[i].maxY<<endl;
    //     cout<<box[i].minZ<<"  "<<box[i].maxZ<<endl;
    // }
    // int temptemp=1;
    // for(list<Box>::iterator itBox = preBoxes.begin() ; itBox!=preBoxes.end() ; itBox++){
    //     cout<<"PRE IDX  "<<temptemp++<<endl;
    //     cout<<itBox->minX<<"  "<<itBox->maxX<<endl;
    //     cout<<itBox->minY<<"  "<<itBox->maxY<<endl;
    //     cout<<itBox->minZ<<"  "<<itBox->maxZ<<endl;
    // }
    //------------------------------------- Object Association ------------------------------------------
    if(preBoxes.size()==0){
        for(int i=1;i<=arrayLength;i++){
            box[i].index=objectIndex++;
        }
    }
    else{
        for(int i=1;i<=arrayLength;i++){
            bool isNew=true;
            int index=99;
            
            int temp=0;
            for(list<Box>::iterator itBox = preBoxes.begin() ; itBox!=preBoxes.end() ; itBox++){
                if(getIOU(*itBox, box[i],temp)){
                    isNew=false;
                    index = itBox->index;
                    cout<<"                     !!  "<<itBox->index<<endl;
                }
                temp++;
            }
            box[i].index = index;
        }
    }
    
    //------------------------------------- MARKER PRINT ------------------------------------------
    for(int i=1;i<=arrayLength;i++){
        // cout<<"IDX = "<<i<<endl;
        // cout<<"X = "<<box[i].minX<<"  ,  "<<box[i].maxX<<endl;
        // cout<<"Y = "<<box[i].minY<<"  ,  "<<box[i].maxY<<endl;
        // cout<<"Z = "<<box[i].minZ<<"  ,  "<<box[i].maxZ<<endl;
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

        visualization_msgs::Marker markerText;
        markerText.header.frame_id = "/velodyne";
        markerText.header.stamp = ros::Time::now();
        markerText.ns = "boxText";
        markerText.text="Object "+to_string(box[i].index);
        markerText.id = i;
        markerText.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        markerText.action = visualization_msgs::Marker::MODIFY;
        markerText.pose.position.x = (box[i].minX + box[i].maxX)/2;
        markerText.pose.position.y = (box[i].minY + box[i].maxY)/2;
        markerText.pose.position.z = (box[i].minZ + box[i].maxZ)/2 + 0.1;
        markerText.pose.orientation.x = 0.0;
        markerText.pose.orientation.y = 0.0;
        markerText.pose.orientation.z = 0.0;
        markerText.pose.orientation.w = 1.0;
        markerText.scale.z = 0.3;
        markerText.color.r = 1.0f;
        markerText.color.g = 0.0f;
        markerText.color.b = 0.0f;
        markerText.color.a = 0.4;
        // markerText.lifetime = ros::Duration();
        markerTextArray.markers.push_back(markerText);
    }



    preBoxes.clear();
    for(int i=1;i<=arrayLength;i++){
        preBoxes.push_back(box[i]);
    }


    marker_pub.publish(markerArray);
    markerText_pub.publish(markerTextArray);
    return;
}
int main(int argc, char** argv){
    ros::init(argc, argv, "Boxing");
    ros::NodeHandle nh;
    ros::Subscriber clustered_sub = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_filtered", 10, boxingCallback);
    marker_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker", 1);
    markerText_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_markerText", 1);
    ros::Rate r(1);
    while(ros::ok()){
        ros::spinOnce();
        r.sleep();
    }

}