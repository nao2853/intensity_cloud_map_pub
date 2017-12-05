#include <ros/ros.h>
#include <sys/time.h>
#include <tf/transform_listener.h>
//#include <tf2_ros/transform_listener.h>

#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Point32.h"
#include "std_srvs/Empty.h"

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
//#include <pcl/pclPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <laser_geometry/laser_geometry.h>
#include "sensor_msgs/point_cloud_conversion.h"

#include <iostream>
#include <string>
//#include <vector>

#include "map/map.h"
using namespace std;

class IntensityCloudMapPubNode
{
  public:
    IntensityCloudMapPubNode();
    ~IntensityCloudMapPubNode();

    void requestMap();
    //map_t* convertMap(const nav_msgs::OccupancyGrid& map_msg);
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    sensor_msgs::PointCloud2 accumulateCloud(const sensor_msgs::LaserScan& scan);
  private:
    ros::Subscriber scan_sub_;
    ros::Publisher assemble_cloud2_pub_;
    ros::Publisher intensity_cloud_pub_;

    laser_geometry::LaserProjection projector_;
    
    //tf2_ros::Buffer tfBuffer;
    tf::TransformListener tf_;
    
    ros::Duration cloud_pub_interval_;
    std::vector<sensor_msgs::PointCloud2> cloud2_v_;
    
};

IntensityCloudMapPubNode::
IntensityCloudMapPubNode()
{
  ros::NodeHandle nh_;
  scan_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/diag_scan", 100, &IntensityCloudMapPubNode::scanCallback, this);
  assemble_cloud2_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("assemble_cloud2", 100, false);
  intensity_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("intensity_cloud", 100, false);
  cloud_pub_interval_.fromSec(0.5);
}

IntensityCloudMapPubNode::
~IntensityCloudMapPubNode()
{
  cloud2_v_.clear();
  //delete pcl_cloud;
  
}

sensor_msgs::PointCloud2
IntensityCloudMapPubNode::accumulateCloud(const sensor_msgs::LaserScan& scan)
{
  sensor_msgs::PointCloud2 cloud2;
  projector_.projectLaser(scan, cloud2);
  
  return cloud2;
}

void
IntensityCloudMapPubNode::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  static ros::Time last_cloud_pub_(0.0);
  
  cloud2_v_.push_back(accumulateCloud(*scan));
  if(cloud2_v_.size() > 50)
  {
    cloud2_v_.erase(cloud2_v_.begin());
  }
  if((scan->header.stamp - last_cloud_pub_) > cloud_pub_interval_)
  {
    map_t* map = map_alloc();
    map->size_x = 600;
    map->size_y = 600;
    map->scale = 0.05;
    map->origin_x = 0.0;
    map->origin_y = 0.0;
    map->cells = (map_cell_t*)malloc(sizeof(map_cell_t) * map->size_x * map->size_y);
    
    for(int i = 0; i < (map->size_x * map->size_y); i++)
    {
      map->cells[i].ave = 0.0;
      map->cells[i].var = 0.0;
    }
    sensor_msgs::PointCloud2 cloud2_msg;
    cloud2_msg.data.clear();
    for(int i=0; i < cloud2_v_.size(); i++)
    {
      tf::StampedTransform transform;
      try
      {
        tf_.lookupTransform(scan->header.frame_id,
                                      scan->header.stamp,
                                      cloud2_v_.at(i).header.frame_id,
                                      cloud2_v_.at(i).header.stamp,
                                      "/odom",
                                      transform);
      }
      catch(tf::TransformException &ex)
      {
        //last_cloud_pub_ = scan->header.stamp;
        //cloud2_v_.clear();
        continue;
      }
      //tf_.lookupTransform("base_link",
      //                         scan->header.stamp,
      //                         cloud2_v_.at(i).header.frame_id,
      //                         cloud2_v_.at(i).header.stamp,
      //                         "odom",
      //                         transform);
      pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud (new pcl::PointCloud<pcl::PointXYZI>);
      pcl::fromROSMsg(cloud2_v_.at(i), *pcl_cloud);
      pcl_ros::transformPointCloud(*pcl_cloud,
                                   *pcl_cloud,
                                   transform);
      for(int j=0; j < pcl_cloud->points.size(); j++)
      {
        if(pcl_cloud->points.at(j).z>0.05)continue;
        map_updata_cell(map, pcl_cloud->points.at(j).x, pcl_cloud->points.at(j).y, pcl_cloud->points.at(j).intensity);
      }
      sensor_msgs::PointCloud2 transform_cloud2;
      pcl::toROSMsg(*pcl_cloud, transform_cloud2);
      pcl::concatenatePointCloud(cloud2_msg, transform_cloud2, cloud2_msg);
    }
    cloud2_msg.header.frame_id = "base_link";
    cloud2_msg.header.stamp = scan->header.stamp;
    assemble_cloud2_pub_.publish(cloud2_msg);
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloudi; //(new pcl::PointCloud<pcl::PointXYZI>);
    pcl_cloud->points.clear();
    for(int i=0; i < map->size_x; i++){
      for(int j=0; j < map->size_y; j++){
        if(!(map->cells[MAP_INDEX(map,i,j)].ave)) continue;
        pcl::PointXYZI pcl_point;
        pcl_point.x = MAP_WXGX(map,i);
        pcl_point.y = MAP_WYGY(map,j);
        pcl_point.z = map->cells[MAP_INDEX(map,i,j)].var;
        pcl_point.intensity = map->cells[MAP_INDEX(map,i,j)].ave;
        pcl_cloud->points.push_back(pcl_point);
      }
    }
    cloud2_msg.data.clear();
    toROSMsg(*pcl_cloud, cloud2_msg);
    
    cloud2_msg.header.frame_id = "base_link";
    cloud2_msg.header.stamp = scan->header.stamp;
    intensity_cloud_pub_.publish(cloud2_msg);
    last_cloud_pub_ = scan->header.stamp;
    
    map_free(map);
    map = NULL;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "intensity_cloud_map_pub");
  IntensityCloudMapPubNode icm;
  ros::spin();
  return 0;
}
