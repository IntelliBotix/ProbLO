#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <boost/filesystem.hpp>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <pcl/visualization/pcl_visualizer.h>
#include <chrono>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ndt.h>
#include <deque>
#include <queue>
#include <nav_msgs/Odometry.h>
#include <boost/format.hpp>
#include <boost/circular_buffer.hpp>
#include <pclomp/ndt_omp.h>
#include <pclomp/gicp_omp.h>
#include <unordered_set>
#include <tuple>
#include <cmath>
#include <unordered_map>




using namespace std;
namespace fs = boost::filesystem;
using PLM3 = std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>>;
using PLV3 = std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>;




template <typename PointT>
void removeInvalidFromPointCloud(const pcl::PointCloud<PointT>& cloudIn, pcl::PointCloud<PointT>& cloudOut, float minRangeThr)
{
    if(&cloudIn != &cloudOut) {
        cloudOut.header = cloudIn.header;
        cloudOut.resize(cloudIn.size());
        cloudOut.sensor_origin_ = cloudIn.sensor_origin_;
        cloudOut.sensor_orientation_ = cloudIn.sensor_orientation_;
    }
    
    std::size_t j = 0;
    float thrSquare = minRangeThr * minRangeThr;
    for(const PointT& pt : cloudIn) {
        if(!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z) ||
          (pt.x * pt.x + pt.y * pt.y + pt.z * pt.z) < thrSquare) {
            continue;
        }
        cloudOut[j++] = pt;
    }

    if(j != cloudIn.size()) {
        cloudOut.resize(j);
    }

    cloudOut.height = 1;
    cloudOut.width = static_cast<std::uint32_t>(j);
    cloudOut.is_dense = true;
}






Eigen::Vector3d t_last_keyFrame_(0,0,0);
Eigen::Vector3d t_curr_(0,0,0);
int keyFrameCount = 0;
bool isKeyFrame(int FC_inner)
{
  return true;
}





void transformCloud(const Eigen::Matrix3f& R, const Eigen::Vector3f& t, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudInPtr)
{
    int size = cloudInPtr->size();
    #pragma omp parallel for num_threads(8)
    for(int i=0; i<size; ++i)
    {
        pcl::PointXYZ& pt = cloudInPtr->points[i];
        pt.getVector3fMap() = R * pt.getVector3fMap() + t;
    }
}



std::mutex mBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> singleframe_cloudBuf;
void laserCloudHandle(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg) 
{
    mBuf.lock();
    singleframe_cloudBuf.push(laserCloudMsg);
    mBuf.unlock();
}




int main(int argc, char** argv) 
{
  ros::init(argc, argv, "icp_node1");
  ros::NodeHandle nh;
  ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 5000, laserCloudHandle);
  ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud", 5000);
  ros::Publisher local_map_pub = nh.advertise<sensor_msgs::PointCloud2>("local_map", 5000);
  ros::Publisher trans_currentframe_pub = nh.advertise<sensor_msgs::PointCloud2>("trans_currentframe", 5000);
  ros::Publisher trajectory_pub = nh.advertise<geometry_msgs::PoseArray>("trajectory", 5000);
  ros::Publisher pubLaserOdometry = nh.advertise<nav_msgs::Odometry>("ndt_odom", 5000);


  string root_directory;
  nh.param<string>("root_directory", root_directory, "");
  string seq_Num;
  nh.param<string>("seq_Num", seq_Num, "");
  int bag_Num;
  nh.param<int>("bag_Num", bag_Num, 0);


  // sensor pose sequence
  std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses;
  poses.resize(100000);
  poses[0].setIdentity();


  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > cloud_ptr_vector;
  int map_size = 1;
  cloud_ptr_vector.resize(map_size);
  for(int i = 0; i < map_size; i++)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_ptr_vector[i] = cloud_ptr;
  }



  // std::vector<int> num_threads = {1, omp_get_max_threads()};
  // std::vector<std::pair<std::string, pclomp::NeighborSearchMethod>> search_methods = {
  //   {"KDTREE", pclomp::KDTREE},
  //   {"DIRECT7", pclomp::DIRECT7},
  //   {"DIRECT1", pclomp::DIRECT1}
  // };
  // // registration method
  // std::cout << "--- pcl::NDT ---" << std::endl;
  // pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr ndt(new pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>());
  // ndt->setResolution(1.0);
  // ndt->setInputTarget(target_cloud);



  std::cout << "--- pcl::ndt_omp ---" << std::endl;
  pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr ndt_omp(new pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>());
  ndt_omp->setResolution(1);
  ndt_omp->setNumThreads(5);
  ndt_omp->setNeighborhoodSearchMethod(pclomp::DIRECT1);
  // ndt_omp->setMaximumIterations(max_iterations);
  // ndt_omp->setMaximumIterations(1);



  geometry_msgs::PoseArray pose_array;
  pose_array.header.frame_id = "map";
  pcl::PointCloud<pcl::PointXYZ>::Ptr all_map(new pcl::PointCloud<pcl::PointXYZ>());
  int frameCount = -1;
  pcl::PointCloud<pcl::PointXYZ>::Ptr rawPointCloudPtr_(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr rawPtr_xyzi(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_map_tmp(new pcl::PointCloud<pcl::PointXYZ>());


  // scan-to-map
  ros::Rate loop(100);
  while(ros::ok()) 
  {
    ros::spinOnce();

    if(!singleframe_cloudBuf.empty()) 
    {
      mBuf.lock();
      frameCount++;
      std::cout << "frameCount: " << frameCount << std::endl;

      ros::Time sweepStamp_ = singleframe_cloudBuf.front()->header.stamp;

      rawPtr_xyzi->clear();
      pcl::fromROSMsg(*singleframe_cloudBuf.front(), *rawPtr_xyzi);
      singleframe_cloudBuf.pop();

      mBuf.unlock();
      

      removeInvalidFromPointCloud(*rawPtr_xyzi, *rawPtr_xyzi, 2);

      rawPointCloudPtr_->clear();
      *rawPointCloudPtr_ = *rawPtr_xyzi;


      if(frameCount == 0)
      {
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        *tmp_cloud = *rawPointCloudPtr_;

        *cloud_ptr_vector[0] = *tmp_cloud;
      }
      else
      {
        pcl::PointCloud<pcl::PointXYZ>::Ptr target_map(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_origi(new pcl::PointCloud<pcl::PointXYZ>());
        
        for(int i = 0; i < map_size; i++)
        {
          *target_map += *cloud_ptr_vector[i];
        }
        std::cout << std::endl;
        pcl::PointCloud<pcl::PointXYZ>::Ptr target_map_tmp(new pcl::PointCloud<pcl::PointXYZ>());
        *target_map_tmp = *target_map;

        *source_cloud = *rawPointCloudPtr_;

        ndt_omp->setInputTarget(target_map_tmp);
        ndt_omp->setInputSource(source_cloud);

        // initial guess
        Eigen::Isometry3d initial_pose;
        if(frameCount == 1)
        {
          initial_pose = Eigen::Isometry3d::Identity();
        }
        if(frameCount>=2)
        {
          initial_pose = poses[frameCount-1] * ( poses[frameCount-2].inverse() * poses[frameCount-1]);
        }


        pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>);
        ndt_omp->align(*aligned, initial_pose.matrix().cast<float>());

        poses[frameCount] = ndt_omp->getFinalTransformation().cast<double>();

        transformCloud(poses[frameCount].rotation().matrix().cast<float>(), poses[frameCount].translation().matrix().cast<float>(), source_cloud);
        
        if( isKeyFrame(frameCount) )
        {
          keyFrameCount++;
          int new_id = keyFrameCount % map_size;
          *cloud_ptr_vector[new_id] = *source_cloud;
        }

      }


      Eigen::Vector3d t_c = poses[frameCount].translation();
      Eigen::Quaterniond q_current(poses[frameCount].rotation());
      nav_msgs::Odometry laserOdometry;
      // laserOdometry.header.frame_id = "map";
      laserOdometry.header.frame_id = "world";
      laserOdometry.child_frame_id = "base_link";
      laserOdometry.header.stamp = sweepStamp_;
      laserOdometry.pose.pose.orientation.x = q_current.x();
      laserOdometry.pose.pose.orientation.y = q_current.y();
      laserOdometry.pose.pose.orientation.z = q_current.z();
      laserOdometry.pose.pose.orientation.w = q_current.w();
      laserOdometry.pose.pose.position.x = t_c.x();
      laserOdometry.pose.pose.position.y = t_c.y();
      laserOdometry.pose.pose.position.z = t_c.z();
      pubLaserOdometry.publish(laserOdometry);

    }

    loop.sleep();
  }

  return 0;
}
