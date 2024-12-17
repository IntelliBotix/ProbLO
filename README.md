# ProbLO
## A Unified Probabilistic Primitive-Driven LiDAR Odometry

LOAM is a widely used feature-based LiDAR odometry method, in which the scan-to-map matching receives the pose from scan-to-scan matching and further refines the pose estimation result. Probabilistic LOAM is an improved version of LOAM using probabilistic primitives. In the scan-to-map matching, the local neighborhood of the projection feature is represented in the form of probabilistic geometry. Then, the point-to-probabilistic line and point-to-probabilistic plane constraints are constructed for pose optimization. This code is modified from LOAM and [A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM).
NDT is a typical distribution-based registration method, where the target map is voxelized and data association takes the form of point-to-voxel correspondence. Probabilistic NDT is an improved version by extracting probabilistic geometric structures in voxels. The corresponding probabilistic constraints are used to facilitate pose estimation. This code is modified from [ndt_omp](https://github.com/koide3/ndt_omp).


## 1. Prerequisites
### 1.1 **Ubuntu** and **ROS**
Ubuntu 64-bit 20.04 or 22.04.
ROS Melodic or Noetic. [ROS Installation](http://wiki.ros.org/ROS/Installation)


### 1.2. **Eigen Library**
Follow [Eigen Installation](http://eigen.tuxfamily.org/dox/GettingStarted.html).

### 1.3. **PCL**
Follow [PCL Installation](http://www.pointclouds.org/downloads/linux.html).


## 2. Build Probabilistic A-LOAM
Clone the repository and catkin_make:
```
    git clone https://github.com/IntelliBotix/ProbLO.git
    cd ~/catkin_ws/src
    cp -r /path/to/ProbLO/Probabilistic A-LOAM .
    cd ../
    catkin_make
    source ~/catkin_ws/devel/setup.bash
```

## 3. Build Probabilistic NDT
Clone the repository and catkin_make:
```
    git clone https://github.com/IntelliBotix/ProbLO.git
    cd ~/catkin_ws/src
    cp -r /path/to/ProbLO/Probabilistic NDT .
    cd ../
    catkin_make
    source ~/catkin_ws/devel/setup.bash
```



## 4. KITTI Example (Velodyne HDL-64)
Download [KITTI Odometry dataset](http://www.cvlibs.net/datasets/kitti/eval_odometry.php) to YOUR_DATASET_FOLDER. 

```
    roslaunch aloam_velodyne aloam_velodyne_HDL_64.launch
    roslaunch ndt_omp run.launch
```


## 5.Acknowledgements
Thanks for [A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM) and [ndt_omp](https://github.com/koide3/ndt_omp).
