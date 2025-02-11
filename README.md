# Funny-Lidar-SLAM

<img src="doc/a.png" alt="a" style="zoom:50%;" />

Demo video: [bilibili](https://www.bilibili.com/video/BV1KqtgeiEzD/?spm_id_from=333.999.0.0)

## 1.Introduction

A real-time multifunctional Lidar SLAM package. It has dual functions of Mapping and Localization. Supports multiple types of IMUs(6-axis and 9-axis) and Lidars(Velodyne, Livox Avia, Livox Mid 360, RoboSense, Ouster, etc). Supports various types of frontends, such as LOAM, NDT, ICP, etc. Supports multiple fusion methods, such as Kalman filter, optimization method, etc. Support multiple LoopClosure methods. The overall accuracy and efficiency are close to the current state-of-the-art methods.

- Supports multiple IMU types.
- Supports multiple fusion methods.
- Supports Mapping and Localization.
- Supports multiple LoopClosure methods.
- Supports multiple Lidar types, and customize Lidar type.
- Supports multiple pointcloud registration methods, and easily extendable to other registration methods.

|                   Features                   |                                                   Details                                                   |
| :------------------------------------------: | :---------------------------------------------------------------------------------------------------------: |
|                  IMU Types                   |                                               6-axis, 9-axis.                                               |
|                 Lidar Types                  | Solid State Lidar(Mid-360, Avia, etc), <br />Mechanical Lidar(Velodyne, Robosense, Ouster, etc),<br /> etc. |
|             Registration Methods             |        Loam Series, <br />Optimized-ICP, <br />Incremental-NDT, <br />even customizable via plug-in.        |
| LoopClosure Methods<br />(only Mapping mode) |                               Euclidean distance based, <br />Feature based.                                |
|                Fusion Methods                |                          Loose Coupling, <br />Kalman Filter, <br />Optimization.                           |

## 2.Prerequisites

### 2.1 Ubuntu and ROS2

**Ubuntu >= 20.04 && ROS Humble && C++17 Compiler**

For Ubuntu 20.04 or higher. After installing ROS, most of the dependent libraries have been installed, including the
PCL/Eigen/CMake/etc.

### 2.2 glog && gflag && gtest && g2o

```shell
sudo apt-get install libgoogle-glog-dev libgflags-dev libgtest-dev ros-humble-libg2o
```

### 2.3 livox_ros_driver2

Please follow the guidance of installation in the [livox_ros_driver2 Installation](https://github.com/Livox-SDK/livox_ros_driver2?tab=readme-ov-file).

## 3.How to Build?

### 3.1 Build from source (Recommend)

```shell
mkdir -p funny_lidar_slam_ws/src
cd funny_lidar_slam_ws/src
git clone -b humble https://github.com/yangfuyuan/funny_lidar_slam.git
cd ../
colcon build
source install/setup.bash
```

### 3.2 Build in Docker

If you want to use docker conatiner to run Funny Lidar SLAM, please install the docker on you machine. Follow [Docker Installation](https://docs.docker.com/engine/install/ubuntu/).

#### 3.2.1 Download project code to any path

```shell	
git clone -b humble https://github.com/yangfuyuan/funny_lidar_slam.git
```

#### 3.2.2 Build Docker image and create the container

```shell
cd funny_lidar_slam
docker build -f Dockerfile -t funny_lidar_slam:v0 .
cd docker
sudo chmod +x run.sh
bash run.sh
```

Tips:  

- When the above commands are executed correctly, you will enter the virtual terminal of the funny_lidar_slam container.  Refer to the following to perform Mapping and Localization.

- Please copy your bag data to the host's directory `funny_lidar_slam/data`. Then you can play them in the container.

## 4.How to run Mapping?

Funny-Lidar-SLAM is a multi-functional Lidar SLAM package. You can freely choose the Lidar model, 6-axis or 9-axis IMU ,
and registration method. To help you get started quickly, here I provide some of my examples on some open source
datasets. Of course you can also try other combinations. 

### 4.1 For Velodyne HDL-32E

- Dataset: [M2DGR](https://github.com/SJTU-ViSYS/M2DGR). You can also get part of the dataset from [Baidu Pan](https://pan.baidu.com/s/1KZZxavUrfRF5nqK4eeDVzw?pwd=yph8) for testing.

```shell
cd funny_lidar_slam_ws
source install/setup.bash
ros2 launch funny_lidar_slam mapping_M2DGR.launch.py

#rosbag play street_02.bag # Play one of the sets of data
```

- Dataset: [NCLT](https://robots.engin.umich.edu/nclt/index.html#download). You can also get part of the dataset from [Baidu Pan](https://pan.baidu.com/s/1Z3UzjP1FWJu9XSwujo_d5g?pwd=jggw) for testing.

```c++
cd funny_lidar_slam_ws
source install/setup.bash
ros2 launch funny_lidar_slam mapping_nclt.launch.py

#rosbag play 20130110.bag # Play one of the sets of data
```

### 4.2 For Velodyne HDL-16E

The test dataset comes from: [LIO-SAM dataset](https://github.com/TixiaoShan/LIO-SAM?tab=readme-ov-file#sample-datasets). You can also get part of the dataset from [Baidu Pan](https://pan.baidu.com/s/1PS7RkyNvbQNlpWaNDtQ1kQ?pwd=w3xe) for testing.

```shell
cd funny_lidar_slam_ws
source install/setup.bash
ros2 launch funny_lidar_slam mapping_lio_sam.launch.py

#rosbag play walking_dataset.bag # Play one of the sets of data
```

### 4.3 For Livox Mid-360

You can download the Livox Mid-360 data from: [Baidu Pan](https://pan.baidu.com/s/1Wi7s23r7TSE5wSyzrDaLNQ?pwd=th3i)

```shell
cd funny_lidar_slam_ws
source install/setup.bash
ros2 launch funny_lidar_slam mapping_mid360.launch.py

#rosbag play mid_360.bag # Play one of the sets of data
```

Note: If your Mid-360 format is `livox_ros_driver2/CustomMsg`, you can use `4.4 For Livox Avia` to run.

### 4.4 For Livox Avia

You can download the Livox Avia data from: [Baidu Pan](https://pan.baidu.com/s/1ZJ25shwYG66iU4jHzpCKyA?pwd=xy36). 

```shell
cd funny_lidar_slam_ws
source install/setup.bash
ros2 launch funny_lidar_slam mapping_livox_avia.launch.py

#rosbag play avia.bag  # Play one of the sets of data
```

Note: If your Livox Avia format is `sensor_msgs/PointCloud2`, you can use `4.3 For Livox Mid-360` to run.

### 4.5 For custom Lidar model (Only supports rotating mechanical Lidar)

You can download the data from: [Baidu Pan](https://pan.baidu.com/s/1rJiUiYibWmlmQ1xw_kOf7Q?pwd=dkpj)

```shell
cd funny_lidar_slam_ws
source install/setup.bash
ros2 launch funny_lidar_slam mapping_turing.launch.py

#rosbag play our.bag # Play one of the sets of data
```

### 4.6 Save PointCloud Map

```shell
cd funny_lidar_slam_ws
source install/setup.bash
ros2 service call /save_map funny_lidar_slam/srv/SaveMap "{map_path: '', split_map: false}"
```
Note: 
`map_path`: is the path where the map is stored. If `map_path` is empty, the default path `funny_lidar_slam/data` is used for saving. If you want to save the map to a specific path, set `map_path`. For example: `"map_path: '/home/xx/xxx.pcd'"`
`split_map`: Whether to divide the map into blocks.

## 5.How to run Localization?

### 5.1 Prepare pointcloud map

Copy your pointcloud map to the `funny_lidar_slam/data`  directory and change its name to `map.pcd`. If you don’t have a pointcloud map, you can generate one according to Section 4.

### 5.2 Run Localization

- For Livox Mid 360

```shell
ros2 launch funny_lidar_slam localization_mid_360.launch.py
```
- For Velodyne-32

```shell
ros2 launch funny_lidar_slam localization_nclt.launch.py
```

- For RoboSense-16

```shell
ros2 launch funny_lidar_slam localization_turing.launch.py
```

**Note**: Other Lidar models can be configured by referring to the above examples.

### 5.3 Set the initial pose

After running the Localization program, you need to play the bag and set the initial pose.

The localization program does not have a global relocalization function, so the initial position and orientation need to be set manually. For convenience, I use the `2D Pose Estimate` plugin in RVIZ to set the initial pose. If the initial pose effect is not good, you can try to use `2D Pose Estimate` multiple times to set the pose.

## 6.Some Tips

- If your IMU is 6-axis, and the fusion method is optimization, you need to ensure that your IMU and Lidar are stationary for at least two seconds before starting to run Mapping or Localization.
- When you run the Mapping program, the default Rviz configuration will be started. In order to reduce resource usage, the pointcloud map is displayed roughly. If you want to display richer details, pass the Rviz configuration path when running the Mapping launch file. For example: `ros2 launch funny_lidar_slam mapping_xxx.launch.py rviz_config:=/path/to/funny_lidar_slam/launch/display_detailed_without_loopclosure.rviz`
