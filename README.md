

# SG-SLAM

Simultaneous Localization and Mapping (SLAM) is one of the fundamental capabilities for intelligent mobile robots to perform state estimation in unknown environments. However, most visual SLAM systems **rely on the static scene assumption** and consequently have severely reduced accuracy and robustness in dynamic scenes. Moreover, the metric maps constructed by many systems **lack semantic information**, so the robots cannot understand their surroundings at a human cognitive level. 

In this paper, we propose SG-SLAM, which is a real-time RGB-D semantic visual SLAM system based on the [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2) framework. First, SG-SLAM adds two new parallel threads: an object detecting thread to obtain 2D semantic information and a semantic mapping thread. Then, a **fast dynamic feature rejection algorithm combining semantic and geometric information** is added to the tracking thread. Finally, they are published to the ROS system for visualization after generating 3D point clouds and 3D semantic objects in the **semantic mapping thread**. 

We performed an experimental evaluation on the TUM dataset, the Bonn dataset, and the OpenLORIS-Scene dataset. The results show that SG-SLAM is not only one of the most real-time, accurate, and robust systems in dynamic scenes, but also allows the creation of intuitive semantic metric maps.

（For the Chinese version, see the README-zh.md file in the directory）

（**中文版本见目录下README-zh.md文件**）



![sg-slam](./doc/sg-slam-system-overview.png)

**Figure1**. Overview of the framework of the SG-SLAM system. The original work of ORB-SLAM2 is presented on an aqua-green background, while **our main new or modified work is presented on a red background**.

<img src="./doc/semantic-object-metric-map.png" style="zoom: 25%;" />

**Figure2**.  Semantic object metric map for tum rgbd dataset fr3/walking_xyz sequence

<img src="./doc/octomap.png" style="zoom: 15%;" />

**Figure3**.  octo map for tum rgbd dataset fr3/long office household sequence

<img src="./doc/realmap.png" style="zoom: 15%;" />

**Figure4**.  Actual effect

**System Features :**

- Based on ORB-SLAM2, NCNN,  ROS, etc.
- Fast running  (if NCNN is well configured)
- Easy to deploy
- ...

## 1. License

SG-SLAM is released under a [GPLv3 license](https://github.com/silencht/SG-SLAM/blob/main/LICENSE).

Paper is available on [IEEE Xplore](https://ieeexplore.ieee.org/abstract/document/9978699/).

If you use SG-SLAM in an academic work, please cite:

```
S. Cheng, C. Sun, S. Zhang and D. Zhang, "SG-SLAM: A Real-Time RGB-D Visual SLAM toward Dynamic Scenes with Semantic and Geometric Information," in IEEE Transactions on Instrumentation and Measurement, doi: 10.1109/TIM.2022.3228006.
```

## 2. Building SG-SLAM

```bash
#Basic
sudo apt-get update
sudo apt install git
sudo apt install cmake
sudo apt install build-essential
sudo apt vim
#test
git --version
gcc --version
g++ --version
cmake --version

#Pangolin
sudo apt install libglew-dev
sudo apt install libboost-dev libboost-thread-dev libboost-filesystem-dev
sudo apt install libpython2.7-dev
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin/
git checkout v0.5
mkdir build
cd build
cmake ..
make -j4
sudo make install

#OpenCV,refer https://docs.opencv.org/3.4.15/d7/d9f/tutorial_linux_install.html
sudo apt install libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
sudo apt install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev
git clone https://github.com/opencv/opencv.git
cd opencv/
git checkout 3.4.15
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local ..
sudo make install
#test
pkg-config opencv --modversion

#Eigen
git clone https://gitlab.com/libeigen/eigen.git
cd eigen/
git checkout 3.1.0
mkdir build
cd build
cmake ..
sudo make install
#test
cat /usr/local/include/eigen3/Eigen/src/Core/util/Macros.h
#Compile orb-slam2 with build.sh,and now orbslam2 can perform well

#ROS
#For china,first,replace our resource(http://wiki.ros.org/ROS/Installation/UbuntuMirrors)
sudo sh -c '. /etc/lsb-release && echo "deb http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'
#Set up keys
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
#this ros below is for ubuntu 18.04
sudo apt install ros-melodic-desktop-full
#Environment setup
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
#Dependencies for building packages
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
#Initialize rosdep.If you encounter problems, refer to this article,https://zhuanlan.zhihu.com/p/397966333
sudo apt install python-rosdep
sudo rosdep init
rosdep update
#test
roscore

#PCL,and pcl-tools(optional)
sudo apt install libpcl-dev pcl-tools

#Octomap,and octovis(optional)
sudo apt install liboctomap-dev octovis
sudo apt install ros-melodic-octomap ros-melodic-octomap-mapping ros-melodic-octomap-msgs ros-melodic-octomap-ros ros-melodic-octomap-rviz-plugins 

#SG-SLAM
git clone https://github.com/silencht/SG-SLAM

#Build Thirdparty Liarbry
cd SG-SLAM/src/sg-slam/
./ThirdpartyBuild.sh
#How to build ncnn completely? Please refer HowTo in https://github.com/Tencent/ncnn/README.md
#After installing the nvidia driver,vulkan and etc., compile ncnn and install
#-DNCNN_DISABLE_RTTI=OFF (https://github.com/Tencent/ncnn/issues/2665)
cd SG-SLAM/src/sg-slam/Thirdparty/ncnn/
mkdir build
cd build
cmake -DCMAKE_TOOLCHAIN_FILE=../toolchains/host.gcc.toolchain.cmake -DNCNN_DISABLE_RTTI=OFF ..
make -j4
sudo make install

#Modify the path in SG-SLAM/src/sg-slam/CMakeLists.txt
#set(ncnn_DIR "there,replace with your path/SG-SLAM/src/sg-slam/Thirdparty/ncnn/build/install/lib/cmake/ncnn" CACHE PATH "Directory that contains ncnnConfig.cmake")

#Init ROS workspace and compile package
cd SG-SLAM/src
catkin_init_workspace
cd ..
catkin_make --pkg cv_bridge
catkin_make --pkg image_geometry
catkin_make --pkg octomap_server
catkin_make --pkg sg-slam
```

## 3. Running SG-SLAM

Put the [TUM dataset](https://vision.in.tum.de/data/datasets/rgbd-dataset/download) into the *Music Path* according to the **run_tum_walking_xyz.sh** file (or adjust the script yourself)

```bash
#Runing SG-SLAM
#terminal 1
roscore
#terminal 2
cd SG-SLAM/src/octomap_server/launch
roslaunch octomap.launch
#terminal 3
roslaunch transform.launch
#terminal 4，you can use my rviz configuration file（like the command below）, its path is located in SG-SLAM/src/sg-slam/Examples/rvizconfig.rviz, This will subscribe to some map-published topics directly in rviz.
#Of course, you can also open rviz directly, and then manually subscribe to the related topic.
rviz -d SG-SLAM/src/sg-slam/Examples/rvizconfig.rviz
#terminal 5
cd SG-SLAM/src/sg-slam/
./run_tum_walking_xyz.sh
```

## 4. Reference Project

**Including but not limited to the following repositories  (In no particular order)**

- https://github.com/raulmur/ORB_SLAM2
- https://github.com/ivipsourcecode/DS-SLAM
- https://github.com/Ewenwan/ORB_SLAM2_SSD_Semantic
- https://github.com/MRwangmaomao/semantic_slam_nav_ros
- https://github.com/gaoxiang12/ORBSLAM2_with_pointcloud_map
- https://github.com/abhineet123/ORB_SLAM2
- https://github.com/floatlazer/semantic_slam
- https://github.com/bijustin/YOLO-DynaSLAM
- https://github.com/bijustin/Fast-Dynamic-ORB-SLAM
- https://github.com/halajun/VDO_SLAM
- https://github.com/Quitino/IndoorMapping
- ...

## 5. Other

### 5.1 octomap_server

/SG-SLAM/src/octomap_server/launch/octomap.launch

This file is used to start the octomap_server node and configure some parameters. You can see what these parameters mean here(http://wiki.ros.org/octomap_server、https://octomap.github.io/)

Among them, the **param name=resolution** parameter indicates the voxel resolution of the octomap. The smaller the parameter, the finer the map voxel segmentation and the higher the resolution. But the processing time and computational complexity also increase.

The **occupancy_min_z** and **occupancy_max_z** parameters can selectively pass through the point cloud within the z-axis range. If your initial camera view is parallel to the ground, you can also use the occupancy_min_z parameter to do a trick to filter out the ground. Similarly, the occupancy_min_z parameter can be used to filter out the top voxels of the house.

**filter_ground** is the normal algorithm for filtering out the ground (not the trick of directly using occupancy_min_z to filter out the ground). Usage can refer to the above URL.

```xml
<!-- 
  Example launch file for octomap_server mapping: 
-->
<launch>
	<node pkg="octomap_server" type="octomap_server_node" name="octomap_server">
		<remap from="cloud_in" to="/SG_SLAM/Point_Clouds" />
		<param name="frame_id" type="string" value="/map" />
		<param name="resolution" value="0.05" />
        <param name="sensor_model/hit" value="0.7" />
        <param name="sensor_model/miss" value="0.4" />
		<param name="sensor_model/max" value="0.99" />
		<param name="sensor_model/min" value="0.12" />
		<param name="sensor_model/max_range" value="-1.0" /> 
		<param name="height_map" type="bool" value="false" />
		<param name="colored_map" type="bool" value="true" />
		<param name="latch" type="bool" value="false" />
		<param name="occupancy_min_z" type="double" value="-1.5" />
		<param name="occupancy_max_z" type="double" value="1.5" />

		<param name="filter_ground" type="bool" value="false" />
		<param name="base_frame_id" type="string" value="/map" />

		<param name="filter_speckles" type="bool" value="true" />
		<param name="ground_filter/distance" type="double" value="0.05" />    
		<param name="ground_filter/angle" type="double" value="0.15" />        
		<param name="ground_filter/plane_distance" type="double" value="0.05" /> 
		<param name="pointcloud_min_z" type="double" value="-5.0" />
		<param name="pointcloud_max_z" type="double" value="5.0" />
	</node>
</launch>
```

### 5.2 camera.yaml

SG-SLAM/src/sg-slam/Examples/astra_pro_camera.yaml

SG-SLAM/src/sg-slam/Examples/TUM1.yaml

……

The last line in the camera configuration parameter file also has a resolution parameter **PointCloudMapping.Resolution**

It can be seen from the code when System.cc reads the yaml configuration file that this parameter is finally passed to the Voxel filter object constructor in the **PointCloudMapping** class.

This parameter is finally passed to the **voxel filter object**, which is the resolution used for voxel filtering on the point cloud after the depth map is transformed into a 3D point cloud. Because the number of 3D point clouds directly converted from the depth map is large (640*480), the calculation burden is heavy, so filtering is performed. The resolution of the voxel filter is similar to that of the octomap, that is, the centroid of the same position is replaced, so that the amount of calculation is reduced. 

After my test, it is generally set to 0.01 on my device, and the calculation efficiency and effect have reached a good balance. Adjustable to individual hardware.

```yaml
%YAML:1.0
#--------------------------------------------------------------------------------------------
# Camera Parameters. Adjust them!
#--------------------------------------------------------------------------------------------
# Camera calibration and distortion parameters (OpenCV) 
Camera.fx: 575.520619
Camera.fy: 575.994771
…………omitted here

#--------------------------------------------------------------------------------------------
# Viewer Parameters
#--------------------------------------------------------------------------------------------
Viewer.KeyFrameSize: 0.05
…………omitted here

PointCloudMapping.Resolution: 0.01
```

### 5.3 Object Position

/SG-SLAM/src/octomap_server/launch/octomap.launch

```xml
<!-- 
  Example launch file for octomap_server mapping: 
  Listens to incoming PointCloud2 data and incrementally builds an octomap. 
  The data is sent out in different representations. 
	RED：X GREEN：Y BLUE：Z
-->
<launch>	
	<node pkg="tf" type="static_transform_publisher" name="map" args="0 0 0 0 0 0 /map /pointCloud 70" />​
</launch>
```

SG-SLAM/src/sg-slam/src/pointcloudmapping.cc

```c++
void PointCloudMapping::MapViewer()
{
    std::cout<<"start viewer."<< std::endl;
    ros::NodeHandle nh;
    pcl_publisher = nh.advertise<sensor_msgs::PointCloud2>("/SG_SLAM/Point_Clouds",100);
    marker_publisher= nh.advertise<visualization_msgs::Marker>("/SG_SLAM/Semantic_Objects",100);
//…………omitted here
// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
            cube_marker_to_publish.pose.position.x = mpDetector3D->mpObjectDatabase->mvSemanticObject[id].centroid[2];
            cube_marker_to_publish.pose.position.y = -mpDetector3D->mpObjectDatabase->mvSemanticObject[id].centroid[0];
            cube_marker_to_publish.pose.position.z = mpDetector3D->mpObjectDatabase->mvSemanticObject[id].centroid[1]+0.8;
//…………omitted here
            text_marker_to_publish.pose.position.x = mpDetector3D->mpObjectDatabase->mvSemanticObject[id].centroid[2];
            text_marker_to_publish.pose.position.y = -mpDetector3D->mpObjectDatabase->mvSemanticObject[id].centroid[0];
            text_marker_to_publish.pose.position.z = mpDetector3D->mpObjectDatabase->mvSemanticObject[id].centroid[1]+0.8;
```

The parameters of the two files here are for adjusting the display effect of the map, and they are all static transformations.

For example "text_marker_to_publish.pose.position.z = mpDetector3D->mpObjectDatabase->mvSemanticObject[id].centroid[1]+0.8;" and "cube_marker_to_publish.pose.position.z = mpDetector3D->mpObjectDatabase->mvSemanticObject[id]. centroid[1]+0.8; "

**The +0.8 in the code** means that the coordinates of the 3D object to be released are consistent with the rviz octree map coordinates. The parameter 0.8 is because my camera's initial pose is 0.8m above the ground

You can adjust this parameter to test the effect yourself. In order to understand its role and change the parameters that suit you.

### 5.4 dynamic feature processing code

The code is located in the RmDynamicPointWithMultiviewGeometry function of Frame.cc

