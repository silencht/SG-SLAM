

# SG-SLAM

SLAM是智能移动机器人在未知环境中进行状态估计的基本能力之一。然而，大多数视觉SLAM系统**依赖于静态场景的假设**，因此在动态场景中的准确性和鲁棒性严重下降。此外，许多系统构建的度量图**缺乏语义信息**，因此机器人无法在人类的认知水平上理解他们的周围环境。

SG-SLAM是一个基于[ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2)框架的实时RGB-D语义视觉SLAM系统。首先，SG-SLAM增加了两个新的并行线程：一个获取2D语义信息的对象检测线程和一个语义建图线程。然后，在跟踪线程中加入了一个融合语义和几何信息的**快速动态特征剔除算法**。最后，在语义建图线程中生成3D点云和3D语义对象后，它们被发布到ROS系统中进行可视化。

在TUM数据集、波恩数据集和OpenLORIS-Scene数据集进行了实验评估，结果表明SG-SLAM不仅是动态场景中非常实时、准确、鲁棒的系统之一，而且还可以创建直观的语义对象度量地图。

![sg-slam](./doc/sg-slam-system-overview-zh.png)

**图1**. SG-SLAM系统框架。ORB-SLAM2的原始框架以水绿色背景呈现，新（或修改的）功能以红色背景呈现。

<img src="./doc/semantic-object-metric-map-zh.png" style="zoom: 25%;" />

**图2**.   tum rgbd dataset fr3/walking_xyz 序列的语义对象度量地图

<img src="./doc/octomap-zh.png" style="zoom: 15%;" />

**图3**.  tum rgbd dataset fr3/long office household 序列的八叉树地图

**系统特点 :**

- 基于ORB-SLAM2, NCNN,  ROS, etc.
- 实时  (如果NCNN配置好GPU CUDA加速的话)
- 较其他同类工作（可能更）容易配置和部署
- ...

## 1. License

SG-SLAM 基于协议 [GPLv3 license](https://github.com/silencht/SG-SLAM/blob/main/LICENSE).

论文可在此处下载（或本仓库doc） [IEEE Xplore](https://ieeexplore.ieee.org/abstract/document/9978699/).

如果是学术使用, 请引用（BibTex）:

```
@ARTICLE{9978699,
  author={Cheng, Shuhong and Sun, Changhe and Zhang, Shijun and Zhang, Dianfan},
  journal={IEEE Transactions on Instrumentation and Measurement}, 
  title={SG-SLAM: A Real-Time RGB-D Visual SLAM Toward Dynamic Scenes With Semantic and Geometric Information}, 
  year={2023},
  volume={72},
  number={},
  pages={1-12},
  doi={10.1109/TIM.2022.3228006}}
```

## 2. 编译配置 SG-SLAM

```bash
#最基本的
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

#OpenCV,可参考https://docs.opencv.org/3.4.15/d7/d9f/tutorial_linux_install.html
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
#中国大陆可换源,resource(http://wiki.ros.org/ROS/Installation/UbuntuMirrors)
sudo sh -c '. /etc/lsb-release && echo "deb http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'
#Set up keys
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
#this ros below is for ubuntu 18.04
sudo apt install ros-melodic-desktop-full
#环境变量配置
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
#依赖安装
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
#初始化rosdep.遇到错误可参考该文章,https://zhuanlan.zhihu.com/p/397966333
sudo apt install python-rosdep
sudo rosdep init
rosdep update
#test
roscore

#PCL,and pcl-tools(可选)
sudo apt install libpcl-dev pcl-tools

#Octomap,and octovis(可选)
sudo apt install liboctomap-dev octovis
sudo apt install ros-melodic-octomap ros-melodic-octomap-mapping ros-melodic-octomap-msgs ros-melodic-octomap-ros ros-melodic-octomap-rviz-plugins 

#SG-SLAM
git clone https://github.com/silencht/SG-SLAM

#编译第三方库：DBoW2、g2o、ncnn
cd SG-SLAM/src/sg-slam/
./ThirdpartyBuild.sh
#如何完整编译安装ncnn? 请参考ncnn仓库的README中的HowTo （https://github.com/Tencent/ncnn/README.md）
#安装完英伟达显卡驱动,vulkan and etc., 编译并安装ncnn
#如果出现编译错误，-DNCNN_DISABLE_RTTI=OFF (https://github.com/Tencent/ncnn/issues/2665)
cd SG-SLAM/src/sg-slam/Thirdparty/ncnn/
mkdir build
cd build
cmake -DCMAKE_TOOLCHAIN_FILE=../toolchains/host.gcc.toolchain.cmake -DNCNN_DISABLE_RTTI=OFF ..
#j后面的数字4可根据自己的CPU线程相应变大，进而提高编译速度
make -j4
sudo make install

#修改 SG-SLAM/src/sg-slam/CMakeLists.txt 中的路径，将该路径设置为你自己系统中的ncnn路径，提醒：该路径下包含ncnnConfig.cmake文件
#set(ncnn_DIR "there,replace with your path/SG-SLAM/src/sg-slam/Thirdparty/ncnn/build/install/lib/cmake/ncnn" CACHE PATH "Directory that contains ncnnConfig.cmake")

#初始化 ROS 工作空间然后依次编译各个功能包
#第一个包功能是将自己相机输出的ROS和OpenCV的话题消息格式转换
#第二个包功能是提供一系列图像几何处理方法
#第三个包功能是负责接收sg-slam发布的3D点云,将之转化为八叉树地图
#第四个包是SG-SLAM系统代码
cd SG-SLAM/src
catkin_init_workspace
cd ..
catkin_make --pkg cv_bridge
catkin_make --pkg image_geometry
catkin_make --pkg octomap_server
catkin_make --pkg sg-slam
```

## 3. 运行 SG-SLAM

将 [TUM dataset](https://vision.in.tum.de/data/datasets/rgbd-dataset/download) 数据集下载完成后放入home目录下的Music路径（该路径与 **run_tum_walking_xyz.sh** 文件调用路径一致，也可根据个人喜好随意设置。

```bash
#Runing SG-SLAM
#terminal 1
roscore
#terminal 2 移动到octomap_server功能包下的launch目录下，该目录下有两个launch文件（octomap.launch，transform.launch），运行之。八叉树建图的各功能参数可在此配置
cd SG-SLAM/src/octomap_server/launch
roslaunch octomap.launch
#terminal 3
roslaunch transform.launch
#terminal 4，打开rviz接收话题显示地图，默认配置文件在SG-SLAM\src\sg-slam\Examplesrvizconfig.rviz，可根据程序、参数配置进行相应调整
rviz
#terminal 5，运行tum数据集的walking_xyz序列。另外，也可运行硬件相机，如文件run_astra_pro_camera.sh
#相机参数配置yaml文件最后一行PointCloudMapping.Resolution: 0.01参数意义为对点云进行体素滤波的分辨率值
cd SG-SLAM/src/sg-slam/
./run_tum_walking_xyz.sh
```

## 4. 参考过的仓库

**包括但不限于以下仓库  (排列顺序无意义) : **

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

## 5. 其他

- 待补充
