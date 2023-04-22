/*
 * <SG-SLAM>
 * Copyright (C) 2016  <copyright holder> <email>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/*
 *--------------------------------------------------------------------------------------------------
 * DS-SLAM: A Semantic Visual SLAM towards Dynamic Environments
　*　Author(s):
 * Chao Yu, Zuxin Liu, Xinjun Liu, Fugui Xie, Yi Yang, Qi Wei, Fei Qiao qiaofei@mail.tsinghua.edu.cn
 * Created by Yu Chao@2018.12.03
 * --------------------------------------------------------------------------------------------------
 * DS-SLAM is a optimized SLAM system based on the famous ORB-SLAM2. If you haven't learn ORB_SLAM2 code, 
 * you'd better to be familiar with ORB_SLAM2 project first. Compared to ORB_SLAM2, 
 * we add anther two threads including semantic segmentation thread and densemap creation thread. 
 * You should pay attention to Frame.cc, ORBmatcher.cc, Pointcloudmapping.cc and Segment.cc.
 * 
 *　@article{murORB2,
 *　title={{ORB-SLAM2}: an Open-Source {SLAM} System for Monocular, Stereo and {RGB-D} Cameras},
　*　author={Mur-Artal, Ra\'ul and Tard\'os, Juan D.},
　* journal={IEEE Transactions on Robotics},
　*　volume={33},
　* number={5},
　* pages={1255--1262},
　* doi = {10.1109/TRO.2017.2705103},
　* year={2017}
 *　}
 * --------------------------------------------------------------------------------------------------
 * Copyright (C) 2018, iVip Lab @ EE, THU (https://ivip-tsinghua.github.io/iViP-Homepage/) and 
 * Advanced Mechanism and Roboticized Equipment Lab. All rights reserved.
 *
 * Licensed under the GPLv3 License;
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * https://github.com/ivipsourcecode/DS-SLAM/blob/master/LICENSE
 *--------------------------------------------------------------------------------------------------
 */
#ifndef POINTCLOUDMAPPING_H
#define POINTCLOUDMAPPING_H

#include "System.h"
#include <condition_variable>
#include <opencv2/core/core.hpp>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include "Detector3D.h"

using namespace ORB_SLAM2;
class Detector3D;

class PointCloudMapping
{

public:
    PointCloudMapping(int is_global_pc_reconstruction_,int is_octo_semantic_map_construction_,
        int is_map_construction_consider_dynamic_,float camera_valid_depth_Min_,
        float camera_valid_depth_Max_,int Sor_Local_MeanK_,double Sor_Local_StddevMulThresh_,float Voxel_Local_LeafSize_,
        int Sor_Global_MeanK_,double Sor_Global_StddevMulThresh_,float Voxel_Global_LeafSize_,int Detect3D_Sor_MeanK_,
        double Detect3D_Sor_StddevMulThresh_,float Detect3D_Voxel_LeafSize_,float Detect3D_EuclideanClusterTolerance_,
        int Detect3D_EuclideanClusterMinSize_,int Detect3D_EuclideanClusterMaxSize_,float Detect3D_DetectSimilarCompareRatio_,
        int global_pc_update_kf_threshold);
    ~PointCloudMapping();

    // Inserting a keyframe updates the map once
    void insertKeyFrame(KeyFrame* pkf,cv::Mat& color, cv::Mat& depth);
    void shutdown();
    void MapViewer();
    inline pcl::PointCloud<pcl::PointXYZRGB> get_globalMap(){return *globalMap;};
    Detector3D* mpDetector3D;

protected:
    void generatePointCloud(KeyFrame* kf ,pcl::PointCloud<pcl::PointXYZRGB>::Ptr camera_pc_,
                                          pcl::PointCloud<pcl::PointXYZRGB>::Ptr world_pc_);
    void generatePointCloudForDyamic(KeyFrame* kf ,pcl::PointCloud<pcl::PointXYZRGB>::Ptr camera_pc_,
                                          pcl::PointCloud<pcl::PointXYZRGB>::Ptr world_pc_,
                                          pcl::PointCloud<pcl::PointXYZRGB>::Ptr semanticobj_pc_);                               
    void settingTextMarkerBasicParameter(double scale);
    void settingCubeMarkerBasicParameter();
    enum Color {Red,Blue,Green,Yellow};
    void settingMarkerColor(Color color);
    bool CheckNewKeyFrames();
    int  NewKeyFramesSize();
    void slam_to_ros_mode_transform(pcl::PointCloud<pcl::PointXYZRGB>& source, pcl::PointCloud<pcl::PointXYZRGB>& out);
    bool isInDynamicRegion(int x,int y,std::vector<cv::Rect_<float> >& vDynamicBorder_);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr semantic_objMap;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr localMap;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr globalMap;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_globalMap;

    boost::shared_ptr<thread>  viewerThread;

    bool  shutDownFlag = false;
    bool  is_global_pc_reconstruction = false;
    bool  is_octo_semantic_map_construction = false;
    bool  is_map_construction_consider_dynamic = false;

    std::mutex   shutDownMutex;

    std::mutex               keyFrameUpdateMutex;

    // Data to generate point clouds
    KeyFrame*                    mpCurrentkeyFrame;
    std::list<KeyFrame*>         mlNewKeyFrames;
    std::mutex                   keyframeMutex;

    pcl::VoxelGrid<pcl::PointXYZRGB>  voxel_local;
    pcl::VoxelGrid<pcl::PointXYZRGB>  voxel_global;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor_local;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor_global;

    tf::Transform camera_to_map_tf;  
    tf::TransformBroadcaster *camera_to_map_tfbroadcaster;
    // ros::Publisher semantic_pcl_publisher;
    ros::Publisher local_pcl_publisher;
    ros::Publisher global_pcl_publisher;
    ros::Publisher marker_publisher;
    
    sensor_msgs::PointCloud2 localMap_pcl_to_publish;
    sensor_msgs::PointCloud2 globalMap_pcl_to_publish;

    visualization_msgs::Marker cube_marker_to_publish;
    visualization_msgs::Marker text_marker_to_publish;

    uint16_t SemanticObject_Count = 0;
    float camera_valid_depth_Min = 0.5;
    float camera_valid_depth_Max = 5.0;
    int global_pc_update_kf_threshold = 20;


};

#endif // POINTCLOUDMAPPING_H
