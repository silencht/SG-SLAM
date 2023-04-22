#include "PointcloudMapping.h"

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "Converter.h"
#include <pcl/visualization/cloud_viewer.h>


PointCloudMapping::PointCloudMapping(int is_global_pc_reconstruction_,int is_octo_semantic_map_construction_,
int is_map_construction_consider_dynamic_, float camera_valid_depth_Min_,
float camera_valid_depth_Max_,int Sor_Local_MeanK_,double Sor_Local_StddevMulThresh_,float Voxel_Local_LeafSize_,
int Sor_Global_MeanK_,double Sor_Global_StddevMulThresh_,float Voxel_Global_LeafSize_,int Detect3D_Sor_MeanK_,
double Detect3D_Sor_StddevMulThresh_,float Detect3D_Voxel_LeafSize_,float Detect3D_EuclideanClusterTolerance_,
int Detect3D_EuclideanClusterMinSize_,int Detect3D_EuclideanClusterMaxSize_,float Detect3D_DetectSimilarCompareRatio_,
int global_pc_update_kf_threshold_):
is_global_pc_reconstruction(is_global_pc_reconstruction_),
is_octo_semantic_map_construction(is_octo_semantic_map_construction_),
is_map_construction_consider_dynamic(is_map_construction_consider_dynamic_),
camera_valid_depth_Min(camera_valid_depth_Min_),
camera_valid_depth_Max(camera_valid_depth_Max_),
global_pc_update_kf_threshold(global_pc_update_kf_threshold_)
{
    semantic_objMap = boost::make_shared< pcl::PointCloud<pcl::PointXYZRGB> >( );
    localMap        = boost::make_shared< pcl::PointCloud<pcl::PointXYZRGB> >( );
    temp_globalMap  = boost::make_shared< pcl::PointCloud<pcl::PointXYZRGB> >( );

    if(is_global_pc_reconstruction)
    {
        globalMap  = boost::make_shared< pcl::PointCloud<pcl::PointXYZRGB> >( );
        voxel_global.setLeafSize(Voxel_Global_LeafSize_,Voxel_Global_LeafSize_,Voxel_Global_LeafSize_);
        sor_global.setMeanK(Sor_Global_MeanK_);
        sor_global.setStddevMulThresh(Sor_Global_StddevMulThresh_);
    }

    if(is_octo_semantic_map_construction)
    {
        mpDetector3D = new Detector3D(Detect3D_Sor_MeanK_,Detect3D_Sor_StddevMulThresh_,Detect3D_Voxel_LeafSize_,
                                  Detect3D_EuclideanClusterTolerance_,Detect3D_EuclideanClusterMinSize_,
                                  Detect3D_EuclideanClusterMaxSize_,Detect3D_DetectSimilarCompareRatio_);
        voxel_local.setLeafSize(Voxel_Local_LeafSize_, Voxel_Local_LeafSize_, Voxel_Local_LeafSize_);
        sor_local.setMeanK(Sor_Local_MeanK_);
        sor_local.setStddevMulThresh(Sor_Local_StddevMulThresh_);
    }
    
    viewerThread = boost::make_shared<thread>(bind(&PointCloudMapping::MapViewer,this));
}

PointCloudMapping::~PointCloudMapping()
{
    delete mpDetector3D;
}

void PointCloudMapping::insertKeyFrame(KeyFrame* pkf, cv::Mat& color, cv::Mat& depth)
{
    unique_lock<mutex> lck(keyframeMutex);
    mlNewKeyFrames.push_back(pkf);
    pkf->ros_time = ros::Time::now();
    color.copyTo(pkf->mImRGB);
    depth.copyTo(pkf->mImDep);
}

bool PointCloudMapping::CheckNewKeyFrames()
{
    unique_lock<mutex> lock(keyframeMutex);
    return(!mlNewKeyFrames.empty());
}

//considering dynamic objects version
void PointCloudMapping::generatePointCloudForDyamic(KeyFrame* pkf ,pcl::PointCloud<pcl::PointXYZRGB>::Ptr camera_pc_ ,
pcl::PointCloud<pcl::PointXYZRGB>::Ptr world_pc_ ,pcl::PointCloud<pcl::PointXYZRGB>::Ptr semanticobj_pc_)
{
    camera_pc_->resize(pkf->mImDep.rows * pkf->mImDep.cols);
    camera_pc_->width = pkf->mImDep.cols;
    camera_pc_->height = pkf->mImDep.rows;
    camera_pc_->is_dense = false;//false if have points are invalid (e.g., have NaN or Inf values).

    if(pkf->mbHaveDynamicObject)
    {
        semanticobj_pc_->resize(pkf->mImDep.rows * pkf->mImDep.cols);
        semanticobj_pc_->width = pkf->mImDep.cols;
        semanticobj_pc_->height = pkf->mImDep.rows;
        semanticobj_pc_->is_dense = false;
        std::vector<cv::Rect_<float> > vDynamicBorder;

        for(unsigned int i = 0; i < pkf->mvObjects2D.size(); i++)
        {
            if(15 == pkf->mvObjects2D[i].id)//is people
            {
                cv::Rect_<float> rect2d = pkf->mvObjects2D[i].rect;
                vDynamicBorder.emplace_back(rect2d);
            }
        }
        for (int m = 0; m < pkf->mImDep.rows; m++)
        {
            for (int n = 0; n < pkf->mImDep.cols; n++)
            {
                float d = pkf->mImDep.ptr<float>(m)[n];
                if(d < camera_valid_depth_Min || d > camera_valid_depth_Max || isnan(d)) continue;
                //notice: x is col, y is row,so there is (n,m),and not (m,n)
                //if this point is in dynamic boundingbox
                size_t index = m * pkf->mImDep.cols + n;
                if(isInDynamicRegion(n,m,vDynamicBorder))
                {
                    semanticobj_pc_->points[index].z = d;
                    semanticobj_pc_->points[index].x = ( n - pkf->cx) * d / pkf->fx;
                    semanticobj_pc_->points[index].y = ( m - pkf->cy) * d / pkf->fy;
                    semanticobj_pc_->points[index].r = pkf->mImRGB.ptr<uchar>(m)[n*3+2];
                    semanticobj_pc_->points[index].g = pkf->mImRGB.ptr<uchar>(m)[n*3+1];
                    semanticobj_pc_->points[index].b = pkf->mImRGB.ptr<uchar>(m)[n*3+0];
                }
                else
                {
                    semanticobj_pc_->points[index].z = d;
                    semanticobj_pc_->points[index].x = ( n - pkf->cx) * d / pkf->fx;
                    semanticobj_pc_->points[index].y = ( m - pkf->cy) * d / pkf->fy;
                    semanticobj_pc_->points[index].r = pkf->mImRGB.ptr<uchar>(m)[n*3+2];
                    semanticobj_pc_->points[index].g = pkf->mImRGB.ptr<uchar>(m)[n*3+1];
                    semanticobj_pc_->points[index].b = pkf->mImRGB.ptr<uchar>(m)[n*3+0];
                    camera_pc_->points[index].z = semanticobj_pc_->points[index].z;
                    camera_pc_->points[index].x = semanticobj_pc_->points[index].x;
                    camera_pc_->points[index].y = semanticobj_pc_->points[index].y;
                    camera_pc_->points[index].r = semanticobj_pc_->points[index].r;
                    camera_pc_->points[index].g = semanticobj_pc_->points[index].g;
                    camera_pc_->points[index].b = semanticobj_pc_->points[index].b;
                }
            }
        }
    }
    else
    {
        for (int m=0; m<pkf->mImDep.rows; m++)
        {
            for (int n=0; n<pkf->mImDep.cols; n++)
            {
                float d = pkf->mImDep.ptr<float>(m)[n];
                if(d < camera_valid_depth_Min || d > camera_valid_depth_Max || isnan(d)) continue;

                size_t index = m * pkf->mImDep.cols + n;
                camera_pc_->points[index].z = d;
                camera_pc_->points[index].x = ( n - pkf->cx) * d / pkf->fx;
                camera_pc_->points[index].y = ( m - pkf->cy) * d / pkf->fy;
                camera_pc_->points[index].r = pkf->mImRGB.ptr<uchar>(m)[n*3+2];
                camera_pc_->points[index].g = pkf->mImRGB.ptr<uchar>(m)[n*3+1];
                camera_pc_->points[index].b = pkf->mImRGB.ptr<uchar>(m)[n*3+0];
            }
        }
    }

    //P_{world) = Twc * P_{camera}
    Eigen::Isometry3d Twc = ORB_SLAM2::Converter::toSE3Quat( pkf->GetPoseInverse() );
    pcl::transformPointCloud(*camera_pc_, *world_pc_, Twc.matrix());

    //3D Semantic Object Detect
    if(pkf->mvObjects2D.size() > 0 && is_octo_semantic_map_construction && pkf->mbHaveDynamicObject)
    {
        pcl::transformPointCloud(*semanticobj_pc_, *semanticobj_pc_, Twc.matrix());
        mpDetector3D->Detect(pkf->mvObjects2D,pkf->mImDep,semanticobj_pc_);
    }   
    else if(pkf->mvObjects2D.size() > 0 && is_octo_semantic_map_construction && !pkf->mbHaveDynamicObject)
        mpDetector3D->Detect(pkf->mvObjects2D,pkf->mImDep,world_pc_);

    //3D Global point cloud reconstrution
    if(is_global_pc_reconstruction)
        slam_to_ros_mode_transform(*world_pc_, *world_pc_);
}
    

// Not considering dynamic objects version
void PointCloudMapping::generatePointCloud(KeyFrame* pkf ,pcl::PointCloud<pcl::PointXYZRGB>::Ptr camera_pc_ ,
pcl::PointCloud<pcl::PointXYZRGB>::Ptr world_pc_)
{
    camera_pc_->resize(pkf->mImDep.rows * pkf->mImDep.cols);
    camera_pc_->width = pkf->mImDep.cols;
    camera_pc_->height = pkf->mImDep.rows;
    camera_pc_->is_dense = false;//false if have points are invalid (e.g., have NaN or Inf values).

    for ( int m=0; m<pkf->mImDep.rows; m+=1)
    {
        for ( int n=0; n<pkf->mImDep.cols; n+=1)
        {
            float d = pkf->mImDep.ptr<float>(m)[n];
            if(d < camera_valid_depth_Min || d > camera_valid_depth_Max || isnan(d)) continue;

            size_t index = m * pkf->mImDep.cols + n;
            camera_pc_->points[index].z = d;
            camera_pc_->points[index].x = ( n - pkf->cx) * d / pkf->fx;
            camera_pc_->points[index].y = ( m - pkf->cy) * d / pkf->fy;
            camera_pc_->points[index].r = pkf->mImRGB.ptr<uchar>(m)[n*3+2];
            camera_pc_->points[index].g = pkf->mImRGB.ptr<uchar>(m)[n*3+1];
            camera_pc_->points[index].b = pkf->mImRGB.ptr<uchar>(m)[n*3+0];
        }
    }
    //P_{world) = Twc * P_{camera}
    Eigen::Isometry3d Twc = ORB_SLAM2::Converter::toSE3Quat( pkf->GetPoseInverse() );
    pcl::transformPointCloud(*camera_pc_, *world_pc_, Twc.matrix());

    //3D Semantic Object Detect
    if(pkf->mvObjects2D.size() > 0 && is_octo_semantic_map_construction)
        mpDetector3D->Detect(pkf->mvObjects2D,pkf->mImDep,world_pc_);

    if(is_global_pc_reconstruction)
        slam_to_ros_mode_transform(*world_pc_, *world_pc_);
}


void PointCloudMapping::MapViewer()
{
    std::cout<<"Start PointCloudMapping Viewer..."<< std::endl;
    ros::NodeHandle nh;
    cv::Mat Keyframe_Pose;


    // semantic_pcl_publisher = nh.advertise<sensor_msgs::PointCloud2>("/SG_SLAM/Semantic_Point_Clouds",10);
    if(is_octo_semantic_map_construction)
    {
        local_pcl_publisher = nh.advertise<sensor_msgs::PointCloud2>("/SG_SLAM/Local_Point_Clouds",10);
        marker_publisher= nh.advertise<visualization_msgs::Marker>("/SG_SLAM/Semantic_Objects",10);
        settingTextMarkerBasicParameter(0.2);
        settingCubeMarkerBasicParameter();
    }
    if(is_global_pc_reconstruction) 
        global_pcl_publisher = nh.advertise<sensor_msgs::PointCloud2>("/SG_SLAM/Global_Point_Clouds",10);

    ros::Rate r(50);
    while(1)
    {
        {
            unique_lock<mutex> lck_shutdown( shutDownMutex );
            if (shutDownFlag && !CheckNewKeyFrames())
            {
                break;
            }
        }
        if(CheckNewKeyFrames())
        {
            {
                unique_lock<mutex> lock(keyframeMutex);
                mpCurrentkeyFrame = mlNewKeyFrames.front();
                mlNewKeyFrames.pop_front();
            }

            localMap->clear();
            temp_globalMap->clear();
            if(is_map_construction_consider_dynamic)
            {
                semantic_objMap->clear();
                generatePointCloudForDyamic(mpCurrentkeyFrame,localMap,temp_globalMap,semantic_objMap);
            }
            else
                generatePointCloud(mpCurrentkeyFrame,localMap,temp_globalMap);

            Keyframe_Pose = mpCurrentkeyFrame->GetPose();
        }
        else
        {
            usleep(10);
            continue;
        }


        if(is_octo_semantic_map_construction && !localMap->empty())
        {
            //compute Transform "camera_to_map (ros mode)" for octomap_server
            cv::Mat Rwc(3,3,CV_32F);
	        cv::Mat twc(3,1,CV_32F);
            Eigen::Matrix<double,3,3> RotationMat;
            Rwc = Keyframe_Pose.rowRange(0,3).colRange(0,3).t();
            twc = -Rwc * Keyframe_Pose.rowRange(0,3).col(3);
            RotationMat << Rwc.at<float>(0,0), Rwc.at<float>(0,1), Rwc.at<float>(0,2),
                           Rwc.at<float>(1,0), Rwc.at<float>(1,1), Rwc.at<float>(1,2),
                           Rwc.at<float>(2,0), Rwc.at<float>(2,1), Rwc.at<float>(2,2);
            Eigen::Quaterniond Q(RotationMat);
            camera_to_map_tf.setOrigin(tf::Vector3(twc.at<float>(2), -twc.at<float>(0), -twc.at<float>(1)));
            camera_to_map_tf.setRotation(tf::Quaternion(Q.z(), -Q.x(), -Q.y(), Q.w()));

            //filter
            voxel_local.setInputCloud(localMap);
            voxel_local.filter(*localMap);
            sor_local.setInputCloud(localMap);
            sor_local.filter(*localMap); //slam mode

            // publish localmap to octomap_server
            slam_to_ros_mode_transform(*localMap, *localMap);// slam mode trans to ros mode
            pcl::toROSMsg(*localMap, localMap_pcl_to_publish);
            localMap_pcl_to_publish.header.frame_id = "/camera_sensor"; //ros mode,camera_sensor frame_id

            if(localMap_pcl_to_publish.data.size())
            {
                camera_to_map_tfbroadcaster = new tf::TransformBroadcaster;
                localMap_pcl_to_publish.header.stamp = mpCurrentkeyFrame->ros_time;
                local_pcl_publisher.publish(localMap_pcl_to_publish);
                camera_to_map_tfbroadcaster->sendTransform(
                    tf::StampedTransform(camera_to_map_tf, mpCurrentkeyFrame->ros_time, "/map", "/camera_sensor"));
                delete camera_to_map_tfbroadcaster;
            }

            //publish semantic object
            int obj_counts = mpDetector3D->mpObjectDatabase->getDataBaseSize();
            for(uint16_t id = 0; id < obj_counts; ++id)
            {
                cube_marker_to_publish.id = id+1;
                
                std::string name;
                name = mpDetector3D->mpObjectDatabase->getObjectByID(id).object_name;

                if (name == "chair")settingMarkerColor(Blue);
                else if(name == "tvmonitor")settingMarkerColor(Red);
                else if(name == "bottle")settingMarkerColor(Green);
                else settingMarkerColor(Yellow);

                // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
                // x->2,y->-0,z->-1 is slam mode to ros mode
                cube_marker_to_publish.pose.position.x = mpDetector3D->mpObjectDatabase->getObjectByID(id).centroid[2];
                cube_marker_to_publish.pose.position.y = -mpDetector3D->mpObjectDatabase->getObjectByID(id).centroid[0];
                cube_marker_to_publish.pose.position.z = -mpDetector3D->mpObjectDatabase->getObjectByID(id).centroid[1];
                //"Uninitialized quaternion, assuming identity" warning in rviz can be eliminated.
                cube_marker_to_publish.pose.orientation.w = 1;
                
                // cube_marker_to_publish.scale.x = mpDetector3D->mpObjectDatabase->getObjectByID(id).size[2];
                // cube_marker_to_publish.scale.y = -mpDetector3D->mpObjectDatabase->getObjectByID(id).size[0];
                // cube_marker_to_publish.scale.z = -mpDetector3D->mpObjectDatabase->getObjectByID(id).size[1];

                // 0.75 is just a meaningless coefficient in order to reduce the volume value
                cube_marker_to_publish.scale.x = mpDetector3D->mpObjectDatabase->getObjectSize(id)*0.75;
                cube_marker_to_publish.scale.y = mpDetector3D->mpObjectDatabase->getObjectSize(id)*0.75;
                cube_marker_to_publish.scale.z = mpDetector3D->mpObjectDatabase->getObjectSize(id)*0.75;
                marker_publisher.publish(cube_marker_to_publish);

                text_marker_to_publish.id = id+1;
                text_marker_to_publish.text = name + ",id:" + to_string(text_marker_to_publish.id) +":("+
                                        to_string(mpDetector3D->mpObjectDatabase->getObjectByID(id).centroid[2])+","+
                                        to_string(-mpDetector3D->mpObjectDatabase->getObjectByID(id).centroid[0])+","+
                                        to_string(-mpDetector3D->mpObjectDatabase->getObjectByID(id).centroid[1])+")";
                text_marker_to_publish.pose.position.x = mpDetector3D->mpObjectDatabase->getObjectByID(id).centroid[2];
                text_marker_to_publish.pose.position.y = -mpDetector3D->mpObjectDatabase->getObjectByID(id).centroid[0];
                text_marker_to_publish.pose.position.z = -mpDetector3D->mpObjectDatabase->getObjectByID(id).centroid[1];
                marker_publisher.publish(text_marker_to_publish);
            }
        }

        if(is_global_pc_reconstruction && !temp_globalMap->empty())
        {
            static int count = 0;
            voxel_global.setInputCloud(temp_globalMap);
            voxel_global.filter(*temp_globalMap);
            sor_global.setInputCloud(temp_globalMap);
            sor_global.filter(*temp_globalMap); //ros mode
            *globalMap += *temp_globalMap;

            //
            if(!CheckNewKeyFrames() || (count > global_pc_update_kf_threshold))
            {
                
                voxel_global.setInputCloud(globalMap);
                voxel_global.filter(*globalMap);
                sor_global.setInputCloud(globalMap);
                sor_global.filter(*globalMap); //ros mode
                
                pcl::toROSMsg(*globalMap, globalMap_pcl_to_publish);
                globalMap_pcl_to_publish.header.stamp = ros::Time::now();
                globalMap_pcl_to_publish.header.frame_id = "/map";//ros mode,map frame_id
                if(globalMap_pcl_to_publish.data.size())
                {
                    global_pcl_publisher.publish(globalMap_pcl_to_publish);
                    count = 0;
                }
            }
            count++;
        }
    }
}

void PointCloudMapping::slam_to_ros_mode_transform(pcl::PointCloud<pcl::PointXYZRGB>& source, pcl::PointCloud<pcl::PointXYZRGB>& out)
{
	Eigen::Matrix4f T_slam_to_ros;

	T_slam_to_ros<< 0,0,1,0,
	               -1,0,0,0,
		            0,-1,0,0,
                    0,0,0,0;

	Eigen::Affine3f transform(T_slam_to_ros);
	pcl::transformPointCloud(source, out, transform);
}

void PointCloudMapping::shutdown()
{
    {
        unique_lock<mutex> lck(shutDownMutex);
        shutDownFlag = true;
    }
    viewerThread->join();
}
void PointCloudMapping::settingTextMarkerBasicParameter(double scale)
{
    text_marker_to_publish.header.frame_id="/map";
    text_marker_to_publish.ns = "sematic_objects_coordinate";
    text_marker_to_publish.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker_to_publish.action = visualization_msgs::Marker::ADD;
    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    text_marker_to_publish.scale.x = scale;
    text_marker_to_publish.scale.y = scale;
    text_marker_to_publish.scale.z = scale;

    text_marker_to_publish.color.r = 0.0f;
    text_marker_to_publish.color.g = 0.0f;
    text_marker_to_publish.color.b = 0.0f;
    text_marker_to_publish.color.a = 1.0;
}
void PointCloudMapping::settingCubeMarkerBasicParameter()
{

    cube_marker_to_publish.header.frame_id="/map";
    cube_marker_to_publish.ns = "sematic_objects";
    cube_marker_to_publish.type = visualization_msgs::Marker::CUBE;
    cube_marker_to_publish.action = visualization_msgs::Marker::ADD;
}
void PointCloudMapping::settingMarkerColor(Color color)
{
    // Set the color -- be sure to set alpha to something non-zero!
    if(color == Red)
    {
        cube_marker_to_publish.color.r = 1.0f;
        cube_marker_to_publish.color.g = 0.0f;
        cube_marker_to_publish.color.b = 0.0f;
        cube_marker_to_publish.color.a = 0.25;
    } else if (color == Blue)
    {
        cube_marker_to_publish.color.r = 0.0f;
        cube_marker_to_publish.color.g = 0.0f;
        cube_marker_to_publish.color.b = 1.0f;
        cube_marker_to_publish.color.a = 0.25;
    }
    else if (color == Green)
    {
        cube_marker_to_publish.color.r = 0.0f;
        cube_marker_to_publish.color.g = 1.0f;
        cube_marker_to_publish.color.b = 0.0f;
        cube_marker_to_publish.color.a = 0.25;
    }
    else if (color == Yellow)
    {
        cube_marker_to_publish.color.r = 1.0f;
        cube_marker_to_publish.color.g = 1.0f;
        cube_marker_to_publish.color.b = 0.0f;
        cube_marker_to_publish.color.a = 0.25;
    }
    else
    {
        cube_marker_to_publish.color.r = 0.8f;
        cube_marker_to_publish.color.g = 0.6f;
        cube_marker_to_publish.color.b = 0.3f;
        cube_marker_to_publish.color.a = 0.25;
    }
}

// if in DynamicRegion,return true, otherwise return fasle.
bool PointCloudMapping::isInDynamicRegion(int x,int y,std::vector<cv::Rect_<float> >& vDynamicBorder_)
{
    for(unsigned int i = 0; i < vDynamicBorder_.size(); i++)
    {
        cv::Rect_<float> rect2d = vDynamicBorder_[i];
        if(x > int(rect2d.x) && x < int(rect2d.x + rect2d.width) && y > int(rect2d.y) && y < int(rect2d.y + rect2d.height))
            return true;
    }
    return false;
}