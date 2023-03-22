#include "pointcloudmapping.h"

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "Converter.h"

#include <pcl/visualization/cloud_viewer.h>


std::vector<SemanticObject> SemanticObjects;


PointCloudMapping::PointCloudMapping(double resolution)
{
    semanticObject  = boost::make_shared< pcl::PointCloud<pcl::PointXYZRGB> >( );
    globalMap       = boost::make_shared< pcl::PointCloud<pcl::PointXYZRGB> >( );

    mpDetector3D = new(Detector3D);
    voxel.setLeafSize( resolution, resolution, resolution);
    sor.setMeanK (50);
    sor.setStddevMulThresh (0);
    viewerThread = boost::make_shared<thread>( bind(&PointCloudMapping::MapViewer, this ) );
}

PointCloudMapping::~PointCloudMapping()
{
    delete mpDetector3D;
}

void PointCloudMapping::insertKeyFrame(KeyFrame* kf, cv::Mat& color, cv::Mat& depth)
{
    unique_lock<mutex> lck(keyframeMutex);
    keyframes.push_back( kf );
    color.copyTo(kf->mImRGB);
    depth.copyTo(kf->mImDep);
    keyFrameUpdated.notify_one();
}

void PointCloudMapping::generatePointCloud(KeyFrame* kf ,pcl::PointCloud<pcl::PointXYZRGB>::Ptr kf_point_cloud_ ,
                                                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr so_point_cloud_)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGB>);
    temp->resize(kf->mImDep.rows * kf->mImDep.cols);
    temp->width = kf->mImDep.cols;
    temp->height = kf->mImDep.rows;
    temp->is_dense = false;

    for ( int m=0; m<kf->mImDep.rows; m+=1)
    {
        for ( int n=0; n<kf->mImDep.cols; n+=1)
        {
            float d = kf->mImDep.ptr<float>(m)[n];

            int index = m * kf->mImDep.cols + n;

            temp->points[index].z = d;
            temp->points[index].x = ( n - kf->cx) * d / kf->fx;
            temp->points[index].y = ( m - kf->cy) * d / kf->fy;
            temp->points[index].r = kf->mImRGB.ptr<uchar>(m)[n*3+2];
            temp->points[index].g = kf->mImRGB.ptr<uchar>(m)[n*3+1];
            temp->points[index].b = kf->mImRGB.ptr<uchar>(m)[n*3+0];
            //if(d < 0.6 || d > 6.0) temp->points[index].z = NAN; //default 1.0 4.0
            //if(temp->points[index].y < -2.5 || temp->points[index].y > 2.5) temp->points[index].z = NAN;
        }
    }

    Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat( kf->GetPose() );
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr kf_point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::transformPointCloud( *temp, *kf_point_cloud, T.inverse().matrix());


    //return_pcl
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr so_point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    //3D Object Detect
    if(kf->mvObject2D.size()>0)
    {
        mpDetector3D->Detect(kf->mvObject2D,kf->mImDep,kf_point_cloud,so_point_cloud);
        SemanticObjects = mpDetector3D->mpObjectDatabase->mvSemanticObject;
    }

    *kf_point_cloud_ = *kf_point_cloud;
    *so_point_cloud_ = *so_point_cloud;
}

void PointCloudMapping::MapViewer()
{
    std::cout<<"start viewer."<< std::endl;
    ros::NodeHandle nh;
    pcl_publisher = nh.advertise<sensor_msgs::PointCloud2>("/SG_SLAM/Point_Clouds",100);
    marker_publisher= nh.advertise<visualization_msgs::Marker>("/SG_SLAM/Semantic_Objects",100);

    settingTextMarkerBasicParameter(0.2);
    settingCubeMarkerBasicParameter();

    ros::Rate r(50);
    while(1)
    {
        {
            unique_lock<mutex> lck_shutdown( shutDownMutex );
            if (shutDownFlag)
            {
                break;
            }
        }

        {
            unique_lock<mutex> lck_keyframeUpdated( keyFrameUpdateMutex );
            keyFrameUpdated.wait( lck_keyframeUpdated );
        }

        size_t N=0;
        {
            unique_lock<mutex> lck( keyframeMutex );
            N = keyframes.size();
        }
        if(N==0)
	    {
	        cout<<"keyframe miss."<<endl;
            usleep(500);
	        continue;
	    }

        semanticObject->clear();
        globalMap->clear();

        for ( size_t i=lastKeyframeSize; i < N ; i++ )
        {
            generatePointCloud(keyframes[i],globalMap,semanticObject);	   
        }

        // if(!semanticObject->empty())
        // {
        //     voxel.setInputCloud( semanticObject );
        //     voxel.filter( semanticObject_filtered );
        // }
       
        if(!globalMap->empty())
        {
            voxel.setInputCloud( globalMap );
            voxel.filter( *globalMap );
            sor.setInputCloud (globalMap);
            sor.filter(globalMap_filtered);
        }
        
	    //Cloud_transform(semanticObject_filtered,semanticObject_cloud_filtered); 
        Cloud_transform(globalMap_filtered,globalMap_cloud_filtered);

	    //pcl::toROSMsg(semanticObject_cloud_filtered, semanticObject_pcl_to_publish);
        pcl::toROSMsg(globalMap_cloud_filtered, globalMap_pcl_to_publish);

        //semanticObject_pcl_to_publish.header.frame_id = "/pointCloud";
        globalMap_pcl_to_publish.header.frame_id = "/pointCloud";

        int so_count = mpDetector3D->mpObjectDatabase->getDataBaseSize();

        for(uint16_t id = 0; id < so_count; ++id)
        {
            cube_marker_to_publish.id = id+1;
            
            string name;
            name = mpDetector3D->mpObjectDatabase->mvSemanticObject[id].object_name;

            if (name == "chair")settingMarkerColor(Blue);
            else if(name == "tvmonitor")settingMarkerColor(Red);
            else if(name == "bottle")settingMarkerColor(Green);
            else settingMarkerColor(Yellow);

            // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
            cube_marker_to_publish.pose.position.x = mpDetector3D->mpObjectDatabase->mvSemanticObject[id].centroid[2];
            cube_marker_to_publish.pose.position.y = -mpDetector3D->mpObjectDatabase->mvSemanticObject[id].centroid[0];
            cube_marker_to_publish.pose.position.z = mpDetector3D->mpObjectDatabase->mvSemanticObject[id].centroid[1]+0.8;

            // cube_marker_to_publish.scale.x = mpDetector3D->mpObjectDatabase->mvSemanticObject[id].size[2];
            // cube_marker_to_publish.scale.y = mpDetector3D->mpObjectDatabase->mvSemanticObject[id].size[0];
            // cube_marker_to_publish.scale.z = mpDetector3D->mpObjectDatabase->mvSemanticObject[id].size[1];

            cube_marker_to_publish.scale.x = 0.5;
            cube_marker_to_publish.scale.y = 0.5;
            cube_marker_to_publish.scale.z = 0.5;

            marker_publisher.publish(cube_marker_to_publish);
            //std::cout<<name<<":"<<cube_marker_to_publish.pose.position.x<<","<<-cube_marker_to_publish.pose.position.y<<","<<
            //cube_marker_to_publish.pose.position.z<<"."<<std::endl;

            text_marker_to_publish.id = id+1;
            text_marker_to_publish.text = name + ":("+to_string(mpDetector3D->mpObjectDatabase->mvSemanticObject[id].centroid[2])
                                     +","+to_string(-mpDetector3D->mpObjectDatabase->mvSemanticObject[id].centroid[0])+","+
                                     to_string(-mpDetector3D->mpObjectDatabase->mvSemanticObject[id].centroid[1])+")";
            text_marker_to_publish.pose.position.x = mpDetector3D->mpObjectDatabase->mvSemanticObject[id].centroid[2];
            text_marker_to_publish.pose.position.y = -mpDetector3D->mpObjectDatabase->mvSemanticObject[id].centroid[0];
            text_marker_to_publish.pose.position.z = mpDetector3D->mpObjectDatabase->mvSemanticObject[id].centroid[1]+0.8;
           
            marker_publisher.publish(text_marker_to_publish);

        }

/*
        for(uint16_t id = SemanticObject_Count; id < so_count; ++id)
        {
            cube_marker_to_publish.id = id+1;
            
            string name;
            name = mpDetector3D->mpObjectDatabase->mvSemanticObject[id].object_name;

            if (name == "chair")settingMarkerColor(Blue);
            else if(name == "tvmonitor")settingMarkerColor(Red);
            else if(name == "bottle")settingMarkerColor(Green);
            else settingMarkerColor(Yellow);

            // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
            cube_marker_to_publish.pose.position.x = mpDetector3D->mpObjectDatabase->mvSemanticObject[id].centroid[2];
            cube_marker_to_publish.pose.position.y = -mpDetector3D->mpObjectDatabase->mvSemanticObject[id].centroid[0];
            cube_marker_to_publish.pose.position.z = mpDetector3D->mpObjectDatabase->mvSemanticObject[id].centroid[1];
            //cube_marker_to_publish.scale.x = mpDetector3D->mpObjectDatabase->mvSemanticObject[id].size[2];
            //cube_marker_to_publish.scale.y = mpDetector3D->mpObjectDatabase->mvSemanticObject[id].size[0];
            //cube_marker_to_publish.scale.z = mpDetector3D->mpObjectDatabase->mvSemanticObject[id].size[1];

            cube_marker_to_publish.scale.x = 0.5;
            cube_marker_to_publish.scale.y = 0.5;
            cube_marker_to_publish.scale.z = 0.5;

            marker_publisher.publish(cube_marker_to_publish);
            //std::cout<<name<<":"<<cube_marker_to_publish.pose.position.x<<","<<-cube_marker_to_publish.pose.position.y<<","<<
            //cube_marker_to_publish.pose.position.z<<"."<<std::endl;

            text_marker_to_publish.id = 9999-(id+1);
            text_marker_to_publish.text = name + ":("+to_string(mpDetector3D->mpObjectDatabase->mvSemanticObject[id].centroid[2])
                                     +","+to_string(-mpDetector3D->mpObjectDatabase->mvSemanticObject[id].centroid[0])+","+
                                     to_string(-mpDetector3D->mpObjectDatabase->mvSemanticObject[id].centroid[1])+")";
            text_marker_to_publish.pose.position.x = mpDetector3D->mpObjectDatabase->mvSemanticObject[id].centroid[2];
            text_marker_to_publish.pose.position.y = -mpDetector3D->mpObjectDatabase->mvSemanticObject[id].centroid[0];
            text_marker_to_publish.pose.position.z = mpDetector3D->mpObjectDatabase->mvSemanticObject[id].centroid[1];
          
            marker_publisher.publish(text_marker_to_publish);

            SemanticObject_Count = so_count;
        }
*/        
        // if(semanticObject_pcl_to_publish.data.size())
        // {
        //     pcl_publisher.publish(semanticObject_pcl_to_publish);
        // }
        if(globalMap_pcl_to_publish.data.size())
        {
            pcl_publisher.publish(globalMap_pcl_to_publish);
        }



        lastKeyframeSize = N;
    }

}

void PointCloudMapping::Cloud_transform(pcl::PointCloud<pcl::PointXYZRGB>& source, pcl::PointCloud<pcl::PointXYZRGB>& out)
{
	Eigen::Matrix4f m;

	m<< 0,0,1,0,
	    -1,0,0,0,
		0,-1,0,0,
        0,0,0,0;

	Eigen::Affine3f transform(m);
	pcl::transformPointCloud (source, out, transform);
}

void PointCloudMapping::shutdown()
{
    {
        unique_lock<mutex> lck(shutDownMutex);
        shutDownFlag = true;
        keyFrameUpdated.notify_one();
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
/*
pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudMapping::generatePointCloud(KeyFrame* kf)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_merge(new pcl::PointCloud<pcl::PointXYZRGB>);
    if(kf->mvObject2D.size()>0)
    {
        temp_merge->resize(kf->mImDep.rows * kf->mImDep.cols);
        temp_merge->width = kf->mImDep.cols;
        temp_merge->height = kf->mImDep.rows;
    }

    for ( int m=0; m<kf->mImDep.rows; m+=1)
    {
        for ( int n=0; n<kf->mImDep.cols; n+=1)
        {
            float d = kf->mImDep.ptr<float>(m)[n];

            pcl::PointXYZRGB p;
            p.z = d;
            p.x = ( n - kf->cx) * p.z / kf->fx;
            p.y = ( m - kf->cy) * p.z / kf->fy;
            //if(p.y < -1.2 || p.y >5.0) continue;
            p.b = kf->mImRGB.ptr<uchar>(m)[n*3+0];
            p.g = kf->mImRGB.ptr<uchar>(m)[n*3+1];
            p.r = kf->mImRGB.ptr<uchar>(m)[n*3+2];


            if(d > 0.8 && d < 4.0) temp->points.push_back(p);

            if(kf->mvObject2D.size()>0)
            {
                int index = m * kf->mImDep.cols + n;
                if(p.z < 0.8 || p.z > 4.0) p.z = 0.0;
                temp_merge->points[index].z = p.z;
                temp_merge->points[index].x = p.x;
                temp_merge->points[index].y = p.y;
                temp_merge->points[index].r = p.r;
                temp_merge->points[index].g = p.g;
                temp_merge->points[index].b = p.b;
                if(p.z < 0.8 || p.z > 4.0) temp_merge->points[index].z = NAN;
            }
        }
    }

    Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat( kf->GetPose() );
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::transformPointCloud( *temp, *cloud, T.inverse().matrix());
    cloud->is_dense = false;

    //return_pcl
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_merge(new pcl::PointCloud<pcl::PointXYZRGB>);
    if(kf->mvObject2D.size()>0)
    {

        pcl::transformPointCloud( *temp_merge, *cloud_merge, T.inverse().matrix());
        cloud_merge->is_dense = false;
        //3D Object Detect
        mpDetector3D->Detect(kf->mvObject2D,kf->mImDep,cloud_merge,object_cloud);// in in in out
        SemanticObjects = mpDetector3D->mpObjectDatabase->mvSemanticObject;
    }

   // return cloud;
   return cloud_merge;
}
*/