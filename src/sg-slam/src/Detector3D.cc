#include "Detector3D.h"

#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/extract_clusters.h>
#include <cmath>

Detector3D::Detector3D()
{
    // 统计学滤波器
    mSor.setMeanK (50);
    mSor.setStddevMulThresh (0);
    mVoxel.setLeafSize(0.01, 0.01, 0.01);
    mpObjectDatabase = new(ObjectDatabase);

}

Detector3D::~Detector3D()
{
    delete mpObjectDatabase;
}

void Detector3D::Detect(std::vector<Object2D>& vobject2d, cv::Mat depth, PointCloud::Ptr kf_point_cloud, PointCloud::Ptr object_point_cloud)
{
    if(!vobject2d.empty())
    {
        for(unsigned int i=0; i<vobject2d.size(); i++)
        {
            SemanticObject semantic_object;
            bool ok = DetectOne(vobject2d[i], semantic_object, depth, kf_point_cloud,object_point_cloud);//in out in in out
            if(ok)
                mpObjectDatabase->addObject(semantic_object);

        }
    }
}

bool Detector3D::DetectOne(Object2D& object2d, SemanticObject& semantic_object, cv::Mat depth_img,
                           PointCloud::Ptr kf_point_cloud,PointCloud::Ptr object_point_cloud)
{
    if(object2d.id !=15)// not people
    {
        float* depth = (float*)depth_img.data;
        cv::Rect_<float> rect = object2d.rect;
        int beg = (int)rect.x + ((int)rect.y-1)*depth_img.cols - 1;

// 1. 计算平均深度
        size_t count = 0;
        double depth_sum = 0.0;
        double depth_threshold ;
        int row_beg = (int)rect.height*0.35;//0.35   0.65
        int row_end = (int)rect.height*0.65;
        int col_beg = (int)rect.width*0.35;
        int col_end = (int)rect.width*0.65;
        for(int k = row_beg; k < row_end; k++)
        {
            int start = beg + k * depth_img.cols + col_beg;
            int end   = beg + k * depth_img.cols + col_end ;
            for(int j = start; j < end; j++)
            {
                float d = depth[j];
                if (d < 0.6 || d > 4.0 || isnan(d)) continue;
                depth_sum += d;
                count++;
            }
        }
        if(count>0)
            depth_threshold = depth_sum / count;
        else return false;

// 2. 根据平均深度值 获取目标点云索引
        pcl::PointIndices indices;
        
        row_beg = (int)rect.height*0.3;
        row_end = (int)rect.height*0.7;
        col_beg = (int)rect.width*0.3;
        col_end = (int)rect.width*0.7;
        
        for(int k = row_beg; k < row_end; k++)
        {
            int start = beg + k*depth_img.cols + col_beg;
            int end   = beg + k*depth_img.cols + col_end ;
            for(int j = start; j < end; j++)
            {
                if( abs(depth[j] - depth_threshold) < 0.25)
                    indices.indices.push_back(j);
            }
        }

// 3. 使用点云索引提取点云
        mExtractInd.setInputCloud(kf_point_cloud);
        mExtractInd.setIndices (boost::make_shared<const pcl::PointIndices> (indices));
        PointCloud::Ptr point_cloud(new PointCloud);
        mExtractInd.filter (*point_cloud);

// 4. 滤波
        if(!point_cloud->empty())
        {
            mVoxel.setInputCloud( point_cloud );
            mVoxel.filter( *point_cloud );
            mSor.setInputCloud (point_cloud);
            mSor.filter(*point_cloud);
        }
        *object_point_cloud = *point_cloud;
        
// 5. 计算点云团参数
        Eigen::Vector4f centeroid;
        pcl::compute3DCentroid(*object_point_cloud, centeroid);
        pcl::PointXYZRGB minPt,maxPt;
        pcl::getMinMax3D (*object_point_cloud, minPt, maxPt);

        semantic_object.object_name = object2d.name;
        semantic_object.class_id    = object2d.id;
        semantic_object.prob        = object2d.prob;
        //semantic_object.size        = Eigen::Vector3f(maxPt[0]-minPt[0], maxPt[1]-minPt[1], maxPt[2]-minPt[2]);
        semantic_object.size        = Eigen::Vector3f(maxPt.x-minPt.x, maxPt.y-minPt.y, maxPt.z-minPt.z);
        semantic_object.centroid    = Eigen::Vector3f(centeroid[0],  -centeroid[1],  centeroid[2]); 
        
        return true;
    }
    return false;
}






