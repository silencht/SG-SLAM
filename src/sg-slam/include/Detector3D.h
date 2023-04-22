#ifndef DETECTOR3D_H
#define DETECTOR3D_H

#ifndef PCL_NO_PRECOMPILE
#define PCL_NO_PRECOMPILE
#endif

#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <Eigen/Core>
#include <vector>
#include "ObjectDatabase.h"
#include "Detector2D.h"

struct SemanticObject;
class  ObjectDatabase;

struct PointXYZPixel
{
  PCL_ADD_POINT4D;
  // PCL_ADD_RGB;
  uint32_t pixel_x;
  uint32_t pixel_y;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZPixel,
                                  (float, x, x)       
                                  (float, y, y)        
                                  (float, z, z)               
                                  // (uint8_t, b, b)
	                                // (uint8_t, g, g)
	                                // (uint8_t, r, r)
                                  (uint32_t, pixel_x, pixel_x)
                                  (uint32_t, pixel_y, pixel_y)
)

class Detector3D
{

public:
    typedef pcl::PointXYZRGB PointT;
    typedef pcl::PointCloud<PointT> PointCloud;
    Detector3D(int Detect3D_Sor_MeanK_,double Detect3D_Sor_StddevMulThresh_,float Detect3D_Voxel_LeafSize_,
    float EuclideanClusterTolerance_,int EuclideanClusterMinSize_,int EuclideanClusterMaxSize_,
    float DetectSimilarCompareRatio_);
    ~Detector3D();
    void Detect(std::vector<Object2D>& vobject2d, cv::Mat &depth, PointCloud::ConstPtr map_point_cloud);
    
    ObjectDatabase* mpObjectDatabase;

private:
    bool DetectOne(Object2D& object2d, SemanticObject& semantic_object, cv::Mat &depth_img,
                   PointCloud::ConstPtr map_point_cloud);

    // from https://github.com/Ewenwan/ORB_SLAM2_SSD_Semantic/blob/master/perfect/src/MergeSG.cc
    bool GetProjectedROI(const pcl::PointCloud<PointXYZPixel>::ConstPtr point_cloud,cv::Rect_<float> & roi);
    float GetSimilarity(const cv::Rect_<float> & r1, const cv::Rect_<float> & r2, float depth, size_t points_num);
    void PointCloudAddPixel(const PointCloud::ConstPtr orig, const std::vector<int>& indices,
                            pcl::PointCloud<PointXYZPixel>::Ptr dest);

    pcl::ExtractIndices<PointT> mExtractInd;
    pcl::VoxelGrid<PointXYZPixel>  mVoxel_Detect3D;
    pcl::StatisticalOutlierRemoval<PointXYZPixel> mSor_Detect3D;
    pcl::EuclideanClusterExtraction<PointXYZPixel> ec;

    float camera_valid_depth_Min = 0.5;
    float camera_valid_depth_Max = 5.0;
    float DetectSimilarCompareRatio = 0.5;
};

#endif

