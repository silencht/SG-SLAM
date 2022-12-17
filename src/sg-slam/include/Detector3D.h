#ifndef DETECTOR3D_H
#define DETECTOR3D_H

#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>

#include <Eigen/Core>
#include <vector>

#include "ObjectDatabase.h"
#include "Detector2D.h"

struct SemanticObject;
class ObjectDatabase;
extern std::vector<SemanticObject> SemanticObjects;


class Detector3D
{

public:
    typedef pcl::PointXYZRGB PointT;
    typedef pcl::PointCloud<PointT> PointCloud;
    Detector3D();
    ~Detector3D();
    void Detect(std::vector<Object2D>& vobject2d, cv::Mat depth, PointCloud::Ptr pclMap, PointCloud::Ptr object_point_cloud);

    ObjectDatabase* mpObjectDatabase;
protected:
    bool DetectOne(Object2D& object2d, SemanticObject& semantic_object, cv::Mat depth_img,
                   PointCloud::Ptr pclMap,PointCloud::Ptr object_point_cloud);

    pcl::ExtractIndices<PointT> mExtractInd;
    pcl::VoxelGrid<PointT>  mVoxel;
    pcl::StatisticalOutlierRemoval<PointT> mSor;
};

#endif

