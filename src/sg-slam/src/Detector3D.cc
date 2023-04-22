#include "Detector3D.h"
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <cmath>

Detector3D::Detector3D(int Detect3D_Sor_MeanK_,double Detect3D_Sor_StddevMulThresh_,float Detect3D_Voxel_LeafSize_,
float EuclideanClusterTolerance_,int EuclideanClusterMinSize_,int EuclideanClusterMaxSize_,float DetectSimilarCompareRatio_)
{
    // 统计学滤波器
    mSor_Detect3D.setMeanK(Detect3D_Sor_MeanK_);
    mSor_Detect3D.setStddevMulThresh(Detect3D_Sor_StddevMulThresh_);
    mVoxel_Detect3D.setLeafSize(Detect3D_Voxel_LeafSize_, Detect3D_Voxel_LeafSize_, Detect3D_Voxel_LeafSize_);
    ec.setClusterTolerance(EuclideanClusterTolerance_);
    ec.setMinClusterSize(EuclideanClusterMinSize_);
    ec.setMaxClusterSize(EuclideanClusterMaxSize_);

    DetectSimilarCompareRatio = DetectSimilarCompareRatio_;
    mpObjectDatabase = new(ObjectDatabase);
}

Detector3D::~Detector3D()
{
    delete mpObjectDatabase;
}

void Detector3D::Detect(std::vector<Object2D>& vobject2d, cv::Mat &depth, PointCloud::ConstPtr map_point_cloud)
{
    if(!vobject2d.empty())
    {
        for(unsigned int i=0; i<vobject2d.size(); i++)
        {
            SemanticObject semantic_object;
            bool ok = DetectOne(vobject2d[i], semantic_object, depth, map_point_cloud);//in out in in
            if(ok)
                mpObjectDatabase->addObject(semantic_object);

        }
    }
}

bool Detector3D::DetectOne(Object2D& object2d, SemanticObject& semantic_object, cv::Mat &depth_img,
                           PointCloud::ConstPtr map_point_cloud)
{
    if(object2d.id != 15)// not people
    {
        float* depths = (float*)depth_img.data;
        cv::Rect_<float> rect2d = object2d.rect;
        size_t beg = (size_t)rect2d.x + (size_t)rect2d.y * depth_img.cols;

//  1. Collect point cloud indices from object2d boundingbox
        pcl::PointIndices object2d_indices;
        size_t row_beg = (size_t)rect2d.height*0.2;
        size_t row_end = (size_t)rect2d.height*0.8;
        size_t col_beg = (size_t)rect2d.width*0.2;
        size_t col_end = (size_t)rect2d.width*0.8;
        for(size_t k = row_beg; k < row_end; k++)
        {
            size_t start = beg + k * depth_img.cols + col_beg;
            size_t end   = beg + k * depth_img.cols + col_end ;
            for(size_t j = start; j < end; j++)
            {
                float d = depths[j];
                if (d < camera_valid_depth_Min || d > camera_valid_depth_Max || isnan(d)) continue;
                object2d_indices.indices.push_back(j);
            }
        }
// 3.  Extract the point cloud using indices and Add pixel information
        pcl::PointCloud<PointXYZPixel>::Ptr allpoint_cloud(new pcl::PointCloud<PointXYZPixel>);
        PointCloudAddPixel(map_point_cloud,object2d_indices.indices,allpoint_cloud);

// 4. Filter
        if(!allpoint_cloud->empty())
        {
            // todo: https://github.com/PointCloudLibrary/pcl/issues/2331
            // mVoxel_Detect3D.setInputCloud(allpoint_cloud);
            // mVoxel_Detect3D.filter(*allpoint_cloud);
            mSor_Detect3D.setInputCloud(allpoint_cloud);
            mSor_Detect3D.filter(*allpoint_cloud);
        }
// 4. Euclidean Cluster Extraction / Segementation
        if(allpoint_cloud->empty()) return false;
        pcl::search::KdTree<PointXYZPixel>::Ptr kdtree(new pcl::search::KdTree<PointXYZPixel>);
        kdtree->setInputCloud(allpoint_cloud);
        std::vector<pcl::PointIndices> vcluster_indices;
        ec.setSearchMethod(kdtree);
        ec.setInputCloud(allpoint_cloud);
        ec.extract(vcluster_indices);
// 5. Obtain every point cloud cluster
        // int i = 0;
        std::vector<pcl::PointCloud<PointXYZPixel>::Ptr> vcloud_cluster;
        for(auto it = vcluster_indices.begin(); it != vcluster_indices.end(); ++it)
        {
            pcl::PointCloud<PointXYZPixel>::Ptr cloud_cluster(new pcl::PointCloud<PointXYZPixel>);
            for(auto pit = it->indices.begin(); pit != it->indices.end(); pit++)
                cloud_cluster->points.push_back(allpoint_cloud->points[*pit]);

            cloud_cluster->is_dense = true;
            vcloud_cluster.emplace_back(cloud_cluster);
            // std::cout<<"current cluster:"<<i<<" points num:"<<cloud_cluster->size()<<std::endl;
            // i++;
        }
// 6. compute every point cloud cluster parameter,and find best object cloud cluster
        float best_similar1 = -1.0;
        float best_similar2 = -1.0;
        Eigen::Vector4f object_centeroid;
        pcl::PointCloud<PointXYZPixel>::Ptr object_cloud_cluster(new pcl::PointCloud<PointXYZPixel>);

        for(auto vit = vcloud_cluster.begin();vit != vcloud_cluster.end(); ++vit)
        {
            Eigen::Vector4f centeroid_;
            pcl::compute3DCentroid(**vit, centeroid_);
            if(centeroid_[2] < camera_valid_depth_Min) continue;

            cv::Rect_<float> rect3d;
            bool ok = GetProjectedROI(*vit,rect3d);
            if(!ok) continue;
            
            size_t points_num = (**vit).points.size();
            float similar = GetSimilarity(rect2d,rect3d,centeroid_[2],points_num);
            if(similar > best_similar1)
            {
                pcl::copyPointCloud(**vit,*object_cloud_cluster);
                object_centeroid = centeroid_;
                best_similar1 = similar;
            }
            else if(similar > best_similar2)
            {
                best_similar2 = similar;
            }
        }
        if(best_similar1 * DetectSimilarCompareRatio < best_similar2 && best_similar2 > 0) 
            return false;

        PointXYZPixel minPt,maxPt;
        pcl::getMinMax3D(*object_cloud_cluster, minPt, maxPt);

        semantic_object.object_name = object2d.name;
        semantic_object.class_id    = object2d.id;
        semantic_object.prob        = object2d.prob;
        semantic_object.size        = Eigen::Vector3f(maxPt.x-minPt.x,maxPt.y-minPt.y,maxPt.z-minPt.z);
        semantic_object.centroid    = Eigen::Vector3f(object_centeroid[0],object_centeroid[1],object_centeroid[2]); 

        /*visualization
        pcl::visualization::PCLVisualizer viewer("PCLVisualizer");
        viewer.initCameraParameters();

        int view1(0);
        viewer.createViewPort(0.0, 0.0, 0.5, 1.0, view1);
        viewer.setBackgroundColor(128.0 / 255.0, 138.0 / 255.0, 135.0 / 255.0, view1);
        viewer.addText("Cloud before segmenting", 10, 10, "view1 test", view1);
        viewer.addPointCloud<PointXYZPixel>(allpoint_cloud, "cloud", view1);

        int view2(0);
        viewer.createViewPort(0.5, 0.0, 1.0, 1.0, view2);
        viewer.setBackgroundColor(128.0 / 255.0, 138.0 / 255.0, 135.0 / 255.0, view2);
        viewer.addText("Cloud after segmenting", 10, 10, "segment", view2);
        for (int i = 0; i < vcloud_cluster.size(); i++)
        {
            string cstr = i + " cloud_segmented";
            pcl::visualization::PointCloudColorHandlerCustom<PointXYZPixel> color(vcloud_cluster[i], 255 * (1 - i)*(2 - i) / 2, 255 * i*(2 - i), 255 * i*(i - 1) / 2);
            viewer.addPointCloud(vcloud_cluster[i], color, cstr, view2);
        }
        while (!viewer.wasStopped())
        {
            viewer.spinOnce(100);
        }
        */
        return true;
    }
    return false;
}

bool Detector3D::GetProjectedROI(const pcl::PointCloud<PointXYZPixel>::ConstPtr point_cloud,cv::Rect_<float> & roi)
{
// lambda function
    auto cmp_x = [](PointXYZPixel const& l, PointXYZPixel const& r) {return l.pixel_x < r.pixel_x; };
// return point_cloud 's min and max element
    auto minmax_x = std::minmax_element(point_cloud->begin(), point_cloud->end(), cmp_x);
// left, roi_pixel_x_min
    roi.x = minmax_x.first->pixel_x;
// right, roi_pixel_x_max
    auto max_x = minmax_x.second->pixel_x;
// verify project parameter is valid
    if(roi.x >= 0 && max_x >= roi.x) 
    {
        roi.width = max_x - roi.x;

        auto cmp_y = [](PointXYZPixel const& l, PointXYZPixel const& r) { return l.pixel_y < r.pixel_y; };
        auto minmax_y = std::minmax_element(point_cloud->begin(), point_cloud->end(), cmp_y);
        //top, roi_pixel_y_min
        roi.y = minmax_y.first->pixel_y;
        //down, roi_pixel_y_max
        auto max_y = minmax_y.second->pixel_y;
        // verify project parameter is valid
        if(roi.y >= 0 && max_y >= roi.y)
        { 
            roi.height = max_y - roi.y;
            return true;
        }  
        else 
            return false;  
    }
    else 
        return false;
}

float Detector3D::GetSimilarity(const cv::Rect_<float> & r1, const cv::Rect_<float> & r2, float depth, size_t points_num)
{
  cv::Rect2f  ir1(r1), ir2(r2);
  cv::Point2f c1(ir1.x + ir1.width / 2, ir1.y + ir1.height / 2);
  cv::Point2f c2(ir2.x + ir2.width / 2, ir2.y + ir2.height / 2);

  float area1 = ir1.area(), area2 = ir2.area(), area0 = (ir1 & ir2).area();
  float overlap = area0 / (area1 + area2 - area0);
  // calculate the deviation between two centers
  float deviate = powf((c1.x - c2.x), 2) + powf((c1.y - c2.y), 2);
  float points_num_score = float(points_num) / 10.0;  //10 is just an order of magnitude lower
  // The larger the area in the ROI, the closer it is to the center of the ROI, 
  // and the closer it is to the camera, the more point clouds of cluster there are, 
  // the higher the similarity
  return (overlap * points_num_score) / (deviate * depth);
}

void Detector3D::PointCloudAddPixel(const PointCloud::ConstPtr orig, 
                                    const std::vector<int>& indices,
                                    pcl::PointCloud<PointXYZPixel>::Ptr dest)
{
  pcl::copyPointCloud(*orig, indices, *dest);
  uint32_t width = orig->width;
  for (uint32_t i = 0; i < indices.size(); i++)
  {
    dest->points[i].pixel_x = indices[i] % width;// col
    dest->points[i].pixel_y = indices[i] / width;// row
  }
}



