#ifndef OBJECTDATABASE_H
#define OBJECTDATABASE_H

#include "System.h"

#include <Eigen/Core>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

// 目标语义信息
typedef struct SemanticObject
{
 Eigen::Vector3f size;    
 Eigen::Vector3f centroid;
 float prob;              
 std::string object_name; // 物体类别名
 int class_id;           
 int object_id;
 bool operator ==(const std::string &x);
} SemanticObject;


class ObjectDatabase
{
public:
    ObjectDatabase();
    ~ObjectDatabase();
    void addObject(SemanticObject& cluster);
    cv::Scalar  getObjectColor(int class_id);
    float getObjectSize(int class_id);

    std::vector<SemanticObject>  getObjectByName(std::string objectName);
    std::vector<SemanticObject> mvSemanticObject;
    inline int getDataBaseSize(){return DataBaseSize;}
protected:
    std::vector<cv::Scalar> mvColors;
    std::vector<float>      mvSizes;
    uint16_t DataBaseSize;
};

#endif // OBJECTDATABASE_H
