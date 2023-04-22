#ifndef OBJECTDATABASE_H
#define OBJECTDATABASE_H

#include "System.h"

#include <Eigen/Core>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>


typedef struct SemanticObject
{
 Eigen::Vector3f size;    
 Eigen::Vector3f centroid;
 float prob;              
 std::string object_name;
 int class_id;        
 int object_id;
 bool operator ==(const std::string &name);
} SemanticObject;


class ObjectDatabase
{
public:
    ObjectDatabase();
    ~ObjectDatabase();
    void addObject(SemanticObject& cluster);
    cv::Scalar  getObjectColor(int class_id);
    float getObjectSize(int class_id);

    inline SemanticObject& getObjectByID(uint64_t id){return mvSemanticObject[id];}
    inline int getDataBaseSize(){return DataBaseSize;}
    
private:
    std::vector<SemanticObject>  mvSemanticObject;
    std::vector<cv::Scalar> mvColors;
    std::vector<float>      mvSizes;
    uint64_t DataBaseSize;
};

#endif // OBJECTDATABASE_H
