#include "ObjectDatabase.h"

bool SemanticObject::operator ==(const std::string &x){
    return(this->object_name == x);
} 

ObjectDatabase::ObjectDatabase()
{
    DataBaseSize = 0;
    mvSemanticObject.clear();

    for (int i = 0; i < 21; i++)
    {  
       mvColors.push_back(cv::Scalar( i*10 + 40, i*10 + 40, i*10 + 40));
    }
    mvColors[5] = cv::Scalar(255,0,255); // 瓶子 粉色
    mvColors[9] = cv::Scalar(255,0,0);   // 椅子 蓝色
    mvColors[15] = cv::Scalar(0,0,255);  // 人 红色
    mvColors[20] = cv::Scalar(0,255,0);  // 显示器 绿色 
    
    for (int i = 0; i < 21; i++)  
    {
      mvSizes.push_back(0.6);
    }
    mvSizes[5] = 0.2;   // 瓶子
    mvSizes[9] = 0.5;   // 椅子
    mvSizes[20] = 0.5;  // 显示器
}

ObjectDatabase::~ObjectDatabase()
{
}

cv::Scalar ObjectDatabase::getObjectColor(int class_id)
{
   return mvColors[class_id];
}

float ObjectDatabase::getObjectSize(int class_id)
{
   return mvSizes[class_id];
}       
// 返回数据库中 同名字的物体数据
std::vector<SemanticObject>  ObjectDatabase::getObjectByName(std::string objectName)
{
    // 按名字查物体是否在数据库
	std::vector<SemanticObject>::iterator iter   = mvSemanticObject.begin()-1;
	std::vector<SemanticObject>::iterator it_end = mvSemanticObject.end();
    std::vector<SemanticObject> sameName; 
	while(true) 
    {
	    iter = find(++iter, it_end, objectName);
	    if (iter != it_end )
                sameName.push_back(*iter);
	    else
	        break;  
	}
    return sameName;
}

void ObjectDatabase::addObject(SemanticObject& cluster)
{
    // 1. 查看总数量,数据库为空直接加入
    if(!mvSemanticObject.size())
    {
        DataBaseSize++;
        cluster.object_id = DataBaseSize;
        mvSemanticObject.push_back(cluster);
        return;
    }
    else
    {
        // 2. 数据库内已经存在物体了，查找新物体是否在数据库内已经存在
        std::vector<SemanticObject>::iterator iter   = mvSemanticObject.begin()-1;
        std::vector<SemanticObject>::iterator it_end = mvSemanticObject.end();
        std::vector<std::vector<SemanticObject>::iterator> likely_obj;
        while(true)
        {
            iter = find(++iter, it_end, cluster.object_name);
            if (iter != it_end )
                    likely_obj.push_back(iter);
            else break;
        }
        // 3. 如果没找到，则直接添加进数据库
        std::vector<SemanticObject>::iterator best_close;
        float center_distance=100;
        if(!likely_obj.size())
        {
            DataBaseSize++;
            cluster.object_id = DataBaseSize;
            mvSemanticObject.push_back(cluster);
            return;
        }
        else
        {
            // 4. 遍例每一个同名字的物体，找到中心点最近的一个
            for(unsigned int j=0; j<likely_obj.size(); j++)
            {
                std::vector<SemanticObject>::iterator& temp_iter = likely_obj[j];
                SemanticObject& temp_cluster = *temp_iter;
                Eigen::Vector3f dis_vec = cluster.centroid - temp_cluster.centroid;// 中心点连接向量
                float dist = dis_vec.norm();
                if(dist < center_distance)
                {
                    center_distance = dist; // 最短的距离
                    best_close      = temp_iter;// 对应的索引
                }
            }
            // 5. 如果距离小于物体尺寸，则认为是同一个空间中的同一个物体，更新数据库中该物体的信息
            if(center_distance < mvSizes[cluster.class_id])
            {
                best_close->prob     = (best_close->prob + cluster.prob)/2.0; // 综合置信度
                best_close->centroid = (best_close->centroid + cluster.centroid)/2.0; // 中心平均
                best_close->size     = (best_close->size + cluster.size)/2.0; // 中心尺寸
            }
            else
            {
            // 6. 如果距离超过物体尺寸则认为是不同位置的同一种物体，直接放入数据库
                DataBaseSize++;
                cluster.object_id = DataBaseSize;
                mvSemanticObject.push_back(cluster);
            }
        }
    }
    return; 
}



