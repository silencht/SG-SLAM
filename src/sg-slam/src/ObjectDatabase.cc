#include "ObjectDatabase.h"

bool SemanticObject::operator ==(const std::string &name){
    return(this->object_name == name);
} 

ObjectDatabase::ObjectDatabase()
{
    DataBaseSize = 0;
    mvSemanticObject.clear();

    for (int i = 0; i < 21; i++)
    {  
       mvColors.push_back(cv::Scalar(255 * (1 - i)*(2 - i) / 2, 255 * i*(2 - i), 255 * i*(i - 1) / 2));
    }
    mvColors[5] = cv::Scalar(255,0,255); // bottle pink
    mvColors[9] = cv::Scalar(255,0,0);   // chair  blue
    mvColors[15] = cv::Scalar(0,0,255);  // people red
    mvColors[20] = cv::Scalar(0,255,0);  // tvmonitor green
    
    for (int i = 0; i < 21; i++)  
    {
        mvSizes.push_back(0.6);
    }
    mvSizes[5] = 0.2;   // bottle
    mvSizes[9] = 1.0;   // chair
    mvSizes[20] = 0.5;  // tvmonitor
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

void ObjectDatabase::addObject(SemanticObject& cluster)
{
    // 1. If the database is empty, it is added directly
    if(!mvSemanticObject.size())
    {
        DataBaseSize++;
        cluster.object_id = DataBaseSize;
        mvSemanticObject.push_back(cluster);
        return;
    }
    else
    {
        // 2. If object already exist in the database, check whether the new object already exists in the database
        std::vector<SemanticObject>::iterator iter   = mvSemanticObject.begin()-1;
        std::vector<SemanticObject>::iterator it_end = mvSemanticObject.end();
        std::vector<std::vector<SemanticObject>::iterator> likely_obj;
        while(true)
        {
            iter = find(++iter, it_end, cluster.object_name);
            if (iter != it_end)
                    likely_obj.push_back(iter);
            else break;
        }
        // 3. If not, add it directly to the database
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
            // 4. Go through every object with the same name and find the nearest center
            // Todo: The Kuhn－Munkres algorithm with various weight information such as
            // color/size/position is used for matching
            for(unsigned int j=0; j<likely_obj.size(); j++)
            {
                std::vector<SemanticObject>::iterator& temp_iter = likely_obj[j];
                SemanticObject& temp_cluster = *temp_iter;
                Eigen::Vector3f dis_vec = cluster.centroid - temp_cluster.centroid;
                float dist = dis_vec.norm();
                if(dist < center_distance)
                {
                    center_distance = dist; // Shortest distance
                    best_close      = temp_iter;
                }
            }
            // 5. If the distance is less than the size of the object, 
            // it is considered to be the same object in the same space, 
            // and the information of the object is updated in the database(a simple mean filtering)
            if(center_distance < mvSizes[cluster.class_id])
            {
                best_close->prob     = (best_close->prob + cluster.prob)/2.0;
                best_close->centroid = (best_close->centroid + cluster.centroid)/2.0;
                best_close->size     = (best_close->size + cluster.size)/2.0;
            }
            else
            {
                DataBaseSize++;
                cluster.object_id = DataBaseSize;
                mvSemanticObject.push_back(cluster);
            }
        }
    }
    return; 
}



