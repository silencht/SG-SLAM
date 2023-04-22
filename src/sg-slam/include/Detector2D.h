#ifndef DETECTOR2D_H
#define DETECTOR2D_H

#include "net.h"
#include "gpu.h" 


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <vector>
#include "iostream"
#include "unistd.h"

// thread
#include <thread>
#include <mutex>



template<class T>
const T& clamp(const T& v, const T& lo, const T& hi)
{
    assert(!(hi < lo));
    return v < lo ? lo : hi < v ? hi : v;
}

//检测目标结果
typedef struct Object2D
{
    cv::Rect_<float> rect;
    float prob;
    std::string name;
    int id;
} Object2D;


namespace ORB_SLAM2 {

    class Tracking;
    class Detector2D{
    public:

        Detector2D(float detection_confidence_threshold_,float dynamic_detection_confidence_threshold_);
        ~Detector2D();

        void detect(const cv::Mat &bgr);
        void draw_objects(cv::Mat &bgr);
        void Run();
        void SetTracker(Tracking *pTracker);
        bool isNewImageArrived();
        void ImageDetectFinished();

        cv::Mat mImageToDetect;
        std::mutex mMutexGetNewImage;
        std::mutex mMutexImageDetectFinished;
        bool mbNewImageFlag;
        Tracking* mpTracker;

        bool bHaveDynamicObject = false;
        std::vector<Object2D> mvObjects2D;
        std::vector<Object2D> mvObjects2D_to_View;

    private:
        const int target_size = 300;
        const float mean_vals[3] = {123.675f, 116.28f, 103.53f};
        const float norm_vals[3] = {1.0f, 1.0f, 1.0f};

        ncnn::Net *detect_net_ptr;
        ncnn::Mat *net_in_ptr;
        static const char *class_names[];

        float detection_confidence_threshold = 0.98;
        float dynamic_detection_confidence_threshold = 0.1;
    };

}
#endif
