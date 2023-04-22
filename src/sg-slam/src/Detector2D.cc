#include "Detector2D.h"
#include "Tracking.h"



namespace ORB_SLAM2 {

    const char *Detector2D::class_names[] = {"background",
                                            "aeroplane", "bicycle", "bird", "boat",
                                            "bottle", "bus", "car", "cat", "chair",
                                            "cow", "diningtable", "dog", "horse",
                                            "motorbike", "person", "pottedplant",
                                            "sheep", "sofa", "train", "tvmonitor"
    };

    Detector2D::Detector2D(float detection_confidence_threshold_,float dynamic_detection_confidence_threshold_):
    detection_confidence_threshold(detection_confidence_threshold_),
    dynamic_detection_confidence_threshold(dynamic_detection_confidence_threshold_)
    {
        detect_net_ptr = new(ncnn::Net);
        net_in_ptr     = new(ncnn::Mat);
        detect_net_ptr->opt.use_vulkan_compute = true;

        detect_net_ptr->load_param("./Thirdparty/ncnn_model/mobilenetv3_ssdlite_voc.param");
        detect_net_ptr->load_model("./Thirdparty/ncnn_model/mobilenetv3_ssdlite_voc.bin");
        mbNewImageFlag=false;
    }

    Detector2D::~Detector2D() {
        delete detect_net_ptr;
        delete net_in_ptr;
    }

    void Detector2D::detect(const cv::Mat &bgr)
    {
        int img_w = bgr.cols;
        int img_h = bgr.rows;

        *net_in_ptr = ncnn::Mat::from_pixels_resize(bgr.data, ncnn::Mat::PIXEL_RGB, bgr.cols, bgr.rows, target_size,target_size);
        net_in_ptr->substract_mean_normalize(mean_vals, norm_vals);
        ncnn::Extractor ex = detect_net_ptr->create_extractor();
        ex.set_light_mode(true);//轻模式，每一层运算后自动回收中间结果内存
        ex.input("input", *net_in_ptr);
        ncnn::Mat out;
        ex.extract("detection_out", out);

        mvObjects2D.clear();
        bHaveDynamicObject = false;
        for (int i = 0; i < out.h; i++) {
            const float *values = out.row(i);
            //If the object confidence is greater than 0.98, 
            //or if currently object is human and its confidence is greater than 0.2.
            //(values[1]>0.2  && int(values[0])==15) is good for dynamic feature culling
            if(values[1] > detection_confidence_threshold || (values[1] > dynamic_detection_confidence_threshold  && int(values[0]) == 15 )){
                Object2D object2d;
                object2d.id = int(values[0]);
                object2d.name = std::string(class_names[int(values[0])]);
                object2d.prob = values[1];
                if(15 == object2d.id)
                {
                    bHaveDynamicObject = true;
                    //std::cout<<"find person."<<std::endl;
                }

                float x1 = clamp(values[2] * target_size, 0.f, float(target_size - 1)) / target_size * img_w;
                float y1 = clamp(values[3] * target_size, 0.f, float(target_size - 1)) / target_size * img_h;
                float x2 = clamp(values[4] * target_size, 0.f, float(target_size - 1)) / target_size * img_w;
                float y2 = clamp(values[5] * target_size, 0.f, float(target_size - 1)) / target_size * img_h;

                object2d.rect.x = x1;
                object2d.rect.y = y1;
                object2d.rect.width = x2 - x1;
                object2d.rect.height = y2 - y1;

                mvObjects2D_to_View.push_back(object2d);
                mvObjects2D.emplace_back(object2d);
            }
        }
    }

    void Detector2D::draw_objects(cv::Mat &image)
    {
        for (size_t i = 0; i < mvObjects2D_to_View.size(); i++)
        {
            const Object2D &obj2d = mvObjects2D_to_View[i];
            if(obj2d.prob > detection_confidence_threshold)
            {
                cv::rectangle(image, obj2d.rect, cv::Scalar(255, 0, 0),3);

                char text[256];
                sprintf(text, "%s %.1f%%", obj2d.name.c_str(), obj2d.prob * 100);

                int baseLine = 0;
                cv::Size label_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.7, 1.75, &baseLine);

                int x = obj2d.rect.x;
                int y = obj2d.rect.y - label_size.height - baseLine;
                if (y < 0)
                    y = 0;
                if (x + label_size.width > image.cols)
                    x = image.cols - label_size.width;

                cv::rectangle(image, cv::Rect(cv::Point(x, y),cv::Size(label_size.width, label_size.height + baseLine)),
                            cv::Scalar(255, 255, 255), CV_FILLED);

                cv::putText(image, text, cv::Point(x, y + label_size.height),cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 0),1.75);
            }
        }
        mvObjects2D_to_View.clear();
    }

    void Detector2D::Run() {
        std::cout<<"Detector2D Thread start ..."<<std::endl;
        while(1)
        {
            usleep(1);
            if(!isNewImageArrived()) continue;
            // detect new image
            detect(mImageToDetect);
            ImageDetectFinished();
        }
    }

    bool Detector2D::isNewImageArrived()
    {
        std::unique_lock<std::mutex> lock(mMutexGetNewImage);
        if(mbNewImageFlag)
        {
            mbNewImageFlag=false;
            return true;
        }
        else
            return false;
    }
    void Detector2D::ImageDetectFinished()
    {
        std::unique_lock <std::mutex> lock(mMutexImageDetectFinished);
        mpTracker->mbDetectImageFinishedFlag=true;
    }

    void Detector2D::SetTracker(Tracking *pTracker)
    {
        mpTracker = pTracker;
    }

}
