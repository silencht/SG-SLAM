#include "Detector2D.h"
#include "Tracking.h"

std::vector<Object2D> vObjects;

bool bHaveDynamicObject;

namespace ORB_SLAM2 {

    const char *Detecting::class_names[] = {"background",
                                            "aeroplane", "bicycle", "bird", "boat",
                                            "bottle", "bus", "car", "cat", "chair",
                                            "cow", "diningtable", "dog", "horse",
                                            "motorbike", "person", "pottedplant",
                                            "sheep", "sofa", "train", "tvmonitor"
    };

    Detecting::Detecting() {
        detect_net_ptr = new(ncnn::Net);
        net_in_ptr     = new(ncnn::Mat);
        detect_net_ptr->opt.use_vulkan_compute = true;

        detect_net_ptr->load_param("./Thirdparty/ncnn_model/mobilenetv3_ssdlite_voc.param");
        detect_net_ptr->load_model("./Thirdparty/ncnn_model/mobilenetv3_ssdlite_voc.bin");
        mbNewImageFlag=false;
    }

    Detecting::~Detecting() {
        delete detect_net_ptr;
        delete net_in_ptr;
    }

    void Detecting::detect(const cv::Mat &bgr, std::vector<Object2D> &vobject2d) {

        int img_w = bgr.cols;
        int img_h = bgr.rows;

        *net_in_ptr = ncnn::Mat::from_pixels_resize(bgr.data, ncnn::Mat::PIXEL_RGB, bgr.cols, bgr.rows, target_size,
                                                    target_size);

        net_in_ptr->substract_mean_normalize(mean_vals, norm_vals);

        ncnn::Extractor ex = detect_net_ptr->create_extractor();
        ex.set_light_mode(true);//轻模式，每一层运算后自动回收中间结果内存
        ex.input("input", *net_in_ptr);
        ncnn::Mat out;
        ex.extract("detection_out", out);


        bHaveDynamicObject = false;
        vobject2d.clear();
        for (int i = 0; i < out.h; i++) {
            const float *values = out.row(i);
            if(values[1]>0.98 || (values[1]>0.2  && int(values[0])==15 )){ //default 0.85 0.35 15
                Object2D object2d;
                object2d.id = int(values[0]);
                object2d.name = std::string(class_names[int(values[0])]);
                object2d.prob = values[1];
                if(object2d.id == 15)
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

                vobject2d.push_back(object2d);
            }
        }
    }

    void Detecting::draw_objects(const cv::Mat &bgr, std::vector<Object2D> &vobject2d)
    {
        cv::Mat image = bgr.clone();
        for (size_t i = 0; i < vobject2d.size(); i++)
        {
            const Object2D &obj2d = vobject2d[i];

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

            // cv::imshow("SG-SLAM: Current Frame", image);
            // cv::waitKey(1e3/30.0);
    }

    void Detecting::Run() {
        std::cout<<"Detecting Thread start ..."<<std::endl;
        //cv::namedWindow("image");
        while(1)
        {
            usleep(1);
            if(!isNewImageArrived()) continue;
            // detect new image
            detect(mImageToDetect,vObjects);
            ImageDetectFinished();

            if(vObjects.size()>0)
                draw_objects(mImageToDetect,vObjects);

        }
    }

    bool Detecting::isNewImageArrived()
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
    void Detecting::ImageDetectFinished()
    {
        std::unique_lock <std::mutex> lock(mMutexImageDetectFinished);
        mpTracker->mbDetectImageFinishedFlag=true;
    }

    void Detecting::SetTracker(Tracking *pTracker)
    {
        mpTracker = pTracker;
    }

}
