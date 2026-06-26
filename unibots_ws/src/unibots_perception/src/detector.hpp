#pragma once
#include <ncnn/net.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

struct Detection {
    float x1, y1, x2, y2;  // pixel coords in original image
    float confidence;
    int   class_id;         // 0=ping_pong_ball  1=bearing  2=robot
};

class Detector {
public:
    Detector(const std::string& param_path,
             const std::string& bin_path,
             int   input_size  = 256,   // 256→~30fps on RPi4; 320→~22fps
             float conf_thresh = 0.35f, // lower catches edge/partial balls
             int   num_threads = 4);

    // Returns detections in original image pixel coords
    std::vector<Detection> detect(const cv::Mat& bgr);

private:
    ncnn::Net net_;
    int       input_size_;
    float     conf_thresh_;
    int       num_threads_;

    // YOLO11n Ultralytics NCNN output: [4+nc, num_preds] — no NMS needed
    std::vector<Detection> parse_output(const ncnn::Mat& out,
                                        int orig_w, int orig_h,
                                        float scale,
                                        int pad_w, int pad_h);
};
