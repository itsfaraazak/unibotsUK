#include "detector.hpp"
#include <ncnn/cpu.h>
#include <stdexcept>
#include <cmath>
#include <algorithm>
#include <cstdio>
#include <opencv2/dnn.hpp>

Detector::Detector(const std::string& param_path,
                   const std::string& bin_path,
                   int   input_size,
                   float conf_thresh,
                   int   num_threads)
    : input_size_(input_size),
      conf_thresh_(conf_thresh),
      num_threads_(num_threads)
{
    net_.opt.num_threads              = num_threads_;
    net_.opt.use_vulkan_compute       = false;
    net_.opt.use_packing_layout       = false;
    net_.opt.use_fp16_packed          = false;
    net_.opt.use_fp16_storage         = false;
    net_.opt.use_fp16_arithmetic      = false;
    net_.opt.use_bf16_storage         = false;
    net_.opt.use_winograd_convolution = false;
    net_.opt.use_sgemm_convolution    = false;

    if (net_.load_param(param_path.c_str()) != 0)
        throw std::runtime_error("Detector: failed to load param: " + param_path);
    if (net_.load_model(bin_path.c_str()) != 0)
        throw std::runtime_error("Detector: failed to load bin: " + bin_path);

    // Warm-up: forces NCNN thread pool init and validates model output shape
    // before rclcpp::spin() starts, so crashes appear here with useful context.
    {
        cv::Mat dummy(input_size_, input_size_, CV_8UC3, cv::Scalar(114, 114, 114));
        ncnn::Extractor ex = net_.create_extractor();
        ncnn::Mat in = ncnn::Mat::from_pixels(
            dummy.data, ncnn::Mat::PIXEL_BGR2RGB, input_size_, input_size_);
        const float norm[3] = {1/255.f, 1/255.f, 1/255.f};
        in.substract_mean_normalize(nullptr, norm);
        ex.input("in0", in);
        ncnn::Mat out;
        ex.extract("out0", out);
        std::fprintf(stderr,
            "[Detector] warmup OK — out.w=%d out.h=%d out.c=%d (layout: %s)\n",
            out.w, out.h, out.c,
            (out.w < out.h) ? "transposed [num_preds,feat]" : "standard [feat,num_preds]");
    }
}

std::vector<Detection> Detector::detect(const cv::Mat& bgr)
{
    const int orig_w = bgr.cols;
    const int orig_h = bgr.rows;

    // Letterbox resize to input_size_ x input_size_
    float scale  = std::min(float(input_size_) / orig_w,
                            float(input_size_) / orig_h);
    int scaled_w = int(orig_w * scale);
    int scaled_h = int(orig_h * scale);
    int pad_w    = (input_size_ - scaled_w) / 2;
    int pad_h    = (input_size_ - scaled_h) / 2;

    cv::Mat resized;
    cv::resize(bgr, resized, {scaled_w, scaled_h});

    ncnn::Mat in = ncnn::Mat::from_pixels(
        resized.data,
        ncnn::Mat::PIXEL_BGR2RGB,
        scaled_w, scaled_h);

    ncnn::Mat in_padded;
    ncnn::copy_make_border(
        in, in_padded,
        pad_h, input_size_ - scaled_h - pad_h,
        pad_w, input_size_ - scaled_w - pad_w,
        ncnn::BORDER_CONSTANT, 114.f);

    const float norm[3] = {1/255.f, 1/255.f, 1/255.f};
    in_padded.substract_mean_normalize(nullptr, norm);

    ncnn::Extractor ex = net_.create_extractor();
    // ex.set_num_threads(num_threads_);

    // Layer names confirmed from model.ncnn.param
    ex.input("in0", in_padded);
    ncnn::Mat out;
    ex.extract("out0", out);

    return parse_output(out, orig_w, orig_h, scale, pad_w, pad_h);
}

std::vector<Detection> Detector::parse_output(
    const ncnn::Mat& out,
    int orig_w, int orig_h,
    float scale,
    int pad_w, int pad_h)
{
    // YOLO11n NCNN output has two possible layouts depending on export version:
    //   Standard:    out.h=4+nc (small), out.w=num_preds → row(feat)[pred]
    //   Transposed:  out.h=num_preds,    out.w=4+nc      → row(pred)[feat]
    // Feature dim (4+nc) is always << num_preds, so min(w,h) identifies it.
    const bool transposed = (out.w < out.h);
    const int  num_preds  = transposed ? out.h : out.w;
    const int  row_size   = transposed ? out.w : out.h;
    const int  num_cls    = row_size - 4;

    if (num_cls <= 0 || num_preds <= 0) return {};

    std::vector<Detection> dets;
    dets.reserve(16);

    for (int i = 0; i < num_preds; ++i) {
        float best_score = -1.f;
        int   best_cls   = -1;
        for (int c = 0; c < num_cls; ++c) {
            float score = transposed
                ? out.channel(0).row(i)[c + 4]
                : out.channel(0).row(c + 4)[i];
            if (score > best_score) {
                best_score = score;
                best_cls   = c;
            }
        }

        if (best_score < conf_thresh_) continue;

        float cx = transposed ? out.channel(0).row(i)[0] : out.channel(0).row(0)[i];
        float cy = transposed ? out.channel(0).row(i)[1] : out.channel(0).row(1)[i];
        float bw = transposed ? out.channel(0).row(i)[2] : out.channel(0).row(2)[i];
        float bh = transposed ? out.channel(0).row(i)[3] : out.channel(0).row(3)[i];

        float x1 = (cx - bw * 0.5f - pad_w) / scale;
        float y1 = (cy - bh * 0.5f - pad_h) / scale;
        float x2 = (cx + bw * 0.5f - pad_w) / scale;
        float y2 = (cy + bh * 0.5f - pad_h) / scale;

        x1 = std::max(0.f, std::min(x1, float(orig_w)));
        y1 = std::max(0.f, std::min(y1, float(orig_h)));
        x2 = std::max(0.f, std::min(x2, float(orig_w)));
        y2 = std::max(0.f, std::min(y2, float(orig_h)));

        dets.push_back({x1, y1, x2, y2, best_score, best_cls});
    }

    //------
    // ── Non-Maximum Suppression ───────────────────────────────────────────────
    // OpenCV 4.13 NMSBoxes only accepts Rect (int), not Rect2f
    std::vector<cv::Rect> boxes_int;
    std::vector<float>    scores;
    boxes_int.reserve(dets.size());
    scores.reserve(dets.size());

    for (const auto& d : dets) {
        boxes_int.push_back({
            int(d.x1), int(d.y1),
            int(d.x2 - d.x1), int(d.y2 - d.y1)
        });
        scores.push_back(d.confidence);
    }

    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes_int, scores, conf_thresh_, 0.3f, indices);

    std::vector<Detection> final_dets;
    final_dets.reserve(indices.size());
    for (int idx : indices)
        final_dets.push_back(dets[idx]);

    return final_dets;

    //return dets;
}
