#include "detector.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <ament_index_cpp/get_package_share_path.hpp>

#include <unibots_msgs/msg/ball_array.hpp>
#include <unibots_msgs/msg/ball_detection.hpp>
#include <unibots_msgs/msg/obstacle_array.hpp>
#include <unibots_msgs/msg/obstacle_detection.hpp>

#include <cmath>
#include <memory>

// Known object diameters (mm) for pinhole distance estimation
constexpr float PING_PONG_DIAMETER_MM = 40.0f;
constexpr float BEARING_DIAMETER_MM   = 20.0f;  // rulebook §4.3.3: 20mm steel alloy

// Assumed robot width (m) for radius estimate
constexpr float ROBOT_WIDTH_M = 0.30f;


class PerceptionNode : public rclcpp::Node
{
public:
    PerceptionNode() : Node("perception_node")
    {
        // Resolve model files via ament index (Lyrical API)
        const std::string share =
            ament_index_cpp::get_package_share_path("unibots_perception").string();

        const std::string param_path = share + "/models/model.ncnn.param";
        const std::string bin_path   = share + "/models/model.ncnn.bin";

        // Tunable params — change live with: ros2 param set /perception_node <name> <val>
        declare_parameter("conf_threshold", 0.35);
        declare_parameter("input_size",     256);    // 256=~30fps RPi4, 320=~22fps
        declare_parameter("num_threads",    4);
        // hfov_deg: MUST be calibrated per lens (see CLAUDE.md calibration section)
        // Default 60° is a typical wide webcam. Wrong value gives wrong distances/bearings.
        declare_parameter("hfov_deg",       60.0);

        const float conf_thresh = float(get_parameter("conf_threshold").as_double());
        const int   input_size  = get_parameter("input_size").as_int();
        const int   num_threads = get_parameter("num_threads").as_int();
        hfov_deg_               = float(get_parameter("hfov_deg").as_double());

        if (std::abs(hfov_deg_ - 60.0f) < 0.1f) {
            RCLCPP_WARN(get_logger(),
                "hfov_deg is default 60°. Distance/bearing will be WRONG unless "
                "your lens actually has 60° HFOV. See CLAUDE.md for calibration.");
        }

        detector_ = std::make_unique<Detector>(
            param_path, bin_path, input_size, conf_thresh, num_threads);

        // SensorDataQoS = best-effort depth-1, matches camera drivers
        sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/unibots/camera/image_raw",
            rclcpp::SensorDataQoS(),
            std::bind(&PerceptionNode::on_image, this, std::placeholders::_1));

        ball_pub_ = create_publisher<unibots_msgs::msg::BallArray>(
            "/vision/balls", 10);
        obs_pub_  = create_publisher<unibots_msgs::msg::ObstacleArray>(
            "/vision/obstacles", 10);

        RCLCPP_INFO(get_logger(),
            "PerceptionNode ready | conf=%.2f | imgsz=%d | threads=%d | hfov=%.1f°",
            conf_thresh, input_size, num_threads, hfov_deg_);
    }

private:
    std::unique_ptr<Detector> detector_;
    float hfov_deg_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr       sub_;
    rclcpp::Publisher<unibots_msgs::msg::BallArray>::SharedPtr     ball_pub_;
    rclcpp::Publisher<unibots_msgs::msg::ObstacleArray>::SharedPtr obs_pub_;

    void on_image(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv::Mat frame;
        try {
            frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_WARN(get_logger(), "cv_bridge: %s", e.what());
            return;
        }

        const auto detections = detector_->detect(frame);

        unibots_msgs::msg::BallArray     balls;
        unibots_msgs::msg::ObstacleArray obstacles;
        balls.header = obstacles.header = msg->header;

        const float img_w    = float(frame.cols);
        const float hfov_rad = hfov_deg_ * float(M_PI) / 180.0f;

        // focal length in pixels from pinhole model: f = (w/2) / tan(HFOV/2)
        const float focal_px = (img_w / 2.0f) / std::tan(hfov_rad / 2.0f);

        for (const auto& d : detections) {
            const float cx     = (d.x1 + d.x2) * 0.5f;
            const float cy     = (d.y1 + d.y2) * 0.5f;
            const float bbox_px = d.x2 - d.x1;

            // Horizontal bearing: degrees from image centre, positive = right
            const float bearing_deg =
                ((cx - img_w * 0.5f) / img_w) * hfov_deg_;

            if (d.class_id == 0 || d.class_id == 1) {
                // class 0 = ping_pong_ball,  class 1 = bearing (steel ball)
                unibots_msgs::msg::BallDetection b;

                b.ball_type  = (d.class_id == 0) ? "ping_pong" : "steel";
                b.pixel_x    = cx;
                b.pixel_y    = cy;
                b.confidence = d.confidence;
                b.bearing_deg    = bearing_deg;
                b.yolo_confirmed = true;
                b.track_id       = 0;  // Kalman filter not wired in yet — teammate owns this

                const float diam_mm = (d.class_id == 0)
                    ? PING_PONG_DIAMETER_MM
                    : BEARING_DIAMETER_MM;
                b.distance_cm = (bbox_px > 1.0f)
                    ? (diam_mm * focal_px) / (bbox_px * 10.0f)
                    : 0.0f;

                balls.balls.push_back(b);

            } else if (d.class_id == 2) {
                // class 2 = robot obstacle
                unibots_msgs::msg::ObstacleDetection obs;

                // world_x/y are zero until EKF + homography wired in
                obs.world_x = 0.0f;
                obs.world_y = 0.0f;

                // Estimate radius and distance from bbox width via pinhole model
                const float dist_m = (bbox_px > 1.0f)
                    ? (ROBOT_WIDTH_M * focal_px) / bbox_px
                    : 1.0f;
                obs.radius_m           = ROBOT_WIDTH_M / 2.0f;
                obs.is_confirmed_robot = true;
                obs.pixel_x            = cx;
                obs.bearing_deg        = bearing_deg;
                obs.distance_m         = dist_m;

                obstacles.obstacles.push_back(obs);
            }
        }

        ball_pub_->publish(balls);
        obs_pub_->publish(obstacles);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PerceptionNode>());
    rclcpp::shutdown();
    return 0;
}
