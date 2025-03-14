#include "common.h"
#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <vector>
#include <algorithm>
#include <map>
#include <thread>
#include <chrono>
#include "std_msgs/msg/string.hpp"
#include <csignal>

class ImageGrabber {
public:
    ImageGrabber(ORB_SLAM3::System *pSLAM) : mpSLAM(pSLAM) {}

	void ProcessImage(const cv::Mat &image, double timestamp) {
	    Sophus::SE3f Tcc0 = mpSLAM->TrackMonocular(image, timestamp);
	    Sophus::SE3f Twc = (Tcc0 * Tc0w).inverse();
	
	    rclcpp::Time msg_time = rclcpp::Clock().now();
	    publish_ros_camera_pose(Twc, msg_time);
	    publish_ros_tf_transform(Twc, world_frame_id, cam_frame_id, msg_time);
	    publish_ros_tracked_mappoints(mpSLAM->GetTrackedMapPoints(), msg_time);
		publish_image(image, msg_time);
	}

    ORB_SLAM3::System *mpSLAM;
};

void processImageDirectory(const std::string& image_directory, 
                         ImageGrabber& igb, 
                         rclcpp::Node::SharedPtr node) {
    std::vector<std::string> image_files;
    for (const auto &entry : std::filesystem::directory_iterator(image_directory)) {
        if (entry.path().extension() == ".png" || entry.path().extension() == ".jpg") {
            image_files.push_back(entry.path().string());
        }
    }
    std::sort(image_files.begin(), image_files.end());

    for (const auto &file : image_files) {
        cv::Mat image = cv::imread(file, cv::IMREAD_GRAYSCALE);
        if (!image.empty()) {
            double timestamp = std::stod(std::filesystem::path(file).stem().string());
            igb.ProcessImage(image, timestamp);
            rclcpp::spin_some(node);
        } else {
            RCLCPP_WARN(node->get_logger(), "Failed to load image: %s", file.c_str());
        }
    }
}

void processVideoFile(const std::string& video_file, 
                     ImageGrabber& igb, 
                     rclcpp::Node::SharedPtr node,
                     int stride) {
    cv::VideoCapture cap(video_file);
    if (!cap.isOpened()) {
        RCLCPP_ERROR(node->get_logger(), "Failed to open video file: %s", video_file.c_str());
        rclcpp::shutdown();
        return;
    }

    int frame_count = 0;
    cv::Mat frame;
    while (cap.read(frame)) {
        if (frame_count % stride == 0) {
            cv::Mat gray;
            cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
            double timestamp = cap.get(cv::CAP_PROP_POS_MSEC) / 1000.0;
            igb.ProcessImage(gray, timestamp);
            rclcpp::spin_some(node);
        }
        frame_count++;
    }
    cap.release();
}

struct StatusPublisher {
    static rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub;
    static std::map<std::string, std::string> params;
    static rclcpp::Node::SharedPtr node;

    static void publish_status(const std::string& status, const std::string& method) {
        if (!pub || !node) return;

        std::string args_str;
        for (const auto& [key, value] : params) {
            if (!args_str.empty()) {
                args_str += ", ";
            }
            args_str += "\"" + key + "\": \"" + value + "\"";
        }
        std::string json_str = "{\"status\": \"" + status + "\", \"method\": \"" + method + "\", \"args\": {" + args_str + "}}";

        std_msgs::msg::String msg;
        msg.data = json_str;
        pub->publish(msg);
        RCLCPP_INFO(node->get_logger(), "Published status: %s", json_str.c_str());
    }

    static void signal_handler(int signum) {
        publish_status("finished", "orbslam3");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        rclcpp::shutdown();
        exit(signum);
    }
};

rclcpp::Publisher<std_msgs::msg::String>::SharedPtr StatusPublisher::pub = nullptr;
std::map<std::string, std::string> StatusPublisher::params;
rclcpp::Node::SharedPtr StatusPublisher::node = nullptr;

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("Mono");

    node->declare_parameter<std::string>("voc_file", "file_not_set");
    node->declare_parameter<std::string>("settings_file", "file_not_set");
    node->declare_parameter<std::string>("world_frame_id", "map");
    node->declare_parameter<std::string>("cam_frame_id", "camera");
    node->declare_parameter<bool>("enable_pangolin", true);
    node->declare_parameter<std::string>("image_directory", "");
    node->declare_parameter<std::string>("video_file", "");
    node->declare_parameter<int>("stride", 1);

    std::string voc_file = node->get_parameter("voc_file").as_string();
    std::string settings_file = node->get_parameter("settings_file").as_string();
    std::string image_directory = node->get_parameter("image_directory").as_string();
    std::string video_file = node->get_parameter("video_file").as_string();
    int stride = node->get_parameter("stride").as_int();

    if (voc_file == "file_not_set" || settings_file == "file_not_set") {
        RCLCPP_ERROR(node->get_logger(), "Please provide voc_file and settings_file");
        rclcpp::shutdown();
        return 1;
    }

    world_frame_id = node->get_parameter("world_frame_id").as_string();
    cam_frame_id = node->get_parameter("cam_frame_id").as_string();
    bool enable_pangolin = node->get_parameter("enable_pangolin").as_bool();

    StatusPublisher::node = node;
    StatusPublisher::pub = node->create_publisher<std_msgs::msg::String>("reconstructor_status", 10);

    StatusPublisher::params["voc_file"] = voc_file;
    StatusPublisher::params["settings_file"] = settings_file;
    StatusPublisher::params["world_frame_id"] = world_frame_id;
    StatusPublisher::params["cam_frame_id"] = cam_frame_id;
    StatusPublisher::params["enable_pangolin"] = std::to_string(enable_pangolin);
    StatusPublisher::params["imagedir"] = image_directory;
    StatusPublisher::params["imagedir"] = video_file;
    StatusPublisher::params["stride"] = std::to_string(stride);

    std::signal(SIGINT, StatusPublisher::signal_handler);
    std::signal(SIGTERM, StatusPublisher::signal_handler);

    StatusPublisher::publish_status("started", "orbslam3");

    Eigen::Vector3d rpy_rad(0, 0, 0);
    node->declare_parameter<double>("world_roll", 0.0);
    node->declare_parameter<double>("world_pitch", 0.0);
    node->declare_parameter<double>("world_yaw", 0.0);
    rpy_rad(0) = node->get_parameter("world_roll").as_double();
    rpy_rad(1) = node->get_parameter("world_pitch").as_double();
    rpy_rad(2) = node->get_parameter("world_yaw").as_double();

    sensor_type = ORB_SLAM3::System::MONOCULAR;
    ORB_SLAM3::System SLAM(voc_file, settings_file, sensor_type, enable_pangolin);
    ImageGrabber igb(&SLAM);

    image_transport::ImageTransport it(node);
    setup_ros_publishers(node, it, rpy_rad);

    if (!image_directory.empty()) {
        processImageDirectory(image_directory, igb, node);
    } else if (!video_file.empty()) {
        processVideoFile(video_file, igb, node, stride);
    } else {
        RCLCPP_ERROR(node->get_logger(), "Please provide either image_directory or video_file");
        rclcpp::shutdown();
        return 1;
    }

    RCLCPP_INFO(node->get_logger(), "Shutting down ORB-SLAM3...");
    SLAM.Shutdown();

    StatusPublisher::publish_status("finished", "orbslam3");

    rclcpp::spin_some(node);
    RCLCPP_INFO(node->get_logger(), "Finalizing ROS publishers...");

    node.reset();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Node destroyed, shutting down rclcpp...");
    rclcpp::shutdown();

    return 0;
}
