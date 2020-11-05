#include <util/image_util.h>

#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <popl.hpp>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    // create options
    popl::OptionParser op("Allowed options");
    auto help = op.add<popl::Switch>("h", "help", "produce help message");
    auto img_dir_path = op.add<popl::Value<std::string>>("i", "img-dir", "directory path which contains images");
    auto img_fps = op.add<popl::Value<unsigned int>>("", "fps", "FPS of images", 2);

    try {
        op.parse(argc, argv);
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        std::cerr << std::endl;
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }

    // check validness of options
    if (help->is_set()) {
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }
    if (!img_dir_path->is_set()) {
        std::cerr << "invalid arguments" << std::endl;
        std::cerr << std::endl;
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }

    // initialize this node
    auto node = std::make_shared<rclcpp::Node>("image_publisher");
    rmw_qos_profile_t custom_qos = rmw_qos_profile_default;
    custom_qos.depth = 1;

    auto publisher = image_transport::create_publisher(node.get(), "/video/image_raw", custom_qos);

    sensor_msgs::msg::Image::SharedPtr msg;

    // load video file
    image_sequence sequence(img_dir_path->value(), img_fps->value());
    const auto frames = sequence.get_frames();

    rclcpp::WallRate pub_rate(img_fps->value());
    rclcpp::executors::SingleThreadedExecutor exec;

    exec.add_node(node);

    std::string extension = ".png";
    char s[25];
    int a;

    for (unsigned int i = 0; i < frames.size(); ++i) {
        a = 100000 + i;
        sprintf(s, "%d", a);
        std::string stg_left = (img_dir_path->value() + s + extension).c_str();
        //const auto& frame = frames.at(i);
        const auto img = cv::imread(stg_left, cv::IMREAD_UNCHANGED);
        msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img).toImageMsg();
        msg->header.stamp = node->now();
        msg->header.frame_id = "image";
        publisher.publish(msg);
        exec.spin_some();
        pub_rate.sleep();
    }

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}