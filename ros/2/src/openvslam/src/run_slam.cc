
#ifdef USE_PANGOLIN_VIEWER
    #include <pangolin_viewer/viewer.h>
    #include <openvslam/publish/map_publisher.h>
#elif USE_SOCKET_PUBLISHER
    #include <socket_publisher/publisher.h>
#endif

#include <openvslam/system.h>
#include <openvslam/config.h>

#include <iostream>
#include <chrono>
#include <numeric>
#include <thread>
#include <string>

#include "openvslam/data/keyframe.h"
#include "openvslam/data/landmark.h"

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
//#include <tf2/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <spdlog/spdlog.h>
#include <popl.hpp>

#ifdef USE_STACK_TRACE_LOGGER
#include <glog/logging.h>
#endif

#ifdef USE_GOOGLE_PERFTOOLS
#include <gperftools/profiler.h>
#endif

tf2::Vector3 cam_position;
auto odom_msg_ = std::make_shared<nav_msgs::msg::Odometry>();
auto odom_trans = std::make_shared<geometry_msgs::msg::TransformStamped>();
auto right_image_msg = std::make_shared<sensor_msgs::msg::Image>();
auto left_image_msg = std::make_shared<sensor_msgs::msg::Image>();
auto right_config = std::make_shared<sensor_msgs::msg::CameraInfo>();
auto left_config = std::make_shared<sensor_msgs::msg::CameraInfo>();
auto tf_msg = std::make_shared<geometry_msgs::msg::TransformStamped>();
std::shared_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster;

const cv::Mat mask = cv::Mat{};
std::chrono::_V2::steady_clock::time_point tp_0;

void publi(auto cam_pose_, auto &odometry_pub_, auto node){
    Eigen::Matrix3d rotation_matrix = cam_pose_.block(0, 0, 3, 3);
    Eigen::Vector3d translation_vector = cam_pose_.block(0, 3, 3, 1);

    tf2::Matrix3x3 tf_camera_rotation(rotation_matrix(0, 0), rotation_matrix(0, 1), rotation_matrix(0, 2),
                                      rotation_matrix(1, 0), rotation_matrix(1, 1), rotation_matrix(1, 2),
                                      rotation_matrix(2, 0), rotation_matrix(2, 1), rotation_matrix(2, 2));
    
    tf2::Vector3 tf_camera_translation(translation_vector(0), translation_vector(1), translation_vector(2));
    //Coordinate transformation matrix from orb coordinate system to ros coordinate system
    //rotate 270deg about z and 270deg about x
    tf2::Matrix3x3 matTrans (0, 0, 1,
                            -1, 0, 0,
                            0,-1, 0);

    //Transform actual coordinate system to ros coordinate system on camera coordinates
    tf_camera_rotation = matTrans*tf_camera_rotation;
    tf_camera_translation = matTrans*tf_camera_translation;

    //Inverse matrix
     tf_camera_rotation = tf_camera_rotation.transpose();
    tf_camera_translation = -(tf_camera_rotation*tf_camera_translation);

    tf_camera_rotation = matTrans*tf_camera_rotation;
    tf_camera_translation = matTrans*tf_camera_translation;

    tf2::Transform transform_tf(tf_camera_rotation, tf_camera_translation);
    cam_position = tf_camera_translation;

    // Create odometry message and update it with current camera pose
    odom_msg_->header.stamp = node->now();
    odom_msg_->header.frame_id = "odom";
    odom_msg_->child_frame_id = "base_link";
    odom_msg_->pose.pose.orientation.x = transform_tf.getRotation().getX();
    odom_msg_->pose.pose.orientation.y = transform_tf.getRotation().getY();
    odom_msg_->pose.pose.orientation.z = transform_tf.getRotation().getZ();
    odom_msg_->pose.pose.orientation.w = transform_tf.getRotation().getW();
    odom_msg_->pose.pose.position.x = transform_tf.getOrigin().getX();
    odom_msg_->pose.pose.position.y = transform_tf.getOrigin().getY();
    odom_msg_->pose.pose.position.z = transform_tf.getOrigin().getZ();
    //odometry_pub_->publish(*odom_msg_);

    odom_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);
    odom_trans->header.stamp =  odom_msg_->header.stamp;
    odom_trans->header.frame_id = "odom";
    odom_trans->child_frame_id = "base_link";
    odom_trans->transform.rotation.x = transform_tf.getRotation().getX();
    odom_trans->transform.rotation.y = transform_tf.getRotation().getY();
    odom_trans->transform.rotation.z = transform_tf.getRotation().getZ();
    odom_trans->transform.rotation.w = transform_tf.getRotation().getW();
    odom_trans->transform.translation.x = transform_tf.getOrigin().getX();
    odom_trans->transform.translation.y =  transform_tf.getOrigin().getY();
    odom_trans->transform.translation.z =  transform_tf.getOrigin().getZ();
    odom_broadcaster->sendTransform(*odom_trans);

}


void callback(const sensor_msgs::msg::Image::ConstSharedPtr& left_image_msg_sub,
                const sensor_msgs::msg::Image::ConstSharedPtr& right_image_msg_sub,
                std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image_<std::allocator<void>>, std::allocator<void>>> left_publisher,
                std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image_<std::allocator<void>>, std::allocator<void>>> right_publisher,
                std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CameraInfo_<std::allocator<void>>, std::allocator<void> > > left_publisher_config,
                std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CameraInfo_<std::allocator<void>>, std::allocator<void> > > right_publisher_config,
                std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry_<std::allocator<void> >, std::allocator<void>>> odometry_publisher,
                openvslam::system* SLAM,
                std::shared_ptr<rclcpp::Node> node
                ){

    std::array<double, 9> K0 = {457.00, 0, 320.00, 0, 457.00, 240.0, 0,0,1};   
    std::vector<double> d0 = {0,0,0,0,0};
    std::array<double, 12> P0 = {457.0, 0, 320.0, 0, 0, 457.0, 240.0, 0, 0, 0, 1, 0};
    std::array<double, 9> R0 =  {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 };

    std::array<double, 9> K1 = {457.00, 0, 320.00, 0, 457.00, 240.0, 0,0,1} ;   
    std::vector<double> d1 = {0,0,0,0,0};
    std::array<double, 12> P1 = {457.0, 0, 320.0, -114.26, 0, 457.0, 240.0, 0, 0, 0, 1, 0};
    std::array<double, 9> R1 =  {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 };

    const auto tp_1 = std::chrono::steady_clock::now();
    const auto timestamp = std::chrono::duration_cast<std::chrono::duration<double>>(tp_1 - tp_0).count();
    //auto cam = SLAM->feed_monocular_frame(cv_bridge::toCvShare(left_image_msg_sub, "bgr8")->image, timestamp, mask);
    auto cam =  SLAM->feed_stereo_frame(cv_bridge::toCvShare(left_image_msg_sub, "bgr8")->image, 
                                        cv_bridge::toCvShare(right_image_msg_sub, "bgr8")->image, timestamp, mask);
    publi(cam, odometry_publisher, node);
    if(cam_position[0] != 0.0){
        //LEFT image
        left_image_msg->data = left_image_msg_sub->data;
        left_image_msg->encoding = left_image_msg_sub->encoding;
        left_image_msg->step = left_image_msg_sub->step;
        left_image_msg->height = left_image_msg_sub->height;
        left_image_msg->width = left_image_msg_sub->width;
        left_image_msg->header.frame_id = "camera_link";
        left_image_msg->header.stamp = odom_msg_->header.stamp;
        
        //RIGHT image
        right_image_msg->data = right_image_msg_sub->data;
        right_image_msg->encoding = right_image_msg_sub->encoding;
        right_image_msg->step = right_image_msg_sub->step;
        right_image_msg->height = right_image_msg_sub->height;
        right_image_msg->width = right_image_msg_sub->width;
        right_image_msg->header.frame_id = "camera_link";
        right_image_msg->header.stamp = odom_msg_->header.stamp;

        // LEFT CONFIG
        left_config->header.stamp = odom_msg_->header.stamp;
        left_config->header.frame_id = "camera_link";
        left_config->width = left_image_msg->width;
        left_config->height = left_image_msg->height;
        left_config->k = K0;
        left_config->d = d0;
        left_config->r = R0;
        left_config->p = P0;
        left_config->distortion_model = "plumb_bob";

        // RIGHT CONFIG
        right_config->header.stamp = odom_msg_->header.stamp;
        right_config->header.frame_id = "camera_link";
        right_config->width = right_image_msg->width;
        right_config->height = right_image_msg->height;
        right_config->k = K1;
        right_config->d = d1;
        right_config->r = R1;
        right_config->p = P1;
        right_config->distortion_model = "plumb_bob";

        // PUBLICATIONS
        odometry_publisher->publish(*odom_msg_);
        right_publisher->publish(*right_image_msg);
        left_publisher->publish(*left_image_msg);
        right_publisher_config->publish(*right_config);
        left_publisher_config->publish(*left_config);
    }
}

void mono_tracking(const std::shared_ptr<openvslam::config>& cfg, const std::string& vocab_file_path,
                   const std::string& mask_img_path, const bool eval_log, const std::string& map_db_path) {
    // load the mask image
    //const cv::Mat mask = mask_img_path.empty() ? cv::Mat{} : cv::imread(mask_img_path, cv::IMREAD_GRAYSCALE);

    // build a SLAM system
    openvslam::system SLAM(cfg, vocab_file_path);
    // startup the SLAM process
    SLAM.startup();

    // create a viewer object
    // and pass the frame_publisher and the map_publisher
#ifdef USE_PANGOLIN_VIEWER
    pangolin_viewer::viewer viewer(cfg, &SLAM, SLAM.get_frame_publisher(), SLAM.get_map_publisher());
#elif USE_SOCKET_PUBLISHER
    socket_publisher::publisher publisher(cfg, &SLAM, SLAM.get_frame_publisher(), SLAM.get_map_publisher());
#endif

    std::vector<double> track_times;
    tp_0 = std::chrono::steady_clock::now();

    // initialize this node
    auto node = std::make_shared<rclcpp::Node>("run_slam");

    // eloquent - eloquent
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster;
    tf_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);
    //static tf2_ros::StaticTransformBroadcaster tf_broadcaster(node);
    tf_msg->header.stamp = node->now();
    tf_msg->header.frame_id = "base_link";
    tf_msg->child_frame_id = "camera_link";
    tf_msg->transform.translation.x = 0.0;
    tf_msg->transform.translation.y = 0.0;
    tf_msg->transform.translation.z = 0.0;
    tf_msg->transform.rotation.x = -0.5;
    tf_msg->transform.rotation.y = 0.5;
    tf_msg->transform.rotation.z = -0.5;
    tf_msg->transform.rotation.w = 0.5;
    tf_broadcaster->sendTransform(*tf_msg);
    
    auto left_publisher = node->create_publisher<sensor_msgs::msg::Image>("/left/image_rect", 1);
    auto right_publisher = node->create_publisher<sensor_msgs::msg::Image>("/right/image_rect", 1);
    auto left_publisher_config = node->create_publisher<sensor_msgs::msg::CameraInfo>("/left/camera_info", 1);
    auto right_publisher_config = node->create_publisher<sensor_msgs::msg::CameraInfo>("/right/camera_info", 1);
    auto odometry_publisher = node->create_publisher<nav_msgs::msg::Odometry>("/odom", 100);
    // SLAM system subscribe LEFT and RIGHT image
    message_filters::Subscriber<sensor_msgs::msg::Image> left_image(node, "/image/left");
    message_filters::Subscriber<sensor_msgs::msg::Image> right_image(node, "/image/right");
    message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::Image> sync(left_image, right_image, 10);
    
    sync.registerCallback(boost::bind(&callback, _1, _2, left_publisher, right_publisher, 
                                            left_publisher_config, right_publisher_config, odometry_publisher, &SLAM, node));

    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);

    // Pangolin needs to run in the main thread on OSX
    std::thread thread([&]() {
        exec.spin();
    });

#ifdef USE_PANGOLIN_VIEWER
    viewer.run();
    if (SLAM.terminate_is_requested()) {
        // wait until the loop BA is finished
        while (SLAM.loop_BA_is_running()) {
            std::this_thread::sleep_for(std::chrono::microseconds(5000));
        }
        rclcpp::shutdown();
    }
#elif USE_SOCKET_PUBLISHER
    publisher.run();
    if (SLAM.terminate_is_requested()) {
        // wait until the loop BA is finished
        while (SLAM.loop_BA_is_running()) {
            std::this_thread::sleep_for(std::chrono::microseconds(5000));
        }
        rclcpp::shutdown();
    }
#endif

    // automatically close the viewer
#ifdef USE_PANGOLIN_VIEWER
    viewer.request_terminate();
    thread.join();
#elif USE_SOCKET_PUBLISHER
    publisher.request_terminate();
    thread.join();
#endif

    // shutdown the SLAM process
    SLAM.shutdown();

    if (eval_log) {
        // output the trajectories for evaluation
        SLAM.save_frame_trajectory("frame_trajectory.txt", "TUM");
        SLAM.save_keyframe_trajectory("keyframe_trajectory.txt", "TUM");
    }

    if (!map_db_path.empty()) {
        // output the map database
        SLAM.save_map_database(map_db_path);
    }

}

int main(int argc, char* argv[]) {
#ifdef USE_STACK_TRACE_LOGGER
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
#endif
    rclcpp::init(argc, argv);

    // create options
    popl::OptionParser op("Allowed options");
    auto help = op.add<popl::Switch>("h", "help", "produce help message");
    auto vocab_file_path = op.add<popl::Value<std::string>>("v", "vocab", "vocabulary file path");
    auto setting_file_path = op.add<popl::Value<std::string>>("c", "config", "setting file path");
    auto mask_img_path = op.add<popl::Value<std::string>>("", "mask", "mask image path", "");
    auto debug_mode = op.add<popl::Switch>("", "debug", "debug mode");
    auto eval_log = op.add<popl::Switch>("", "eval-log", "store trajectory and tracking times for evaluation");
    auto map_db_path = op.add<popl::Value<std::string>>("", "map-db", "store a map database at this path after SLAM", "");
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
    if (!vocab_file_path->is_set() || !setting_file_path->is_set()) {
        std::cerr << "invalid arguments" << std::endl;
        std::cerr << std::endl;
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }

    // setup logger
    spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] %^[%L] %v%$");
    if (debug_mode->is_set()) {
        spdlog::set_level(spdlog::level::debug);
    }
    else {
        spdlog::set_level(spdlog::level::info);
    }

    // load configuration
    std::shared_ptr<openvslam::config> cfg;
    try {
        cfg = std::make_shared<openvslam::config>(setting_file_path->value());
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

#ifdef USE_GOOGLE_PERFTOOLS
    ProfilerStart("slam.prof");
#endif

    // run tracking
    if (cfg->camera_->setup_type_ == openvslam::camera::setup_type_t::Monocular) {
        mono_tracking(cfg, vocab_file_path->value(), mask_img_path->value(), eval_log->is_set(), map_db_path->value());
    }
    else if (cfg->camera_->setup_type_ == openvslam::camera::setup_type_t::Stereo) {
        mono_tracking(cfg, vocab_file_path->value(), mask_img_path->value(), eval_log->is_set(), map_db_path->value());
    }
    else {
        throw std::runtime_error("Invalid setup type: " + cfg->camera_->get_setup_type_string());
    }

#ifdef USE_GOOGLE_PERFTOOLS
    ProfilerStop();
#endif

    return EXIT_SUCCESS;
}
