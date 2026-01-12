/**
 * @file aerosimx_bridge_node.cpp
 * @brief Main ROS2 bridge node for AeroSimX
 */

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.hpp>

namespace aerosimx_bridge {

/**
 * @brief Main ROS2 bridge node
 *
 * Provides bidirectional communication between AeroSimX simulation
 * and ROS2 ecosystem.
 */
class AeroSimXBridgeNode : public rclcpp::Node {
public:
  AeroSimXBridgeNode() : Node("aerosimx_bridge") {
    // Declare parameters
    this->declare_parameter<std::string>("host", "localhost");
    this->declare_parameter<int>("port", 41451);
    this->declare_parameter<double>("update_rate", 100.0);
    this->declare_parameter<std::string>("vehicle_name", "drone1");
    this->declare_parameter<bool>("publish_tf", true);

    // Get parameters
    host_ = this->get_parameter("host").as_string();
    port_ = this->get_parameter("port").as_int();
    update_rate_ = this->get_parameter("update_rate").as_double();
    vehicle_name_ = this->get_parameter("vehicle_name").as_string();
    publish_tf_ = this->get_parameter("publish_tf").as_bool();

    // Initialize publishers
    init_publishers();

    // Initialize subscribers
    init_subscribers();

    // Initialize TF broadcaster
    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    // Create timer for main update loop
    auto period = std::chrono::duration<double>(1.0 / update_rate_);
    timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        std::bind(&AeroSimXBridgeNode::update_callback, this));

    RCLCPP_INFO(this->get_logger(),
                "AeroSimX Bridge started. Connecting to %s:%d", host_.c_str(),
                port_);
  }

private:
  void init_publishers() {
    // Lidar point cloud
    lidar_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "aerosimx/lidar", 10);

    // Camera images
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "aerosimx/camera/image_raw", 10);
    depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "aerosimx/camera/depth", 10);

    // IMU
    imu_pub_ =
        this->create_publisher<sensor_msgs::msg::Imu>("aerosimx/imu", 10);

    // GPS
    gps_pub_ =
        this->create_publisher<sensor_msgs::msg::NavSatFix>("aerosimx/gps", 10);

    // Odometry
    odom_pub_ =
        this->create_publisher<nav_msgs::msg::Odometry>("aerosimx/odom", 10);

    // Pose
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "aerosimx/pose", 10);
  }

  void init_subscribers() {
    // Velocity command
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "aerosimx/cmd_vel", 10,
        std::bind(&AeroSimXBridgeNode::cmd_vel_callback, this,
                  std::placeholders::_1));

    // Target pose command
    target_pose_sub_ =
        this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "aerosimx/target_pose", 10,
            std::bind(&AeroSimXBridgeNode::target_pose_callback, this,
                      std::placeholders::_1));
  }

  void update_callback() {
    // In production, this would:
    // 1. Get data from AeroSimX simulation
    // 2. Convert to ROS messages
    // 3. Publish on topics

    auto now = this->get_clock()->now();

    // Publish placeholder IMU
    auto imu_msg = sensor_msgs::msg::Imu();
    imu_msg.header.stamp = now;
    imu_msg.header.frame_id = vehicle_name_ + "/imu";
    imu_msg.linear_acceleration.z = 9.81; // Gravity
    imu_pub_->publish(imu_msg);

    // Publish placeholder pose
    auto pose_msg = geometry_msgs::msg::PoseStamped();
    pose_msg.header.stamp = now;
    pose_msg.header.frame_id = "world";
    pose_msg.pose.orientation.w = 1.0;
    pose_pub_->publish(pose_msg);

    // Publish TF
    if (publish_tf_) {
      publish_tf(now);
    }
  }

  void publish_tf(const rclcpp::Time &stamp) {
    geometry_msgs::msg::TransformStamped transform;

    transform.header.stamp = stamp;
    transform.header.frame_id = "world";
    transform.child_frame_id = vehicle_name_ + "/base_link";

    // In production, get from simulation
    transform.transform.translation.x = 0.0;
    transform.transform.translation.y = 0.0;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation.w = 1.0;

    tf_broadcaster_->sendTransform(transform);
  }

  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    // Send velocity command to simulation
    RCLCPP_DEBUG(this->get_logger(),
                 "Received cmd_vel: linear=[%.2f, %.2f, %.2f] angular=[%.2f, "
                 "%.2f, %.2f]",
                 msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.x,
                 msg->angular.y, msg->angular.z);

    // In production: client_->move_by_velocity(...)
  }

  void
  target_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    // Send target pose to simulation
    RCLCPP_DEBUG(this->get_logger(), "Received target_pose: [%.2f, %.2f, %.2f]",
                 msg->pose.position.x, msg->pose.position.y,
                 msg->pose.position.z);

    // In production: client_->move_to_position(...)
  }

  // Parameters
  std::string host_;
  int port_;
  double update_rate_;
  std::string vehicle_name_;
  bool publish_tf_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      target_pose_sub_;

  // TF
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace aerosimx_bridge

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<aerosimx_bridge::AeroSimXBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
