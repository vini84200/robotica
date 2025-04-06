#ifndef PHIROS2ARIA_HPP
#define PHIROS2ARIA_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#ifdef ADEPT_PKG
  #include <Aria.h>
#else
  #include <Aria/Aria.h>
#endif

#include "Pose.hpp"

/** @brief Node that interfaces between ROS and mobile robot base features via ARIA library. 

    PhiROS2Aria will use ARIA to connect to a robot controller (configure via ~port parameter), 
    either direct serial connection or over the network.  
    It runs ARIA's robot communications cycle in a background thread, 
    and as part of that cycle (a sensor interpretation task which calls PhiROS2Aria::publish()),
    it  publishes various topics with newly received robot data.  
    It also sends velocity commands to the robot when received in the
    cmd_vel topic, and handles dynamic_reconfigure and Service requests.
*/

class PhiROS2Aria : public rclcpp::Node
{
  public:
    PhiROS2Aria();
    bool Setup();
    void publish();

  private:

    void cmdvel_cb( const geometry_msgs::msg::Twist::ConstSharedPtr &msg);

    // ARIA variables
    ArRobotConnector *conn;
    ArLaserConnector *laserConnector;
    ArRobot *robot;
    ArSick *sick;
    ArPose pos;
    ArFunctorC<PhiROS2Aria> myPublishCB;
//    LaserPublisher* laserPublisher;
    ArLaser *laser;

    bool isSimulation_;

    // Parameters
    std::string serial_port;
    int serial_baud;
    // Debug Aria
    bool debug_aria;
    std::string aria_log_filename;
    // whether to publish aria lasers
    bool publish_aria_lasers;
    // flag indicating whether sonar was enabled or disabled on the robot
    bool sonar_enabled; 
    // enable and publish sonar topic. set to true when first subscriber connects, set to false when last subscriber disconnects. 
    bool publish_sonar; 

    std::string frame_id_odom;
    std::string frame_id_base_link;
    std::string frame_id_sonar;
    std::string frame_id_laser;

    // True Pose from MobileSim
    Pose getAbsoluteTruePose();
    Pose getRelativeTruePose();
    ArGlobalRetFunctor1<bool, ArRobotPacket *>  simStatHandler_;
    static bool simStatPacketHandler(ArRobotPacket* packet);

    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pose_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr absTruePose_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr relTruePose_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr sonar_pub;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laserScan_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr laserCloud_pub;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    sensor_msgs::msg::LaserScan laserScanMsg_;
    rclcpp::Time veltime;

    // rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_imu_;
    // message_filters::Subscriber<sensor_msgs::msg::Imu> subscription_imu_corrected_;
    // std::shared_ptr<message_filters::TimeSequencer<sensor_msgs::msg::Imu>> seq_;
    // rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_filtered_;
    
};

#endif //PHIROS2ARIA_HPP
