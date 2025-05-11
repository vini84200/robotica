#include <iostream>
#include <pthread.h>

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "Perception.h"
#include "Utils.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

class MappingNode : public rclcpp::Node
{
public:
  MappingNode(Perception &perception)
      : Node("mapping_p3dx"), perception_(perception)
  {
    // Initialize subscribers
    sub_odometry = this->create_subscription<nav_msgs::msg::Odometry>("/pose", 100,
                                                                           std::bind(&Perception::receiveOdometry, &perception, _1));
    sub_laserscan = this->create_subscription<sensor_msgs::msg::LaserScan>("/laser_scan", 100,
                                                                           std::bind(&Perception::receiveLaser, &perception, _1));
    sub_sonar = this->create_subscription<sensor_msgs::msg::PointCloud2>("/sonar", 100,
                                                                         std::bind(&Perception::receiveSonar, &perception, _1));

    // Initialize publishers
    pub_mapLaserLogOdds_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/mapa_laser_log_odds", 1);
    pub_mapLaserHIMM_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/mapa_laser_HIMM", 1);
    pub_mapSonar_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/mapa_sonar", 1);

    // Initialize timer callback
    timer_ = this->create_wall_timer(100ms, std::bind(&MappingNode::timer_callback, this));
  }

private:
  void timer_callback()
  {
    // Get latest sensor readings
    std::vector<float> lasers = perception_.getLatestLaserRanges();
    std::vector<float> sonars = perception_.getLatestSonarRanges();
    RCLCPP_INFO(this->get_logger(), "Read %ld laser measurements", lasers.size());
    RCLCPP_INFO(this->get_logger(), "Read %ld sonar measurements", sonars.size());

    // Get current robot pose given by ODOM
    Pose2D robotPose = perception_.getLatestPoseFromOdometry();
    RCLCPP_INFO(this->get_logger(), "Odometry (%f %f %f)", robotPose.x, robotPose.y, robotPose.theta);

    if(lasers.size() > 0){
      // Update maps
      nav_msgs::msg::OccupancyGrid& msg_mapLaserLogOdds = perception_.updateMapLaserWithLogOdds(lasers, robotPose);
      nav_msgs::msg::OccupancyGrid& msg_mapLaserHIMM = perception_.updateMapLaserWithHIMM(lasers, robotPose);
      nav_msgs::msg::OccupancyGrid& msg_mapSonar = perception_.updateMapSonar(sonars, robotPose);

      // Publish maps
      pub_mapLaserLogOdds_->publish(msg_mapLaserLogOdds);
      pub_mapLaserHIMM_->publish(msg_mapLaserHIMM);
      pub_mapSonar_->publish(msg_mapSonar);
      RCLCPP_INFO(this->get_logger(), "Published maps");
    }
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laserscan;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_sonar;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_mapLaserLogOdds_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_mapLaserHIMM_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_mapSonar_;

  Perception &perception_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  Perception perception(100, 100, 0.1);

  std::shared_ptr<MappingNode> nh = std::make_shared<MappingNode>(perception);
  rclcpp::spin(nh);

  rclcpp::shutdown();

  return 0;
}
