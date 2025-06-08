#include <iostream>
#include <pthread.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "Perception.h"
#include "Utils.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

class LocalizationNode : public rclcpp::Node
{
public:
  LocalizationNode(Perception &perception)
      : Node("localization_p3dx"), perception_(perception)
  {

    previousOdom_.x = 0;
    previousOdom_.y = 0;
    previousOdom_.theta = 0;

    // Initialize tf listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    rmw_qos_profile_t map_topic_qos = rmw_qos_profile_default;
    map_topic_qos.depth=5;
    map_topic_qos.history=RMW_QOS_POLICY_HISTORY_KEEP_LAST;              // alternative to: RMW_QOS_POLICY_HISTORY_KEEP_ALL
    map_topic_qos.reliability=RMW_QOS_POLICY_RELIABILITY_RELIABLE;       // alternative to: RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT
    map_topic_qos.durability=RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;  // alternative to: RMW_QOS_POLICY_DURABILITY_VOLATILE
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(map_topic_qos.history, map_topic_qos.depth), map_topic_qos);

    // Initialize subscribers
    sub_laserscan = this->create_subscription<sensor_msgs::msg::LaserScan>("/laser_scan", 100,
                                                                           std::bind(&Perception::receiveLaser, &perception, _1));
    sub_gridmap_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", qos,
                                                                           std::bind(&Perception::receiveGridmap, &perception, _1));

    // Initialize publishers
    pub_particles_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/particles", 1);

    // Initialize timer callback
    timer_ = this->create_wall_timer(100ms, std::bind(&LocalizationNode::timer_callback, this));
  }

private:
  void timer_callback()
  {
    RCLCPP_INFO(this->get_logger(), "Running");

    if(perception_.hasReceivedMap())
    {
      if(perception_.hasStartedMCL() == false)
      {
          perception_.MCL_initialize();
      }
      else
      {

        // Get latest sensor readings
        std::vector<float> z = perception_.getLatestLaserRanges();
        RCLCPP_INFO(this->get_logger(), "Read %ld laser measurements", z.size());

        // Get current robot pose given by ODOM
        Pose2D currentOdom;
        geometry_msgs::msg::TransformStamped transformStamped;
        bool validPose=true;
        try{ 
          transformStamped = tf_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero); 
        } catch (const tf2::TransformException & ex) { 
          RCLCPP_INFO(this->get_logger(), "Could not transform 'odom' to 'base_link': %s", ex.what());
          validPose=false;
        }

        if(validPose){
          // Update currentOdom from robot transformation
          currentOdom.x = transformStamped.transform.translation.x;
          currentOdom.y = transformStamped.transform.translation.y;
          // Convert quaternion to euler angles
          tf2::Quaternion q4(transformStamped.transform.rotation.x,
                            transformStamped.transform.rotation.y, 
                            transformStamped.transform.rotation.z, 
                            transformStamped.transform.rotation.w);
          tf2::Matrix3x3 m4(q4);
          double roll, pitch, yaw;
          m4.getRPY(roll,pitch,yaw);
          // Update  currentOdom orientation with yaw
          currentOdom.theta = RAD2DEG(yaw);
          RCLCPP_INFO(this->get_logger(), "Current robot pose - odom (%f %f %f)", currentOdom.x, currentOdom.y, currentOdom.theta);

          // Compute Action u, based on odometry variation
          Action u;
          u.rot1 = atan2(currentOdom.y-previousOdom_.y,currentOdom.x-previousOdom_.x)-DEG2RAD(currentOdom.theta);
          u.trans = sqrt(pow(currentOdom.x-previousOdom_.x,2)+pow(currentOdom.y-previousOdom_.y,2));
          u.rot2 = DEG2RAD(currentOdom.theta)-DEG2RAD(previousOdom_.theta)-u.rot1;

          // check if there is enough robot motion
          if(u.trans > 0.1 || fabs(normalizeAngleDEG(currentOdom.theta-previousOdom_.theta)) > 30)
          {
              perception_.MCL_sampling(u);
              perception_.MCL_weighting(z);
              perception_.MCL_resampling();

              previousOdom_ = currentOdom;
          }

          geometry_msgs::msg::PoseArray msg_particles;
          perception_.MCL_updateParticles(msg_particles, this->get_clock()->now());
          pub_particles_->publish(msg_particles);
          RCLCPP_INFO(this->get_logger(), "Published particles");
        }  
      }         
    }
  }

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laserscan;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_gridmap_;

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_particles_;

  Perception &perception_;
  Pose2D previousOdom_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  Perception perception;

  std::shared_ptr<LocalizationNode> nh = std::make_shared<LocalizationNode>(perception);
  rclcpp::spin(nh);

  rclcpp::shutdown();

  return 0;
}
