#include <iostream>
#include <pthread.h>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float32.hpp>

#include "Action.h"
#include "Perception.h"
#include "Utils.h"

using std::placeholders::_1;
using namespace std::chrono_literals;
char pressedKey;

class NavigationNode : public rclcpp::Node
{
public:
  NavigationNode(Action &action, Perception &perception)
      : Node("navigation_p3dx"), action_(action), perception_(perception)
  {
    pub_twist = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
    sub_laserscan = this->create_subscription<sensor_msgs::msg::LaserScan>("/laser_scan", 100,
                                                                           std::bind(&Perception::receiveLaser, &perception, _1));
    sub_sonar = this->create_subscription<sensor_msgs::msg::PointCloud2>("/sonar", 100,
                                                                         std::bind(&Perception::receiveSonar, &perception, _1));
    timer_ = this->create_wall_timer(100ms, std::bind(&NavigationNode::timer_callback, this));
    // Debug publishers
    error_ = this->create_publisher<std_msgs::msg::Float32>("/error", 1);
    derivative_error_ = this->create_publisher<std_msgs::msg::Float32>("/derivative_error", 1);
    integrated_error_ = this->create_publisher<std_msgs::msg::Float32>("/integrated_error", 1);
    pid_ = this->create_publisher<std_msgs::msg::Float32>("/pid", 1);
  }

private:
  void pid_info_callback(float error, float derivative, float integral, float pid)
  {
    std_msgs::msg::Float32 error_msg;
    error_msg.data = error;
    error_->publish(error_msg);

    std_msgs::msg::Float32 derivative_error_msg;
    derivative_error_msg.data = derivative;
    derivative_error_->publish(derivative_error_msg);

    std_msgs::msg::Float32 integrated_error_msg;
    integrated_error_msg.data = integral;
    integrated_error_->publish(integrated_error_msg);

    std_msgs::msg::Float32 pid_msg;
    pid_msg.data = pid;
    pid_->publish(pid_msg);

  }

  void timer_callback()
  {
    // Get latest sensor readings
    std::vector<float> lasers = perception_.getLatestLaserRanges();
    std::vector<float> sonars = perception_.getLatestSonarRanges();
    std::cout << "Read " << lasers.size() << " laser measurements" << std::endl;
    std::cout << "Read " << sonars.size() << " sonar measurements" << std::endl;

    // Get keyboard input
    char ch = pressedKey;
    MotionControl mc = action_.handlePressedKey(ch);
    std::cout << mc.mode << ' ' << mc.direction << std::endl;

    // Compute next action
    if (mc.mode == MANUAL)
    {
      action_.manualRobotMotion(mc.direction);
    }
    else if (mc.mode == WANDER)
    {
      action_.avoidObstacles(lasers, sonars);
    }
    else if (mc.mode == FARFROMWALLS)
    {
      action_.keepAsFarthestAsPossibleFromWalls(lasers, sonars, std::bind(&NavigationNode::pid_info_callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
    }

    action_.correctVelocitiesIfInvalid();

    // Publish robot motion
    geometry_msgs::msg::Twist twistROS;
    twistROS.linear.x = action_.getLinearVelocity();
    twistROS.angular.z = action_.getAngularVelocity();
    pub_twist->publish(twistROS);
    std::cout << "Published linVel " << twistROS.linear.x << " angVel " << twistROS.angular.z << std::endl;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_twist;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laserscan;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_sonar;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr error_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr derivative_error_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr integrated_error_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pid_;

  Action &action_;
  Perception &perception_;
};

void *keyboardThreadFunction(void *arg)
{
  while (pressedKey != 27)
  {
    pressedKey = getCharWithoutWaitingENTER();
    std::cout << "PressedKey: " << pressedKey << std::endl;
  }

  std::cout << "Ending: " << pressedKey << std::endl;
  // exit(0); // force exit

  return NULL;
}

void *mainThreadFunction(void *arg)
{
  Action action;
  Perception perception;

  std::shared_ptr<NavigationNode> nh = std::make_shared<NavigationNode>(action, perception);
  rclcpp::spin(nh);

  return NULL;
}


int main(int argc, char **argv)
{
  pressedKey = 'x';

  rclcpp::init(argc, argv);

  pthread_t mainThread, keyboardThread;

  pthread_create(&(mainThread), NULL, mainThreadFunction, NULL);
  pthread_create(&(keyboardThread), NULL, keyboardThreadFunction, NULL);

  pthread_join(mainThread, 0);
  pthread_join(keyboardThread, 0);

  rclcpp::shutdown();

  return 0;
}
