#include <iostream>
#include <pthread.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "Action.h"
#include "Perception.h"
#include "Utils.h"

using std::placeholders::_1;
using namespace std::chrono_literals;
char pressedKey;

class ExplorationNode : public rclcpp::Node
{
public:
  ExplorationNode(Action &action, Perception &perception)
      : Node("exploration"), action_(action), perception_(perception)
  {
    // Initialize subscribers
    sub_odometry = this->create_subscription<nav_msgs::msg::Odometry>("/pose", 100,
                                                                      std::bind(&Perception::receiveOdometry, &perception, _1));
    sub_gridmap_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/mapa_laser_HIMM", 1,
                                                                           std::bind(&Perception::receiveGridmap, &perception, _1));

    // Initialize publishers
    pub_twist_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
    pub_mapOccType_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/mapa_occ_types", 1);
    pub_mapPlanType_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/mapa_plan_types", 1);
    pub_directionOfNavigation_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/direction_navigation", 1);

    // Initialize timer callback
    timer_ = this->create_wall_timer(100ms, std::bind(&ExplorationNode::timer_callback, this));
  }

private:
  void timer_callback()
  {
    // Get current robot pose given by ODOM
    Pose2D robotPose = perception_.getLatestPoseFromOdometry();
    RCLCPP_INFO(this->get_logger(), "Odometry (%f %f %f)", robotPose.x, robotPose.y, robotPose.theta);

    if(perception_.hasValidMap()){
      // Update potential field information
      nav_msgs::msg::OccupancyGrid& msg_occTypes = perception_.getOccTypesMapMsg();
      nav_msgs::msg::OccupancyGrid& msg_planTypes = perception_.getPlanTypesMapMsg();
      geometry_msgs::msg::PoseStamped& msg_directionOfNavigation = perception_.getDirectionOfNavigationMsg();

      // Publish maps
      pub_mapOccType_->publish(msg_occTypes);
      pub_mapPlanType_->publish(msg_planTypes);
      pub_directionOfNavigation_->publish(msg_directionOfNavigation);
      RCLCPP_INFO(this->get_logger(), "Published maps");
    }

    // Get keyboard input
    char ch = pressedKey;
    MotionControl mc = action_.handlePressedKey(ch);
    std::cout << mc.mode << ' ' << mc.direction << std::endl;

    // Compute next action
    if (mc.mode == MANUAL)
    {
      action_.manualRobotMotion(mc.direction);
    }
    else if (mc.mode == EXPLORE)
    {
      if(perception_.hasValidDirection()){
          double angle = perception_.getDirectionOfNavigation();
          action_.followDirection(angle);
      }else{
          action_.stopRobot();   
      }
    }
  
    action_.correctVelocitiesIfInvalid();

    // Publish robot motion
    geometry_msgs::msg::Twist twistROS;
    twistROS.linear.x = action_.getLinearVelocity();
    twistROS.angular.z = action_.getAngularVelocity();
    pub_twist_->publish(twistROS);
    std::cout << "Published linVel " << twistROS.linear.x << " angVel " << twistROS.angular.z << std::endl;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_gridmap_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_twist_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_mapOccType_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_mapPlanType_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_directionOfNavigation_;

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

  std::shared_ptr<ExplorationNode> nh = std::make_shared<ExplorationNode>(action, perception);
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
