#ifndef PERCEPTION_H
#define PERCEPTION_H

#include "Utils.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

#include <vector>
#include <random>

class Perception
{
public:
    Perception();
    
    void MCL_sampling(const Action &u);
    void MCL_weighting(const std::vector<float> &z);
    void MCL_resampling();

    void MCL_initialize();
    void MCL_updateParticles(geometry_msgs::msg::PoseArray &msg_particles, rclcpp::Time now);

    bool hasReceivedMap();
    bool hasStartedMCL();

    std::vector<float> getLatestLaserRanges();
    void receiveLaser(const sensor_msgs::msg::LaserScan::ConstSharedPtr &value);
    void receiveGridmap(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr &value);

private:
    float computeExpectedMeasurement(int index, Pose2D &pose);

    sensor_msgs::msg::LaserScan laserROS_;
    nav_msgs::msg::OccupancyGrid gridMap_;

    int numCellsX_;
    int numCellsY_;
    float mapWidth_;
    float mapHeight_;
    float scale_;
    Pose2D mapOrigin_;
    float maxRange_;

    bool receivedMap_;
    bool startedMCL_;

    int numParticles_;
    std::vector<Particle> particles_;
    std::default_random_engine* generator_;
};

#endif // PERCEPTION_H
