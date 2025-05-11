#ifndef PERCEPTION_H
#define PERCEPTION_H

#include "Utils.h"

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <vector>

class Perception
{
public:
    Perception(float mapWidth, float mapHeight, float cellSize);
    
    std::vector<float> getLatestLaserRanges();
    std::vector<float> getLatestSonarRanges();
    Pose2D getLatestPoseFromOdometry();
    
    void receiveOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr &value);
    void receiveLaser(const sensor_msgs::msg::LaserScan::ConstSharedPtr &value);
    void receiveSonar(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &value);

    nav_msgs::msg::OccupancyGrid& updateMapLaserWithHIMM(const std::vector<float>& z, const Pose2D& robot);
    nav_msgs::msg::OccupancyGrid& updateMapLaserWithLogOdds(const std::vector<float>& z, const Pose2D& robot);
    nav_msgs::msg::OccupancyGrid& updateMapSonar(const std::vector<float>& z, const Pose2D& robot);

private:
    int getNearestSonarBeam(float angle);
    float getAngleOfSonarBeam(int k);
    int getNearestLaserBeam(float angle);
    float getAngleOfLaserBeam(int k);
    int getCellIndexFromXY(int x, int y);

    nav_msgs::msg::Odometry odomROS;
    sensor_msgs::msg::LaserScan laserROS;
    sensor_msgs::msg::PointCloud2 sonarROS;

    int numCellsX_;
    int numCellsY_;

    float mapWidth_;
    float mapHeight_;
    float scale_;

    std::vector<int> gridLaserHIMM_;
    std::vector<float> gridLaserLogOdds_;
    std::vector<float> gridSonar_;

    nav_msgs::msg::OccupancyGrid msg_mapLaserLogOdds_;
    nav_msgs::msg::OccupancyGrid msg_mapLaserHIMM_;
    nav_msgs::msg::OccupancyGrid msg_mapSonar_;

};

#endif // PERCEPTION_H
