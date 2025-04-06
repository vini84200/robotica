#ifndef PERCEPTION_H
#define PERCEPTION_H

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class Perception
{
public:
    Perception();
    
    std::vector<float> getLatestLaserRanges();
    std::vector<float> getLatestSonarRanges();
    
    void receiveLaser(const sensor_msgs::msg::LaserScan::ConstSharedPtr &value);
    void receiveSonar(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &value);

private:
    sensor_msgs::msg::LaserScan laserROS;
    sensor_msgs::msg::PointCloud2 sonarROS;

};

#endif // PERCEPTION_H
