#ifndef PERCEPTION_H
#define PERCEPTION_H

#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include "Utils.h"

#define OCC_OCCUPIED 100
#define OCC_NEAROBSTACLE 90
#define OCC_FREE 50
#define OCC_FRONTIER 30
#define OCC_UNEXPLORED -1

#define PLAN_GOALS 0
#define PLAN_MARKEDGOALS 100
#define PLAN_REGULAR 40
#define PLAN_PATH 10
#define PLAN_INVALID -1

class Perception
{
public:
    Perception();
    
    bool hasValidDirection();
    double getDirectionOfNavigation();
    Pose2D getLatestPoseFromOdometry();

    bool hasValidMap();

    geometry_msgs::msg::PoseStamped& getDirectionOfNavigationMsg();
    nav_msgs::msg::OccupancyGrid& getOccTypesMapMsg();
    nav_msgs::msg::OccupancyGrid& getPlanTypesMapMsg();

    void receiveOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr &value);
    void receiveGridmap(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr &value);

private:

    void updateCellsClassification();
    void computeHeuristic(int goalIndex);
    int computeShortestPathToFrontier(int robotIndex);
    
    void updateGridKnownLimits();
    int clusterFrontiersAndReturnIndexOfClosestOne(int robotIndex);
    void markPathCells(int goal);
    double computeDirectionOfNavigation(int robotIndex, int goalIndex);
    int getNearestFreeCell(Pose2D robotPose);

    bool started_;
    bool validDirection_;

    unsigned int numCellsX_;
    unsigned int numCellsY_;
    float mapWidth_;
    float mapHeight_;
    float scale_;
    unsigned int minKnownX_, minKnownY_, maxKnownX_, maxKnownY_;

    std::vector<int8_t> occupancyTypeGrid_;
    std::vector<int8_t> planningTypeGrid_;
    std::vector<double> fValueGrid_;
    std::vector<double> gValueGrid_;
    std::vector<double> hValueGrid_;
    std::vector<int> parentGrid_;

    std::vector<int> frontierCentersIndices;

    nav_msgs::msg::Odometry odomROS_;

    double directionOfNavigation_;
    nav_msgs::msg::OccupancyGrid msg_occTypes_;
    nav_msgs::msg::OccupancyGrid msg_planTypes_;
    geometry_msgs::msg::PoseStamped msg_directionOfNavigation_;

};

#endif // PERCEPTION_H
