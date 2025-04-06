#ifndef POSE_HPP
#define POSE_HPP

#include <sstream>

#define DEG2RAD(x) x*M_PI/180.0
#define RAD2DEG(x) x*180.0/M_PI

class Pose{
    public:
        Pose();
        Pose(double a, double b, double c);

        friend std::ostream& operator<<(std::ostream& os, const Pose& p);

        double x, y, theta;
        bool up;
};

#endif //POSE_HPP