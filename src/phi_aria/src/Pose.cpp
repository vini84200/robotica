#include "Pose.hpp"

#include <iomanip>

Pose::Pose(){
    x=y=theta=0.0;
    up=false;
}

Pose::Pose(double a, double b, double c){
    x=a; y=b; theta=c;
}

std::ostream& operator<<(std::ostream& os, const Pose& p)
{
    os << std::setprecision(3) << "(" << p.x << ',' << p.y << ',' << p.theta << ")";
    return os;
}