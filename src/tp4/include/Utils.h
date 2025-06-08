#ifndef UTILS_H
#define UTILS_H

#include <cmath>
#include <iostream>

#define DEG2RAD(x) x*M_PI/180.0
#define RAD2DEG(x) x*180.0/M_PI

float normalizeAngleDEG(float a);
float normalizeAngleRAD(float a);

float getLikelihoodFromLogOdds(float logodds);
float getLogOddsFromLikelihood(float likelihood);

char getCharWithoutWaitingENTER(void);

typedef struct{
    float x, y, theta;
} Pose2D;

typedef struct{
    double rot1;
    double trans;
    double rot2;
} Action;

typedef struct
{
    Pose2D p;
    double w;
}Particle;

#endif // UTILS_H
