#include "Action.h"

#include "Utils.h"

//////////////////
/// CONSTRUTOR ///
//////////////////

Action::Action()
{
    linVel = 0.0;
    angVel = 0.0;
}

///////////////////////////////
/// FUNCOES DE MOVIMENTACAO ///
///////////////////////////////

const double ANGLE_THRESHOLD = 10.0; // degrees
const double FORWARD_VELOCITY = 0.5; // m/s
const double ROTATION_VELOCITY = 0.5; // rad/s

void Action::followDirection(double angle)
{
    if (fabs(angle) > ANGLE_THRESHOLD)
    {
        if (angle > 0)
        {
            linVel = 0.0;
            angVel = ROTATION_VELOCITY;
        }
        else
        {
            linVel = 0.0;
            angVel = -ROTATION_VELOCITY;
        }
    }
    else
    {
        linVel = FORWARD_VELOCITY * ((ANGLE_THRESHOLD - fabs(angle)) / ANGLE_THRESHOLD);
        angVel = ROTATION_VELOCITY * angle / ANGLE_THRESHOLD;
    }
}

void Action::manualRobotMotion(MovingDirection direction)
{
    if(direction == FRONT){
        linVel= 0.5; angVel= 0.0;
    }else if(direction == BACK){
        linVel=-0.5; angVel= 0.0;
    }else if(direction == LEFT){
        linVel= 0.0; angVel= 0.5;
    }else if(direction == RIGHT){
        linVel= 0.0; angVel=-0.5;
    }else if(direction == STOP){
        linVel= 0.0; angVel= 0.0;
    }
}

void Action::stopRobot()
{
    linVel=0.0;
    angVel=0.0;
}

///////////////////////////
/// FUNCOES AUXILIARES  ///
///////////////////////////

void Action::correctVelocitiesIfInvalid()
{
    float b=0.38;

    float leftVel  = linVel - angVel*b/(2.0);
    float rightVel = linVel + angVel*b/(2.0);

    float VELMAX = 0.5;

    float absLeft = fabs(leftVel);
    float absRight = fabs(rightVel);

    if(absLeft>absRight){
        if(absLeft > VELMAX){
            leftVel *= VELMAX/absLeft;
            rightVel *= VELMAX/absLeft;
        }
    }else{
        if(absRight > VELMAX){
            leftVel *= VELMAX/absRight;
            rightVel *= VELMAX/absRight;
        }
    }
    
    linVel = (leftVel + rightVel)/2.0;
    angVel = (rightVel - leftVel)/b;
}

float Action::getLinearVelocity()
{
    return linVel;
}

float Action::getAngularVelocity()
{
    return angVel;
}

MotionControl Action::handlePressedKey(char key)
{
    MotionControl mc;
    mc.mode=MANUAL;
    mc.direction=STOP;

    if(key=='1'){
        mc.mode=MANUAL;
        mc.direction=STOP;
    }else if(key=='2'){
        mc.mode=EXPLORE;
        mc.direction=AUTO;
    }else if(key=='w' or key=='W'){
        mc.mode=MANUAL;
        mc.direction = FRONT;
    }else if(key=='s' or key=='S'){
        mc.mode=MANUAL;
        mc.direction = BACK;
    }else if(key=='a' or key=='A'){
        mc.mode=MANUAL;
        mc.direction = LEFT;
    }else if(key=='d' or key=='D'){
        mc.mode=MANUAL;
        mc.direction = RIGHT;
    }else if(key==' '){
        mc.mode=MANUAL;
        mc.direction = STOP;
    }
    
    return mc;
}

