#include "Action.h"

#include "Utils.h"
#include <chrono>

Action::Action()
{
    linVel = 0.0;
    angVel = 0.0;

}

bool willCollide(std::vector<float> lasers, std::vector<float> sonars) {
    const float COLLISION_THRESHOLD = 1.4;
    
    // Checando os sonares
    const int VALID_SONARS[] = {3, 4};

    for (int s : VALID_SONARS) {
        if (sonars[s] <= COLLISION_THRESHOLD) {
            return true;
        }
    }

    // Checando os lasers
    const int FOV = 20;

    for (int i = 90 - FOV; i <= 90 + FOV; i++) {
        if (lasers[i] <= COLLISION_THRESHOLD) {
            return true;
        }
    }

    const float SAFETY_ENVELOPE = 0.5;

    for (int i = 90; i <= 180; i++) {
        if (lasers[i] <= SAFETY_ENVELOPE) {
            printf("Inside safety envelope!\n");
            return true;
        }
    }

        // Sem colisão
        return false;
}

void Action::avoidObstacles(std::vector<float> lasers, std::vector<float> sonars)
{
    const float ANGULAR_VELOCITY = 0.3;
    const float FORWARD_VELOCITY = 1.4;

    bool collide = willCollide(lasers, sonars);

    if (collide) {
        printf("Collide: WILL COLLIDE, spinning!\n");
        linVel = 0.0;
        angVel = ANGULAR_VELOCITY;
    } else  {
        printf("Collide: won't collide, going forward!\n");
        linVel = FORWARD_VELOCITY;
        angVel = 0.0;
    }
}

float limit(float value, float min, float max)
{
    if (value < min) {
        return min;
    } else if (value > max) {
        return max;
    } else {
        return value;
    }
}

float limit(float value, float max)
{
    return limit(value, -max, max);
}

struct LaserSpan {
    int start;
    int end;
};

float get_closest_laser(std::vector<float> lasers, LaserSpan span)
{
    float closest = lasers[span.start];
    for (int i = span.start; i <= span.end; i++) {
        if (lasers[i] < closest) {
            closest = lasers[i];
        }
    }
    return closest;
}





void Action::keepAsFarthestAsPossibleFromWalls(std::vector<float> lasers, std::vector<float> sonars, MotionCallback callback)
{
    static double last_error = 0.0;
    static double integrated_error = 0.0;
    static auto last_time = std::chrono::system_clock::now();

    const float P = 0.5;
    const float I = 0.002;
    const float D = 3.2;

    const float FORWARD_VELOCITY = 3.4;
    const float MAX_INTEGRAL = 10.0;

    auto now = std::chrono::system_clock::now();
    using Ms = std::chrono::duration<double, std::chrono::milliseconds::period>;
    double deltaTimeMs = Ms(now - last_time).count();
    if (deltaTimeMs > 150.) {
        deltaTimeMs = 150.;
    }
    last_time = now;

    const int SIDE_FOV = 45;
    LaserSpan left_span = {0, SIDE_FOV};
    LaserSpan right_span = {180 - SIDE_FOV, 180};

    // calcular a distancia das paredes
    float e_left = get_closest_laser(lasers, left_span);
    float e_right = get_closest_laser(lasers, right_span);
    float e = e_right - e_left;

    // Limita o erro
    const float MAX_ERROR = 2.5;
    e = limit(e, MAX_ERROR);

    // Derivada
    float e_prime = (e - last_error) / (deltaTimeMs / 1000);
    last_error = e;
    e_prime = limit(e_prime, 2.5);

    // Integral
    integrated_error += e * deltaTimeMs / 1000;
    // Atenua a integral
    integrated_error *= 0.95;

    // Limitar a integral
    integrated_error = limit(integrated_error, MAX_INTEGRAL); 

    // Calcula parametros pid
    float pid = - P * e - D * e_prime - I * integrated_error;

    // Limita a velocidade angular
    pid = limit(pid, 0.8);

    printf("\n\n Erro: %.2f, Derivada: %.2f, Integral: %.2f, PID: %.2f\n", e, e_prime, integrated_error, pid);
    // Printa os valores relativo ao PID
    printf("PID: %.2f, P: %.2f, I: %.2f, D: %.2f\n", pid, P * e/pid, I * integrated_error/pid, D * e_prime/pid);

    callback(e, e_prime, integrated_error, pid);

    if (willCollide(lasers, sonars)) {
        avoidObstacles(lasers, sonars);
    }
    else
    {
        printf("No imminent collision detected.\n");
        linVel = FORWARD_VELOCITY;
        angVel = pid;
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
        mc.mode=WANDER;
        mc.direction=AUTO;
    }else if(key=='3'){
        mc.mode=FARFROMWALLS;
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

