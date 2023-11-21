#ifndef GIMBALCONTROL_H
#define GIMBALCONTROL_H
#include <iostream>

const double PI      = 3.1415926535; //π
const float  GRAVITY = 9.78f;        //重力g

class GimbalControl{
public:
    GimbalControl(){}
    void   init();
    double Transform(float depth, float height);

private:
    float BulletModel(float x, float v, float angle);
    float getPitch(float x, float y, float v);

private:
    float init_v_;
    float init_k_;
};
#endif // GIMBALCONTROL_H
