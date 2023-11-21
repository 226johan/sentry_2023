#include "AngleSolver/GimbalControl.h"

#include <cmath>
#include <stdio.h>
/**
 * @brief 初始化射速、摩擦系数
 */
void GimbalControl::init()
{
    init_v_ = 30;  //28 30
    init_k_ = 0.020f; //0.026
}

/**
 * @brief 依据我们获取的z也就是深度和速度来计算子弹落地实际需要的Y
 * @param <depth>  敌人距离我们水平深度(z坐标)
 * @param <speed>  子弹射速
 * @param <patch>  敌人的pitch
 * @return height  按照深度计算子弹成功击打目标时候需要的高度来满足抛物线
 */
float GimbalControl::BulletModel(float depth, float speed, float pitch)
{
    float t, height;
    t = ( float )((exp(init_k_ * depth) - 1) / (init_k_ * speed * cos(pitch)));
    height = ( float )(speed * sin(pitch) * t - GRAVITY * t * t / 2);
    return height;
}

/**
 * @brief  获取Pitch
 * @param <depth>  敌人距离我们水平深度(z坐标)
 * @param <speed>  子弹射速
 * @param <height> 敌人的高度
 * @return 获取的pitch
 */
float GimbalControl::getPitch(float depth, float height, float speed)
{
    float y_temp, y_actual, dy;
    float pitch = 0;
    y_temp      = height;  ///获取高度
    for (int i = 0; i < 20; i++)
    {
        pitch    = (float)atan2(y_temp, depth);       ///计算敌方装甲和我们的pitch
        y_actual = BulletModel(depth, speed, pitch);  ///计算此时深度对应的子弹下落高度、理论上是小于实际高度的
        dy       = height - y_actual;                 ///获取差值
        y_temp   = y_temp + dy;

        if (fabsf(dy) < 0.001f)
        {
            break;
        }
    }
    return pitch;
}

/**
 * @brief  实现弹道预测
 * @param  <depth>  计算装甲的深度,不能使用距离
 * @param  <height> 计算装甲和我们的高度
 * @return 弹道预测之后的pitch
 */
double GimbalControl::Transform(float depth, float height)
{
    double pitch = -( float )getPitch(depth / 1000, -height / 1000, init_v_);
    return pitch;
}
