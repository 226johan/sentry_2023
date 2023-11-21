#ifndef ARMORPARAM_H
#define ARMORPARAM_H

class ImageProcessParam
{
public:
    ImageProcessParam();
    int red_gary_thre_value;
    int blue_gary_thre_value;  // 图像灰度化后阈值化的阈值
    int red_minus_thre_value;
    int blue_minus_thre_value;  // 图像通道相减后阈值化的阈值
};

class ArmorParam
{
public:
    void AdjustParam();
    ImageProcessParam     image_param;
};


#endif // ARMORPARAM_H
