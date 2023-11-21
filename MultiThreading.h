#ifndef MULTITHREADING_H
#define MULTITHREADING_H
#include "Camera_MindVision/Camera_Mindvision_Running.h"

#include "Debug.h"

#include "SerialPort/SerialPort.h"

extern Mat              src;
extern MindvisionCamera camera_mindvision;
extern int              fd_serial0;

extern ArmorDetetion armordetetion;
//定义解算器
extern Survey       survey;
extern Kalmanfilter kalman;
extern Tracker      tracker;
extern double       fps;
extern float time_;
extern bool            serial_state;

// void* CameraMindvision(void*);
// void* ImageProcess(void*);

#endif  // MULTITHREADING_H
