#ifndef DEBUG_H
#define DEBUG_H

#include "fstream"
#include "utility"
#include "vector"
#include <cstdlib>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <pthread.h>
#include <semaphore.h>
#include <thread>
#include <unistd.h>

#include "AngleSolver/AngleSolver.h"
#include "ArmorDetection/ArmorDetection.h"
#include "KalmanFilter/KalmanFilter.h"
#include "Survey/Survey.h"
#include "Tracker/Tracker.h"
#include <sys/time.h>

using namespace cv;
using namespace std;

#endif  // DEBUG_H
