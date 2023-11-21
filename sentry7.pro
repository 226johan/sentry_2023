TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

INCLUDEPATH += /usr/local/include/eigen3/

#Other
INCLUDEPATH +=/usr/local/include/
INCLUDEPATH +=/usr/local/include/opencv4/
INCLUDEPATH +=/usr/local/include/opencv4/opencv2/
LIBS += /usr/local/lib/*.so

LIBS += -lpthread
LIBS += -lgxiapi \


LIBS += -L$$/lib -lMVSDK

SOURCES += \
        AngleSolver/AngleSolver.cpp \
        AngleSolver/GimbalControl.cpp \
        ArmorDetection/ArmorDetection.cpp \
        ArmorDetection/Armorparam.cpp \
        Camera_MindVision/Camera_Mindvision_Running.cpp \
        KalmanFilter/KalmanFilter.cpp \
        MultiThreading.cpp \
        SerialPort/SerialPort.cpp \
        Survey/Survey.cpp \
        Tracker/Tracker.cpp \
        main.cpp

HEADERS += \
    AngleSolver/AngleSolver.h \
    AngleSolver/GimbalControl.h \
    ArmorDetection/ArmorDetection.h \
    ArmorDetection/Armorparam.h \
    Camera_MindVision/CameraApi.h \
    Camera_MindVision/CameraDefine.h \
    Camera_MindVision/CameraStatus.h \
    Camera_MindVision/Camera_Mindvision_Running.h \
    Debug.h \
    KalmanFilter/KalmanFilter.h \
    MultiThreading.h \
    SerialPort/SerialPort.h \
    Survey/Survey.h \
    Tracker/Tracker.h


