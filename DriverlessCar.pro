TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
        main.cpp \
    config.cpp \
    imagecapture.cpp \
    carcontrol.cpp \
    application.cpp \
    detectlane.cpp

LIBS += `pkg-config opencv --cflags --libs` -lpigpio -lwiringPi -lraspicam -lraspicam_cv -lpcd8544

HEADERS += \
    config.h \
    lanedetect.h \
    imagecapture.h \
    carcontrol.h \
    application.h \
    detectlane.h
