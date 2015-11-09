TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt
CONFIG += c++11

SOURCES += main.cpp \
    loadg2o.cpp \
    factorgraph.cpp \
    vertex.cpp \
    edge.cpp \
    pose.cpp \
    optimizer.cpp

include(deployment.pri)
qtcAddDeployment()

HEADERS += \
    loadg2o.h \
    factorgraph.h \
    vertex.h \
    edge.h \
    pose.h \
    optimizer.h
INCLUDEPATH += /home/valerio/Eigen \
               /home/valerio/eigen-eigen-10219c95fe65

