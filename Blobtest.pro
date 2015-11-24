TEMPLATE = app
CONFIG += console
QT += core
CONFIG -= app_bundle
#CONFIG -= qt
CONFIG += c++14
LIBS += `pkg-config opencv --libs`
SOURCES += main.cpp \
    backup.cpp \
    ds.cpp \
    z.cpp \
    datamining.cpp \
    utility.cpp \
    region.cpp \
    test.cpp \
    writingsegmentation.cpp

include(deployment.pri)
qtcAddDeployment()

HEADERS += \
    datamining.h \
    warputility.h \
    utility.h \
    basicimagealgo.h \
    buglocation.h \
    buglocation.h

