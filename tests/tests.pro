QT -= gui

TEMPLATE = app

################################################################################

DESTDIR = $$PWD/../bin
TARGET = libmcsim_tests

################################################################################

CONFIG += console c++17 thread
CONFIG -= app_bundle qt

################################################################################

QMAKE_CXXFLAGS += -fprofile-arcs -ftest-coverage

################################################################################

DEFINES += QT_DEPRECATED_WARNINGS
DEFINES += _GTEST_ _TESTS_

win32: DEFINES += \
    NOMINMAX \
    WIN32 \
    _WINDOWS \
    _CRT_SECURE_NO_DEPRECATE \
    _SCL_SECURE_NO_WARNINGS \
    _USE_MATH_DEFINES

win32: CONFIG(release, debug|release): DEFINES += NDEBUG
win32: CONFIG(debug, debug|release):   DEFINES += _DEBUG

unix:  DEFINES += _LINUX_
win32: DEFINES += WIN32

################################################################################

INCLUDEPATH += ./ ../

win32: INCLUDEPATH += \
    $(OSG_ROOT)/include/libxml2

unix: INCLUDEPATH += \
    /usr/include/libxml2

################################################################################

win32: LIBS += \
    -L$(OSG_ROOT)/lib \
    -llibxml2

unix: LIBS += \
    -L/lib \
    -L/usr/lib \
    -lxml2

################################################################################

LIBS += \
    -lgcov --coverage \
    -lgtest \
    -lgtest_main \
    -pthread \
    -lxml2 \
    -lmcutils_geo \
    -lmcutils_math \
    -lmcutils_misc \
    -lmcutils_net \
    -lmcutils_signal \
    -lmcutils_time \
    -lmcutils_xml

################################################################################

include(tests.pri)
include(../mcsim/mcsim.pri)
