QT -= core gui

TEMPLATE = app

################################################################################

DESTDIR = $$PWD/../bin
TARGET = tests

################################################################################

CONFIG += console c++17
CONFIG -= app_bundle qt

################################################################################

unix: QMAKE_CXXFLAGS += -O0 \
    --coverage \
    -fno-default-inline \
    -fno-inline \
    -fno-inline-small-functions \
    -fprofile-arcs \
    -ftest-coverage \
    -pedantic

################################################################################

DEFINES += QT_DEPRECATED_WARNINGS
DEFINES += _GTEST_ _TESTS_

win32-msvc*: DEFINES += \
    NOMINMAX \
    _CRT_SECURE_NO_DEPRECATE \
    _SCL_SECURE_NO_WARNINGS \
    _USE_MATH_DEFINES

win32-msvc*: CONFIG(release, debug|release): DEFINES += NDEBUG
win32-msvc*: CONFIG(debug, debug|release):   DEFINES += _DEBUG

unix:  DEFINES += _LINUX_
win32: DEFINES += WIN32 _WINDOWS

################################################################################

INCLUDEPATH += ./ $$PWD/../

unix: INCLUDEPATH += \
    /usr/include/libxml2

win32: INCLUDEPATH += \
    $(GTEST_DIR)/include \
    $(LIBXML_DIR)/include

################################################################################

unix: LIBS += \
    -L/lib \
    -L/usr/lib \
    -L/usr/local/lib

win32: LIBS += \
    -L$(GTEST_DIR)/lib \
    -L$(LIBXML_DIR)/lib

LIBS += \
    -lgtest \
    -lgtest_main \
    -lmcutils-ctrl \
    -lmcutils-geo \
    -lmcutils-math \
    -lmcutils-misc \
    -lmcutils-net \
    -lmcutils-time \
    -lmcutils-xml

unix: LIBS += \
    -lgcov --coverage \
    -lxml2 \
    -pthread

win32: LIBS += \
    -lws2_32  \
    -llibxml2

################################################################################

include(../mcsim/mcsim.pri)
include(tests.pri)
