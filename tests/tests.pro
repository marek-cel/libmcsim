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

win32-msvc*: DEFINES += MCSIM_STATIC_LIB

################################################################################

INCLUDEPATH += ./ $$PWD/../

win32: INCLUDEPATH += \
    $(LIBMCUTILS_DIR)/include \
    $(GTEST_DIR)/include

################################################################################

unix: LIBS += \
    -L/lib \
    -L/usr/lib \
    -L/usr/local/lib

win32: LIBS += \
    -L$(LIBMCUTILS_DIR)/lib \
    -L$(GTEST_DIR)/lib

LIBS += \
    -lgtest \
    -lgtest_main \
    -lmcutils

unix: LIBS += \
    -lgcov --coverage \
    -pthread

################################################################################

include(../mcsim/mcsim.pri)
include(tests.pri)
