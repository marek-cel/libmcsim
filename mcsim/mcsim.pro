QT -= core gui

TEMPLATE = lib

################################################################################

DESTDIR = $$PWD/../lib
TARGET = mcsim

VERSION = 0.1.0

################################################################################

unix: target.path = /usr/local/lib
!isEmpty(target.path): INSTALLS += target

################################################################################

CONFIG += c++17

################################################################################

win32: CONFIG(release, debug|release): QMAKE_CXXFLAGS += -O2
unix:  CONFIG(release, debug|release): QMAKE_CXXFLAGS += -O2

win32: QMAKE_LFLAGS += /INCREMENTAL:NO

################################################################################

DEFINES += QT_DEPRECATED_WARNINGS

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

win32-msvc*: DEFINES += MCSIM_DLL_EXPORTS

################################################################################

INCLUDEPATH += ../

win32: INCLUDEPATH += \
    $(LIBMCUTILS_DIR)/include

unix: INCLUDEPATH += \
    /usr/local/include

################################################################################

win32: LIBS += \
    -L$(LIBMCUTILS_DIR)/lib \
    -lmcutils

################################################################################

include($$PWD/mcsim.pri)
