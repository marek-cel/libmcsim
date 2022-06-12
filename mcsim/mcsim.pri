HEADERS += \
    $$PWD/defs.h \
    $$PWD/doxygen.h

SOURCES += \
    $$PWD/mcsim.cpp

################################################################################

HEADERS += \
    $$PWD/aero/Fuselage.h \
    $$PWD/aero/StabilizerHor.h \
    $$PWD/aero/StabilizerVer.h \
    $$PWD/aero/TailOff.h

SOURCES += \
    $$PWD/aero/Fuselage.cpp \
    $$PWD/aero/StabilizerHor.cpp \
    $$PWD/aero/StabilizerVer.cpp \
    $$PWD/aero/TailOff.cpp

################################################################################

HEADERS += \
    $$PWD/environment/AtmosphereUS76.h

SOURCES += \
    $$PWD/environment/AtmosphereUS76.cpp

################################################################################

HEADERS += \
    $$PWD/external/WinchLauncher.h \
    $$PWD/external/WingRunner.h

SOURCES += \
    $$PWD/external/WinchLauncher.cpp \
    $$PWD/external/WingRunner.cpp

################################################################################

HEADERS += \
    $$PWD/landing_gear/SimpleSupport.h

SOURCES += \
    $$PWD/landing_gear/SimpleSupport.cpp

################################################################################

HEADERS += \
    $$PWD/propulsion/PistonEngine.h \
    $$PWD/propulsion/Propeller.h \
    $$PWD/propulsion/PropellerGovernor.h

SOURCES += \
    $$PWD/propulsion/PistonEngine.cpp \
    $$PWD/propulsion/Propeller.cpp \
    $$PWD/propulsion/PropellerGovernor.cpp

################################################################################

HEADERS += \
    $$PWD/utils/AeroAngles.h \
    $$PWD/utils/HingeMoment.h \
    $$PWD/utils/InertiaMatrix.h \
    $$PWD/utils/MeanAeroChord.h \
    $$PWD/utils/PointMass.h \
    $$PWD/utils/PrandtlGlauert.h \
    $$PWD/utils/SchrenkDist.h

SOURCES += \
    $$PWD/utils/AeroAngles.cpp \
    $$PWD/utils/HingeMoment.cpp \
    $$PWD/utils/InertiaMatrix.cpp \
    $$PWD/utils/MeanAeroChord.cpp \
    $$PWD/utils/PointMass.cpp \
    $$PWD/utils/PrandtlGlauert.cpp \
    $$PWD/utils/SchrenkDist.cpp
