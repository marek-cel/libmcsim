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
    $$PWD/env/AtmosphereUS76.h

SOURCES += \
    $$PWD/env/AtmosphereUS76.cpp

################################################################################

HEADERS += \
    $$PWD/ext/WinchLauncher.h \
    $$PWD/ext/WingRunner.h

SOURCES += \
    $$PWD/ext/WinchLauncher.cpp \
    $$PWD/ext/WingRunner.cpp

################################################################################

HEADERS += \
    $$PWD/gear/SimpleSupport.h

SOURCES += \
    $$PWD/gear/SimpleSupport.cpp

################################################################################

HEADERS += \
    $$PWD/prop/PistonEngine.h \
    $$PWD/prop/Propeller.h \
    $$PWD/prop/PropellerGovernor.h

SOURCES += \
    $$PWD/prop/PistonEngine.cpp \
    $$PWD/prop/Propeller.cpp \
    $$PWD/prop/PropellerGovernor.cpp

################################################################################

HEADERS += \
    $$PWD/rotor/MainRotor.h \
    $$PWD/rotor/TailRotor.h

SOURCES += \
    $$PWD/rotor/MainRotor.cpp \
    $$PWD/rotor/TailRotor.cpp

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
