HEADERS += \
    $$PWD/defs.h \
    $$PWD/doxygen.h

SOURCES += \
    $$PWD/mcsim.cpp

################################################################################

HEADERS += \
    $$PWD/aero/AeroAngles.h \
    $$PWD/aero/AeroBody.h \
    $$PWD/aero/Fuselage.h \
    $$PWD/aero/MeanAeroChord.h \
    $$PWD/aero/RhoEffects.h \
    $$PWD/aero/SchrenkDist.h \
    $$PWD/aero/StabilizerHor.h \
    $$PWD/aero/StabilizerVer.h \
    $$PWD/aero/WingBody.h

SOURCES += \
    $$PWD/aero/AeroAngles.cpp \
    $$PWD/aero/AeroBody.cpp \
    $$PWD/aero/MeanAeroChord.cpp \
    $$PWD/aero/RhoEffects.cpp \
    $$PWD/aero/SchrenkDist.cpp \
    $$PWD/aero/StabilizerHor.cpp \
    $$PWD/aero/StabilizerVer.cpp \
    $$PWD/aero/WingBody.cpp

################################################################################

HEADERS += \
    $$PWD/ctrl/HingeMoment.h

SOURCES += \
    $$PWD/ctrl/HingeMoment.cpp

################################################################################

HEADERS += \
    $$PWD/env/AtmosphereUS76.h

SOURCES += \
    $$PWD/env/AtmosphereUS76.cpp

################################################################################

HEADERS += \
    $$PWD/extra/WinchLauncher.h \
    $$PWD/extra/WingRunner.h

SOURCES += \
    $$PWD/extra/WinchLauncher.cpp \
    $$PWD/extra/WingRunner.cpp

################################################################################

HEADERS += \
    $$PWD/gear/BrakeGroup.h \
    $$PWD/gear/SimpleSupport.h

SOURCES += \
    $$PWD/gear/SimpleSupport.cpp

################################################################################

HEADERS += \
    $$PWD/mass/InertiaMatrix.h \
    $$PWD/mass/PointMass.h

SOURCES += \
    $$PWD/mass/InertiaMatrix.cpp \
    $$PWD/mass/PointMass.cpp

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
    $$PWD/rotor/IInGroundEffect.h \
    $$PWD/rotor/IVortexRingState.h \
    $$PWD/rotor/MainRotor.h \
    $$PWD/rotor/RotorUtils.h \
    $$PWD/rotor/TailRotor.h

SOURCES += \
    $$PWD/rotor/MainRotor.cpp \
    $$PWD/rotor/RotorUtils.cpp \
    $$PWD/rotor/TailRotor.cpp
