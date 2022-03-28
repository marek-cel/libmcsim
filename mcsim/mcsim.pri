HEADERS += \
    $$PWD/defs.h \
    $$PWD/doxygen.h

SOURCES += \
    $$PWD/mcsim.cpp

################################################################################

HEADERS += \
    $$PWD/aero/AeroAngles.h \
    $$PWD/aero/Fuselage.h \
    $$PWD/aero/MAC.h \
    $$PWD/aero/MainRotor.h \
    $$PWD/aero/MainRotorBE.h \
    $$PWD/aero/PrandtlGlauert.h \
    $$PWD/aero/RotorBlade.h \
    $$PWD/aero/Schrenk.h \
    $$PWD/aero/StabilizerHor.h \
    $$PWD/aero/StabilizerVer.h \
    $$PWD/aero/TailOff.h \
    $$PWD/aero/TailRotor.h

SOURCES += \
    $$PWD/aero/AeroAngles.cpp \
    $$PWD/aero/Fuselage.cpp \
    $$PWD/aero/MAC.cpp \
    $$PWD/aero/MainRotor.cpp \
    $$PWD/aero/MainRotorBE.cpp \
    $$PWD/aero/PrandtlGlauert.cpp \
    $$PWD/aero/RotorBlade.cpp \
    $$PWD/aero/Schrenk.cpp \
    $$PWD/aero/StabilizerHor.cpp \
    $$PWD/aero/StabilizerVer.cpp \
    $$PWD/aero/TailOff.cpp \
    $$PWD/aero/TailRotor.cpp

################################################################################

HEADERS += \
    $$PWD/ctrl/Channel.h \
    $$PWD/ctrl/HingeMoment.h

SOURCES += \
    $$PWD/ctrl/HingeMoment.cpp

################################################################################

HEADERS += \
    $$PWD/envr/AtmosphereUS76.h

SOURCES += \
    $$PWD/envr/AtmosphereUS76.cpp

################################################################################

HEADERS += \
    $$PWD/gear/Wheel.h \
    $$PWD/gear/WinchLauncher.h \
    $$PWD/gear/WingRunner.h

SOURCES += \
    $$PWD/gear/Wheel.cpp \
    $$PWD/gear/WinchLauncher.cpp \
    $$PWD/gear/WingRunner.cpp

################################################################################

HEADERS += \
    $$PWD/mass/InertiaMatrix.h \
    $$PWD/mass/VariableMass.h

SOURCES += \
    $$PWD/mass/InertiaMatrix.cpp \
    $$PWD/mass/VariableMass.cpp

################################################################################

HEADERS += \
    $$PWD/prop/Governor.h \
    $$PWD/prop/PistonEngine.h \
    $$PWD/prop/Propeller.h

SOURCES += \
    $$PWD/prop/Governor.cpp \
    $$PWD/prop/PistonEngine.cpp \
    $$PWD/prop/Propeller.cpp
