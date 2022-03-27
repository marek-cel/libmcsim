HEADERS += \
    $$PWD/defs.h \
    $$PWD/doxygen.h

################################################################################

HEADERS += \
    $$PWD/aerodynamics/AeroAngles.h \
    $$PWD/aerodynamics/Fuselage.h \
    $$PWD/aerodynamics/MAC.h \
    $$PWD/aerodynamics/MainRotor.h \
    $$PWD/aerodynamics/PrandtlGlauert.h \
    $$PWD/aerodynamics/Schrenk.h \
    $$PWD/aerodynamics/SimpleRotor.h \
    $$PWD/aerodynamics/StabilizerHor.h \
    $$PWD/aerodynamics/StabilizerVer.h \
    $$PWD/aerodynamics/TailOff.h \
    $$PWD/aerodynamics/TailRotor.h

SOURCES += \
    $$PWD/aerodynamics/AeroAngles.cpp \
    $$PWD/aerodynamics/Fuselage.cpp \
    $$PWD/aerodynamics/MAC.cpp \
    $$PWD/aerodynamics/MainRotor.cpp \
    $$PWD/aerodynamics/PrandtlGlauert.cpp \
    $$PWD/aerodynamics/Schrenk.cpp \
    $$PWD/aerodynamics/SimpleRotor.cpp \
    $$PWD/aerodynamics/StabilizerHor.cpp \
    $$PWD/aerodynamics/StabilizerVer.cpp \
    $$PWD/aerodynamics/TailOff.cpp \
    $$PWD/aerodynamics/TailRotor.cpp

################################################################################

HEADERS += \
    $$PWD/controls/Channel.h \
    $$PWD/controls/HingeMoment.h

SOURCES += \
    $$PWD/controls/HingeMoment.cpp

################################################################################

HEADERS += \
    $$PWD/environment/AtmosphereUS76.h

SOURCES += \
    $$PWD/environment/AtmosphereUS76.cpp

################################################################################

HEADERS += \
    $$PWD/landing_gear/Wheel.h \
    $$PWD/landing_gear/WinchLauncher.h \
    $$PWD/landing_gear/WingRunner.h

SOURCES += \
    $$PWD/landing_gear/Wheel.cpp \
    $$PWD/landing_gear/WinchLauncher.cpp \
    $$PWD/landing_gear/WingRunner.cpp

################################################################################

HEADERS += \
    $$PWD/mass/InertiaMatrix.h \
    $$PWD/mass/VariableMass.h

SOURCES += \
    $$PWD/mass/InertiaMatrix.cpp \
    $$PWD/mass/VariableMass.cpp

################################################################################

HEADERS += \
    $$PWD/propulsion/Governor.h \
    $$PWD/propulsion/PistonEngine.h \
    $$PWD/propulsion/Propeller.h

SOURCES += \
    $$PWD/propulsion/Governor.cpp \
    $$PWD/propulsion/PistonEngine.cpp \
    $$PWD/propulsion/Propeller.cpp
