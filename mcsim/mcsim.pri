HEADERS += \
    $$PWD/defs.h \
    $$PWD/doxygen.h \
    $$PWD/Aircraft.h \
    $$PWD/CrashCause.h \
    $$PWD/IAerodynamics.h \
    $$PWD/IAircraft.h \
    $$PWD/IBuilder.h \
    $$PWD/IControls.h \
    $$PWD/IEnvironment.h \
    $$PWD/IIntersections.h \
    $$PWD/ILandingGear.h \
    $$PWD/IMass.h \
    $$PWD/IPropulsion.h \
    $$PWD/IRecorder.h \
    $$PWD/StateIndex.h

SOURCES += \
    $$PWD/Aircraft.cpp

################################################################################

HEADERS += \
    $$PWD/aerodynamics/AeroAngles.h \
    $$PWD/aerodynamics/Fuselage.h \
    $$PWD/aerodynamics/MAC.h \
    $$PWD/aerodynamics/MainRotor.h \
    $$PWD/aerodynamics/MainRotorBE.h \
    $$PWD/aerodynamics/PrandtlGlauert.h \
    $$PWD/aerodynamics/RotorBlade.h \
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
    $$PWD/aerodynamics/MainRotorBE.cpp \
    $$PWD/aerodynamics/PrandtlGlauert.cpp \
    $$PWD/aerodynamics/RotorBlade.cpp \
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
    $$PWD/environment/AtmosphereMars.h \
    $$PWD/environment/AtmosphereUS76.h \
    $$PWD/environment/WindShearFAA.h

SOURCES += \
    $$PWD/environment/AtmosphereMars.cpp \
    $$PWD/environment/AtmosphereUS76.cpp \
    $$PWD/environment/WindShearFAA.cpp

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
    $$PWD/propulsion/Battery.h \
    $$PWD/propulsion/ElectricMotor.h \
    $$PWD/propulsion/FuelTank.h \
    $$PWD/propulsion/Governor.h \
    $$PWD/propulsion/PistonEngine.h \
    $$PWD/propulsion/Propeller.h \
    $$PWD/propulsion/Turbofan.h

SOURCES += \
    $$PWD/propulsion/Battery.cpp \
    $$PWD/propulsion/ElectricMotor.cpp \
    $$PWD/propulsion/FuelTank.cpp \
    $$PWD/propulsion/Governor.cpp \
    $$PWD/propulsion/PistonEngine.cpp \
    $$PWD/propulsion/Propeller.cpp \
    $$PWD/propulsion/Turbofan.cpp
