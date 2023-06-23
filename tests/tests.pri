SOURCES += \
    $$PWD/main.cpp

################################################################################

SOURCES += \
    $$PWD/aero/TestAeroAngles.cpp \
    $$PWD/aero/TestAeroBody.cpp \
    $$PWD/aero/TestFuselage.cpp \
    $$PWD/aero/TestMeanAeroChord.cpp \
    $$PWD/aero/TestRhoEffects.cpp \
    $$PWD/aero/TestSchrenkDist.cpp \
    $$PWD/aero/TestStabilizerHor.cpp \
    $$PWD/aero/TestStabilizerVer.cpp \
    $$PWD/aero/TestWingBody.cpp

################################################################################

SOURCES += \
    $$PWD/ctrl/TestHingeMoment.cpp

################################################################################

SOURCES += \
    $$PWD/env/TestAtmosphereUS76.cpp

################################################################################

SOURCES += \
    $$PWD/extra/TestWinchLauncher.cpp \
    $$PWD/extra/TestWingRunner.cpp

################################################################################

SOURCES += \
    $$PWD/gear/TestSimpleGear.cpp

################################################################################

SOURCES += \
    $$PWD/mass/TestInertiaMatrix.cpp \
    $$PWD/mass/TestPointMass.cpp

################################################################################

SOURCES += \
    $$PWD/prop/TestPistonEngine.cpp \
    $$PWD/prop/TestPropeller.cpp \
    $$PWD/prop/TestPropellerGovernor.cpp

################################################################################

SOURCES += \
    $$PWD/rotor/TestMainRotor.cpp \
    $$PWD/rotor/TestRotorUtils.cpp \
    $$PWD/rotor/TestTailRotor.cpp
