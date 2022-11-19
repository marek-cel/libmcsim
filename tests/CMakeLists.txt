set( TARGET_NAME tests )

################################################################################

#find_package( LibXml2 REQUIRED )
find_package( GTest REQUIRED )
find_package( Threads REQUIRED )

################################################################################

include_directories( . )

################################################################################

set( SOURCES
    main.cpp

    aero/TestAeroAngles.cpp
    aero/TestFuselage.cpp
    aero/TestMeanAeroChord.cpp
    aero/TestRhoEffects.cpp
    aero/TestSchrenkDist.cpp
    aero/TestStabilizerHor.cpp
    aero/TestStabilizerVer.cpp
    aero/TestTailOff.cpp

    ctrl/TestHingeMoment.cpp

    env/TestAtmosphereUS76.cpp

    extra/TestWinchLauncher.cpp
    extra/TestWingRunner.cpp

    gear/TestSimpleSupport.cpp

    mass/TestInertiaMatrix.cpp
    mass/TestPointMass.cpp

    prop/TestPistonEngine.cpp
    prop/TestPropeller.cpp
    prop/TestPropellerGovernor.cpp

    rotor/TestMainRotor.cpp
    rotor/TestRotorUtils.cpp
    rotor/TestTailRotor.cpp
)

################################################################################

add_executable( ${TARGET_NAME} WIN32 ${SOURCES} )

target_link_libraries( ${TARGET_NAME}

    -Wl,--start-group

    gcov

    aero
    ctrl
    env
    extra
    gear
    mass
    prop
    rotor

    mcutils-math
    mcutils-misc

    ${LIBXML2_LIBRARIES}
    GTest::gtest
    GTest::gtest_main
    ${CMAKE_THREAD_LIBS_INIT}

    -Wl,--end-group
)

################################################################################

add_test( NAME ${TARGET_NAME} COMMAND ${TARGET_NAME} )
