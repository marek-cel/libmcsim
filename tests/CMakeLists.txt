set(TARGET_NAME tests)

################################################################################

find_package(GTest REQUIRED)

################################################################################

include_directories(.)

################################################################################

set(SOURCES
    CsvFileReader.cpp
    main.cpp

    aero/TestAeroAngles.cpp
    aero/TestMeanAeroChord.cpp
    aero/TestRhoEffects.cpp
    aero/TestSchrenkDist.cpp

    ctrl/TestHingeMoment.cpp

    env/TestAtmosphereUS76.cpp

    gear/TestPacejka.cpp
    gear/TestSimpleGear.cpp

    mass/TestInertiaMatrix.cpp
    mass/TestPointMass.cpp

    prop/TestPistonEngine.cpp
    prop/TestPropeller.cpp
    prop/TestPropellerGovernor.cpp

    rotor/MomentumTheory.cpp
    rotor/TestMainRotor.cpp
    rotor/TestRotorUtils.cpp
    rotor/TestTailRotor.cpp
)

################################################################################

add_executable(${TARGET_NAME} WIN32 ${SOURCES})

target_link_libraries(${TARGET_NAME}

    -Wl,--start-group

    gcov

    mcsim

    ${MCUTILS_LIBRARY}

    GTest::gtest
    GTest::gtest_main

    -Wl,--end-group
)

################################################################################

gtest_discover_tests(${TARGET_NAME}
    WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
)
