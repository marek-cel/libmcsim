set( TARGET_NAME aero )

################################################################################

set( HEADERS
    AeroAngles.h
    Fuselage.h
    MAC.h
    MainRotor.h
    PrandtlGlauert.h
    Schrenk.h
    SimpleRotor.h
    StabilizerHor.h
    StabilizerVer.h
    TailOff.cpp
    TailOff.h
    TailRotor.h
)

set( SOURCES
    AeroAngles.cpp
    Fuselage.cpp
    MAC.cpp
    MainRotor.cpp
    PrandtlGlauert.cpp
    Schrenk.cpp
    SimpleRotor.cpp
    StabilizerHor.cpp
    StabilizerVer.cpp
    TailOff.cpp
    TailRotor.cpp
)

################################################################################

add_library( ${TARGET_NAME} STATIC ${SOURCES} )

################################################################################

include( GNUInstallDirs )

set_target_properties( ${TARGET_NAME} PROPERTIES 
    PUBLIC_HEADER "${HEADERS}"
)

install( TARGETS ${TARGET_NAME}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/mcSim/aero
)

