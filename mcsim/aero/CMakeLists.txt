set(TARGET_NAME aero)

################################################################################

set(HEADERS
    AeroAngles.h
    AeroBody.h
    Fuselage.h
    MeanAeroChord.h
    RhoEffects.h
    SchrenkDist.h
    StabilizerHor.h
    StabilizerVer.h
    WingBody.h
)

set(SOURCES
    AeroAngles.cpp
    AeroBody.cpp
    MeanAeroChord.cpp
    RhoEffects.cpp
    SchrenkDist.cpp
    StabilizerHor.cpp
    StabilizerVer.cpp
    WingBody.cpp
)

################################################################################

add_library(${TARGET_NAME} OBJECT ${SOURCES})

################################################################################

include(GNUInstallDirs)

set_target_properties(${TARGET_NAME} PROPERTIES 
    PUBLIC_HEADER "${HEADERS}"
)

install(TARGETS ${TARGET_NAME}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/mcsim/aero
)

