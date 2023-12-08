set(TARGET_NAME aero)

################################################################################

set(HEADERS
    AeroAngles.h
    MeanAeroChord.h
    RhoEffects.h
    SchrenkDist.h
)

set(SOURCES
    AeroAngles.cpp
    MeanAeroChord.cpp
    RhoEffects.cpp
    SchrenkDist.cpp
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

