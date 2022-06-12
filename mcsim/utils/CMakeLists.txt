set( TARGET_NAME mcsim_utils )

################################################################################

set( HEADERS
    AeroAngles.h
    HingeMoment.h
    InertiaMatrix.h
    MeanAeroChord.h
    PointMass.h
    PrandtlGlauert.h
    SchrenkDist.h
)

set( SOURCES
    AeroAngles.cpp
    HingeMoment.cpp
    InertiaMatrix.cpp
    MeanAeroChord.cpp
    PointMass.cpp
    PrandtlGlauert.cpp
    SchrenkDist.cpp
)

################################################################################

add_library( ${TARGET_NAME} STATIC ${SOURCES} )

################################################################################

include( GNUInstallDirs )

set_target_properties( ${TARGET_NAME} PROPERTIES 
    PUBLIC_HEADER "${HEADERS}"
)

install( TARGETS ${TARGET_NAME}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/mcsim/utils
)

