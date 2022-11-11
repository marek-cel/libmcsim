set( TARGET_NAME mcsim-rotor )

################################################################################

set( HEADERS
    IInGroundEffect.h
    IVortexRingState.h
    MainRotor.h
    RotorUtils.h
    TailRotor.h
)

set( SOURCES
    MainRotor.cpp
    RotorUtils.cpp
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
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/mcsim/rotor
)

