set( TARGET_NAME mcutilCommonHeaders )

################################################################################

set( HEADERS
    defs.h
    Aircraft.h
    CrashCause.h
    IAerodynamics.h
    IAircraft.h
    IBuilder.h
    IControls.h
    IEnvironment.h
    IIntersections.h
    ILandingGear.h
    IMass.h
    IPropulsion.h
    IRecorder.h
    StateIndex.h
)

################################################################################

add_library( ${TARGET_NAME} INTERFACE )

################################################################################

include( GNUInstallDirs )

set_target_properties( ${TARGET_NAME} PROPERTIES 
    PUBLIC_HEADER "${HEADERS}"
)

install( TARGETS ${TARGET_NAME}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/mcutil
)

################################################################################

add_subdirectory( geo )
add_subdirectory( math )
add_subdirectory( misc )
add_subdirectory( net )
add_subdirectory( physics )
add_subdirectory( signal )
add_subdirectory( time )
add_subdirectory( xml )
