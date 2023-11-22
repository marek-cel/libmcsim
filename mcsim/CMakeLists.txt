set(TARGET_NAME mcsim)

################################################################################

if(WIN32)
    if(BUILD_SHARED_LIBS)
        add_definitions(-DMCSIM_DLL_EXPORTS)
    endif()
endif()

################################################################################

set(HEADERS
    defs.h
)

set(SOURCES
    mcsim.cpp
)

################################################################################

add_subdirectory(aero)
add_subdirectory(ctrl)
add_subdirectory(env)
add_subdirectory(extra)
add_subdirectory(gear)
add_subdirectory(mass)
add_subdirectory(prop)
add_subdirectory(rotor)

################################################################################

add_library(${TARGET_NAME} ${SOURCES}
    $<TARGET_OBJECTS:aero>
    $<TARGET_OBJECTS:ctrl>
    $<TARGET_OBJECTS:env>
    $<TARGET_OBJECTS:extra>
    $<TARGET_OBJECTS:gear>
    $<TARGET_OBJECTS:mass>
    $<TARGET_OBJECTS:prop>
    $<TARGET_OBJECTS:rotor>
)

################################################################################

include(GNUInstallDirs)

set_target_properties(${TARGET_NAME} PROPERTIES
    VERSION ${MCSIM_VERSION_STRING}
    SOVERSION ${MCSIM_VERSION_MAJOR}
    PUBLIC_HEADER "${HEADERS}"
)

install(TARGETS ${TARGET_NAME}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/mcsim
)

################################################################################

if(WIN32)
    set(LIBS ${MCUTILS_LIBRARY})
endif()

################################################################################

target_link_libraries(${TARGET_NAME}
    ${LIBS}
)
