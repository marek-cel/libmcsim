set(TARGET_NAME mass)

################################################################################

set(HEADERS
    InertiaMatrix.h
    PointMass.h
)

set(SOURCES
    InertiaMatrix.cpp
    PointMass.cpp
)

################################################################################

add_library(${TARGET_NAME} OBJECT ${SOURCES})

################################################################################

include(GNUInstallDirs)

set_target_properties(${TARGET_NAME} PROPERTIES 
    PUBLIC_HEADER "${HEADERS}"
)

install(TARGETS ${TARGET_NAME}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/mcsim/mass
)

