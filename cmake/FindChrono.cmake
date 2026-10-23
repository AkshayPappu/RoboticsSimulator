# FindChrono.cmake
# -----------------
# Locate Project Chrono installation.
#
# This module defines:
#   Chrono_FOUND          - True if Chrono was found
#   CHRONO_INCLUDE_DIRS   - Include directories for Chrono
#   CHRONO_LIBRARIES      - Libraries to link against
#   CHRONO_DATA_DIR       - Path to Chrono data directory
#   CHRONO_DLL_DIR        - Path to Chrono DLLs (Windows)
#   CHRONO_DLL_NAMES      - List of DLL names to copy (Windows)
#
# The user can set Chrono_DIR to guide the search:
#   cmake -DChrono_DIR=/path/to/chrono/install/lib/cmake/Chrono ..
#
# Alternatively, set CHRONO_ROOT to the Chrono install prefix:
#   cmake -DCHRONO_ROOT=/path/to/chrono/install ..

# Try the config-mode find first (preferred)
find_package(Chrono QUIET CONFIG
    HINTS
        ${Chrono_DIR}
        ${CHRONO_ROOT}/lib/cmake/Chrono
        $ENV{CHRONO_ROOT}/lib/cmake/Chrono
        $ENV{CHRONO_DIR}
)

if(Chrono_FOUND)
    return()
endif()

# Fallback: search common install locations
find_path(CHRONO_INCLUDE_DIR
    NAMES chrono/ChConfig.h
    HINTS
        ${CHRONO_ROOT}/include
        $ENV{CHRONO_ROOT}/include
        /usr/local/include
        /usr/include
    PATH_SUFFIXES chrono
)

find_library(CHRONO_LIBRARY
    NAMES ChronoEngine
    HINTS
        ${CHRONO_ROOT}/lib
        $ENV{CHRONO_ROOT}/lib
        /usr/local/lib
        /usr/lib
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Chrono
    REQUIRED_VARS CHRONO_LIBRARY CHRONO_INCLUDE_DIR
)

if(Chrono_FOUND)
    set(CHRONO_INCLUDE_DIRS ${CHRONO_INCLUDE_DIR})
    set(CHRONO_LIBRARIES ${CHRONO_LIBRARY})
endif()
