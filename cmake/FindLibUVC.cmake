
# - Finds libUVC and all dependencies
# Once done this will define
#  LibUVC_FOUND - System has libUVC
#  LibUVC_INCLUDE_DIRS - The libUVC include directories
#  LibUVC_LIBRARIES - The libraries needed to use libUVC

find_library(LibUVC_LIBRARY
  NAMES libuvc.so libuvc.lib libuvc.dylib
  PATHS ${LibUVC_ROOT}/lib
    /usr/local/lib
)

if(NOT LibUVC_IGNORE_HEADERS)
  find_path(LibUVC_INCLUDE_DIR
    NAMES libuvc_config.h libuvc.h
    PATHS ${LibUVC_ROOT}/include
      /usr/include
      /usr/local/include
      /usr/local/Cellar
)
endif()

# brew installed libuvc include dirs on os x are hard to find and depends on the installed version.
if(LibUVC_INCLUDE_DIR STREQUAL "LibUVC_INCLUDE_DIR-NOTFOUND")
  file(GLOB LibUVC_INCLUDE_DIR /usr/local/Cellar/libuvc/*/include/libuvc/libuvc_config.h
  /usr/local/Cellar/libuvc/*/include/libuvc/libuvc.h)
endif()

if((LibUVC_LIBRARY STREQUAL "LibUVC_LIBRARY-NOTFOUND") OR (LibUVC_INCLUDE_DIR STREQUAL "LibUVC_INCLUDE_DIR-NOTFOUND"))
  set(LibUVC_ROOT "" CACHE PATH "Path to the root of a libuvc installation")
  set(LibUVC_FOUND 1)
  message(WARNING "LibUVC not found. Please try specifying LibUVC_ROOT")
else()
  set(LibUVC_FOUND 1)
  set(LibUVC_INCLUDE_DIRS ${LibUVC_INCLUDE_DIR})
  set(LibUVC_LIBRARIES ${LibUVC_LIBRARY})
endif()
