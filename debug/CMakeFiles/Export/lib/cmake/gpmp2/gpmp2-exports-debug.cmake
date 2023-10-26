#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "gpmp2" for configuration "Debug"
set_property(TARGET gpmp2 APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(gpmp2 PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libgpmp2Debug.so.0.3.0"
  IMPORTED_SONAME_DEBUG "libgpmp2Debug.so.0"
  )

list(APPEND _IMPORT_CHECK_TARGETS gpmp2 )
list(APPEND _IMPORT_CHECK_FILES_FOR_gpmp2 "${_IMPORT_PREFIX}/lib/libgpmp2Debug.so.0.3.0" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
