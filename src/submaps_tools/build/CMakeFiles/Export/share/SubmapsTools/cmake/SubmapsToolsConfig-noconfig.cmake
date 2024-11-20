#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "submaps_tools" for configuration ""
set_property(TARGET submaps_tools APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(submaps_tools PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libsubmaps_tools.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS submaps_tools )
list(APPEND _IMPORT_CHECK_FILES_FOR_submaps_tools "${_IMPORT_PREFIX}/lib/libsubmaps_tools.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
