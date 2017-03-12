# Locate yaml-cpp
#
# This module defines
#  YAMLCPP_FOUND, if false, do not try to link to yaml-cpp
#  YAMLCPP_INCLUDE_DIR, where to find yaml.h
#
# If yaml-cpp is not installed in a standard path, you can use the YAMLCPP_DIR CMake variable
# to tell CMake where yaml-cpp is.

find_package(PkgConfig)
pkg_check_modules(YAML_CPP yaml-cpp)

# find the yaml-cpp include directory
find_path(YAML_CPP_INCLUDE_DIR
          NAMES yaml-cpp/yaml.h
          PATHS
          /usr/local/include/
          /usr/include/
          /opt/yaml-cpp/
          ${YAML_CPP_DIR}/include/)

# handle the QUIETLY and REQUIRED arguments and set YAMLCPP_FOUND to TRUE if all listed variables are TRUE
include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(YAML_CPP DEFAULT_MSG YAML_CPP_INCLUDE_DIR)
mark_as_advanced(YAML_CPP_INCLUDE_DIR)
