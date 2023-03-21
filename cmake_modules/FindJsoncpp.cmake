# - Try to find LibJsoncpp
# Once done this will define
#  LIBJSONCPP_FOUND - System has LibXml2
#  LIBJSONCPP_INCLUDE_DIRS - The LibXml2 include directories
#  LIBJSONCPP_LIBRARIES - The libraries needed to use LibXml2
#  LIBJSONCPP_DEFINITIONS - Compiler switches required for using LibXml2

find_path(JSONCPP_INCLUDE_DIR json
          HINTS /usr/include/jsoncpp/ /usr/local/include/ /opt/local/include/ ${PC_JSONCPP_INCLUDEDIR} ${PC_JSONCPP_INCLUDE_DIRS}
          PATH_SUFFIXES json )

find_library(JSONCPP_LIBRARY NAMES jsoncpp libjsoncpp 
             HINTS /usr/local/lib/ /opt/local/lib/ ${PC_JSONCPP_LIBDIR} ${PC_JSONCPP_LIBRARY_DIRS} )

set(JSONCPP_LIBRARIES ${JSONCPP_LIBRARY} )
set(JSONCPP_INCLUDE_DIRS ${JSONCPP_INCLUDE_DIR} )

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set LIBXML2_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(Jsoncpp  DEFAULT_MSG
                                  JSONCPP_LIBRARY JSONCPP_INCLUDE_DIR)

mark_as_advanced(JSONCPP_INCLUDE_DIR JSONCPP_LIBRARY )

if(JSONCPP_FOUND)
	add_definitions(-DJSONCPP_FOUND)
endif()
