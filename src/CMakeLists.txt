set(CMAKE_INCLUDE_CURRENT_DIR ON)

include_directories(.)

# ##############################################################################
# Find required libraries
# ##############################################################################

# Find Boost, at least ver. 1.41
find_package(Boost 1.41.0 REQUIRED COMPONENTS system filesystem date_time)
include_directories(SYSTEM ${Boost_INCLUDE_DIR})
#set(Boost_LIBRARIES "${Boost_LIBRARIES} /home/akarbarc/MGR2/libs/boost_1_62_0/inst/lib")

# Set variable with list of all libraries common for this DCL
set(DCL_LIBRARIES "${DisCODe_LIBRARIES} ${Boost_LIBRARIES}")

# Find another necessary libraries
find_package(PCL 1.7.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
# # OpenCV library
FIND_PACKAGE( OpenCV REQUIRED )
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})

SET(DCL_LIBRARIES ${DCL_LIBRARIES} ${OpenCV_LIBS})

# Find another DCLs our one depends on
# Use macro: DISCODE_FIND_DCL(<DCL_NAME>)
discode_find_dcl(PCLCoreTypes)
discode_find_dcl(CvCoreTypes)
discode_find_dcl(CvBasic) # Required for Types/Features.hpp -> TODO!

# Check, whether all necessary libraries are linked
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-z,defs") 

# ##############################################################################
# Add source directories
# ##############################################################################

# CvBlobs components
add_subdirectory(Components)

# CvBlobs types
add_subdirectory(Types)


# Prepare config file to use from another DCLs
CONFIGURE_FILE(ROSIntegrationConfig.cmake.in ${CMAKE_INSTALL_PREFIX}/ROSIntegrationConfig.cmake @ONLY)
