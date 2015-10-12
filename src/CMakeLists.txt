
# Check for CMake minimum version
CMAKE_MINIMUM_REQUIRED(VERSION 3.0)

# This project is designed to be built outside the source tree.
PROJECT( VISUALIZER )

if(NOT ${CMAKE_CL_64} EQUAL 1)
	message( FATAL_ERROR "You need to select a 64 bit compiler!\nDelete cache and choose a 64 bit compiler." )
endif()


# CMake options
set(CMAKE_COLOR_MAKEFILE ON)
Set(CMAKE_VERBOSE_MAKEFILE ON)

# Parallel builds and reduce warnings
if(CMAKE_BUILD_TOOL MATCHES "(msdev|devenv|nmake)")
    add_definitions(/W0)
    add_definitions(/MP)
endif()

# Find Qt
FIND_PACKAGE( Qt4 REQUIRED )
INCLUDE( ${QT_USE_FILE} )
ADD_DEFINITIONS( ${QT_DEFINITIONS} )


find_package(VTK REQUIRED)
include(${VTK_USE_FILE})
add_definitions(${VTK_DEFINITIONS})

find_package(PCL 1.7)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
message("PCL_LIBRARIES - ${PCL_INCLUDE_DIRS}")

#Find the boost libraries - system,regex,date_time required for asio
FIND_PACKAGE(Boost 1.53.0 REQUIRED COMPONENTS system filesystem regex date_time thread chrono atomic iostreams serialization)
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})

# So compiler can find generated header files
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR} ${QT_DIR})

# Specifiy link directory for qtserialport - TODO make find package for this
LINK_DIRECTORIES( ${QT_LIBRARY_DIR} )


# Header Files
SET( VISUALIZATION_CRIO_HEADERS )

# Source Files
SET( VISUALIZATION_SOURCES visualization.cpp ${VISUALIZATION_HEADERS} )

# GUI Headers
SET( GUI_HEADERS guiWindow.h ASIOUDPDevice.h DataStrUDPTip.h expm_boost.h)

# GUI Sources
SET( GUI_SOURCES guiWindow.cpp ASIOUDPDevice.cpp ${GUI_HEADERS} )

# QT wrapper for processing gui via moc
QT4_WRAP_CPP( GUI_HEADERS_MOC ${GUI_HEADERS} OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED
				-DBOOST_NO_TEMPLATE_PARTIAL_SPECIALIZATION )

# Generate header file from Qt designer file
QT4_WRAP_UI(GUI_FORMS_HEADERS guiWindow.ui)

add_definitions(${Boost_DEFINITIONS} ${QT_DEFINITIONS} ${PCL_DEFINITIONS} ${VTK_DEFINITIONS})
# Build from source files
ADD_EXECUTABLE( Visualization ${GUI_HEADERS_MOC} ${GUI_SOURCES} ${VISUALIZATION_SOURCES} ${GUI_FORMS_HEADERS} )

TARGET_LINK_LIBRARIES( Visualization ${QT_LIBRARIES} QVTK ${VTK_LIBRARIES} ${PCL_LIBRARIES})