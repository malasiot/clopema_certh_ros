INCLUDE(${QT_USE_FILE})

MACRO(ADD_QT_LIBRARY _target)

	PARSE_ARGUMENTS(${_target} "SOURCES;HEADERS;FORMS;RESOURCES" "" ${ARGN} )

	QT4_WRAP_CPP(${_target}_MOC_SOURCES ${${_target}_HEADERS} )
	QT4_WRAP_UI(${_target}_UI_HEADERS ${${_target}_FORMS} )
	QT4_ADD_RESOURCES(${_target}_RC_SOURCES ${${_target}_RESOURCES} )

	ADD_DEFINITIONS(-DQT_NO_KEYWORDS)

	rosbuild_add_library(${_target} ${${_target}_SOURCES} ${${_target}_MOC_SOURCES} ${${_target}_RC_SOURCES} ${${_target}_UI_HEADERS})
	target_link_libraries(${_target} ${QT_LIBRARIES})
ENDMACRO(ADD_QT_LIBRARY)

MACRO(ADD_QT_EXECUTABLE _target )
	PARSE_ARGUMENTS(${_target} "SOURCES;HEADERS;FORMS;RESOURCES" "" ${ARGN} )

	QT4_WRAP_CPP(${_target}_MOC_SOURCES ${${_target}_HEADERS} )
	QT4_WRAP_UI(${_target}_UI_HEADERS ${${_target}_FORMS} )
	QT4_ADD_RESOURCES(${_target}_RC_SOURCES ${${_target}_RESOURCES} )

	ADD_DEFINITIONS(-DQT_NO_KEYWORDS)

	rosbuild_add_executable(${_target} ${${_target}_SOURCES} ${${_target}_MOC_SOURCES} ${${_target}_RC_SOURCES} ${${_target}_UI_HEADERS})
	target_link_libraries(${_target} ${QT_LIBRARIES})
ENDMACRO(ADD_QT_EXECUTABLE)
