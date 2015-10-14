FILE( GLOB executables RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp )

foreach( executable ${executables} )
	get_filename_component( executable_name ${executable} NAME_WE )
	add_definitions( "-std=c++11" )
	rosbuild_add_executable( ${executable_name} ${executable} )
#	target_link_libraries( ${executable_name} )
endforeach()
