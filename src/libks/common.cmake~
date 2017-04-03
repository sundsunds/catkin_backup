# Custom Libraries #
####################

function(ks_find_library libname header name msgType)
	find_library(${name}_LIBRARY ${libname})
	if(NOT ${name}_LIBRARY)
		message(${msgType} "Unable to locate library: ${libname}")
	endif(NOT ${name}_LIBRARY)
	find_path(${name}_INCLUDE_DIR ${header}
		HINTS /usr/local/include /usr/include /opt/local/include)
	if(NOT ${name}_INCLUDE_DIR)
		message(${msgType} "Unable to locate include file: ${header}")
	endif(NOT ${name}_INCLUDE_DIR)

	if(${name}_LIBRARY AND ${name}_INCLUDE_DIR)
		set(Found${name} 1 PARENT_SCOPE)
		include_directories(${${name}_INCLUDE_DIR})
		string(TOUPPER ${name} define)
		add_definitions("-DHAS_${define}")
                message("  ++ RAJ ++ definition added :-DHAS_${define}")
	else ()
		set(Found${name} 0)
	endif (${name}_LIBRARY AND ${name}_INCLUDE_DIR)	
endfunction(ks_find_library)

##########################
# Symbolic Link Creation #
##########################

function(ks_create_link target old new)
	add_custom_target(rm_${target} COMMAND ${CMAKE_COMMAND} -E remove ${new})
	add_custom_target(ln_${target} COMMAND ${CMAKE_COMMAND} -E create_symlink ${old} ${new})
	add_dependencies(ln_${target} rm_${target})
	add_dependencies(${target} ln_${target})
endFunction(ks_create_link)

#######################
# Common Dependencies #
#######################

function(flycapture_dependencies)
	#find_package(SDL REQUIRED)
	#find_package(Eigen REQUIRED)
	ks_find_library(flycapture flycapture/FlyCapture2.h FlyCapture WARNING)
	ks_find_library(cvd cvd/esm.h CVD ERROR)

	#include_directories(${SDL_INCLUDE_DIR})
	include_directories(${CVD_INCLUDE_DIR})
	#include_directories(${Eigen_INCLUDE_DIRS})
	
	# Export
	set(FoundFlyCapture ${FoundFlyCapture} PARENT_SCOPE)
	set(FlyCapture_LIBRARY ${FlyCapture_LIBRARY} PARENT_SCOPE)
	#set(SDL_LIBRARY ${SDL_LIBRARY} PARENT_SCOPE)
endFunction(flycapture_dependencies)
