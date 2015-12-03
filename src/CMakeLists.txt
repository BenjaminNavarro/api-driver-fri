execute_process(COMMAND which xeno-config RESULT_VARIABLE res OUTPUT_VARIABLE XENO_CONFIG_PATH OUTPUT_STRIP_TRAILING_WHITESPACE)
if(res)
	set(FOUND_XENOMAI FALSE)
else()
	set(FOUND_XENOMAI TRUE)
endif()
message(STATUS "Found Xenomai ${FOUND_XENOMAI} : ${XENO_CONFIG_PATH}")

#declare library components
if(APPLE)
	declare_PID_Component(
		SHARED_LIB 
		NAME fri-driver 
		DIRECTORY api_driver_fri
		EXPORTED DEFINITIONS __MACOS__ 	
		INTERNAL COMPILER_OPTIONS m64
		RUNTIME_RESOURCES fri_configurations)

elseif(UNIX)
	if(${PROJECT_NAME}_ARCH EQUAL 32)
		declare_PID_Component(
			SHARED_LIB 
			NAME fri-driver 
			DIRECTORY api_driver_fri
			EXPORTED DEFINITIONS __LINUX__
			RUNTIME_RESOURCES fri_configurations)
	elseif(${PROJECT_NAME}_ARCH EQUAL 64)
		declare_PID_Component(
			SHARED_LIB 
			NAME fri-driver
			DIRECTORY api_driver_fri
			EXPORTED DEFINITIONS __LINUX__ 
			INTERNAL COMPILER_OPTIONS -m64
			RUNTIME_RESOURCES fri_configurations)

		if(${FOUND_XENOMAI})
			execute_process(COMMAND ${XENO_CONFIG_PATH} --skin=posix --cflags OUTPUT_VARIABLE XENO_CFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
			message("${XENO_CONFIG_PATH} --skin=posix --cflags : ${XENO_CFLAGS}")

			execute_process(COMMAND ${XENO_CONFIG_PATH} --skin=posix --ldflags OUTPUT_VARIABLE XENO_LDFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
			message("${XENO_CONFIG_PATH} --skin=posix --ldflags : ${XENO_LDFLAGS}")

			declare_PID_Component(
				SHARED_LIB
				NAME fri-driver-rt
				DIRECTORY api_driver_fri
				EXPORTED DEFINITIONS __LINUX__
				EXPORTED COMPILER_OPTIONS ${XENO_CFLAGS} -D__LINUX__
				INTERNAL COMPILER_OPTIONS -m64
				RUNTIME_RESOURCES fri_configurations)

			declare_PID_Component_Dependency(
				COMPONENT fri-driver-rt
				LINKS SHARED -lpthread -lrt -ldl)
			   
			declare_PID_Component_Dependency(
				COMPONENT fri-driver-rt
				LINKS SHARED /usr/xenomai/lib/libcobalt.so)

			set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${XENO_LDFLAGS}")
			set(CMAKE_CFLAGS "${CMAKE_CFLAGS} {XENO_CFLAGS}")
			set(CMAKE_CPPFLAGS "${CMAKE_CPPFLAGS} {XENO_CFLAGS}")
		endif()
	endif()
endif()

declare_PID_Component_Dependency(
	COMPONENT fri-driver
	LINKS SHARED -lpthread -ldl)

