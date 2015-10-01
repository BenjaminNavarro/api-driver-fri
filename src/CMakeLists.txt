
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
	endif()
endif()

#find_package (Threads)
declare_PID_Component_Dependency(
	COMPONENT fri-driver
	LINKS SHARED -lpthread)
	
#find_package (dl)
declare_PID_Component_Dependency(
	COMPONENT fri-driver
	LINKS SHARED -ldl)
