
get_PID_Platform_Info(OS system)

# standard fri-driver
if(system STREQUAL macosx)
	declare_PID_Component(
		SHARED_LIB
		NAME fri-driver
		DIRECTORY api_driver_fri
		EXPORTED DEFINITIONS __MACOS__
		INTERNAL COMPILER_OPTIONS m64 -std=c++11)

else()
	declare_PID_Component(
		SHARED_LIB
		NAME fri-driver
		DIRECTORY api_driver_fri
		EXPORTED DEFINITIONS __LINUX__
		INTERNAL COMPILER_OPTIONS -std=c++11)
endif()

declare_PID_Component_Dependency(COMPONENT fri-driver LINKS SHARED ${posix_LINK_OPTIONS})
