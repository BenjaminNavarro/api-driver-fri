

# standard fri-driver
if(macosx64)
	declare_PID_Component(
		SHARED_LIB
		NAME fri-driver
		DIRECTORY api_driver_fri
		EXPORTED DEFINITIONS __MACOS__
		INTERNAL COMPILER_OPTIONS m64 -std=c++11)

elseif(xeno32 OR linux32)
	declare_PID_Component(
		SHARED_LIB
		NAME fri-driver
		DIRECTORY api_driver_fri
		EXPORTED DEFINITIONS __LINUX__
		INTERNAL COMPILER_OPTIONS -std=c++11)

elseif(linux64 OR xeno64)
	declare_PID_Component(
		SHARED_LIB
		NAME fri-driver
		DIRECTORY api_driver_fri
		EXPORTED DEFINITIONS __LINUX__
		INTERNAL COMPILER_OPTIONS -m64 -std=c++11)
endif()

declare_PID_Component_Dependency(COMPONENT fri-driver LINKS SHARED ${posix_LINK_OPTIONS})


## RT driver
if(xeno32) #RT driver can be compiled
	declare_PID_Component(
		SHARED_LIB
		NAME fri-driver-rt
		DIRECTORY api_driver_fri
		EXPORTED 	DEFINITIONS __LINUX__
		INTERNAL 	COMPILER_OPTIONS ${xenomai_COMPILE_OPTIONS} -std=c++11
					LINKS ${xenomai_LINK_OPTIONS})

	declare_PID_Component_Dependency(COMPONENT fri-driver-rt LINKS SHARED ${posix_LINK_OPTIONS})
elseif(xeno64)
	declare_PID_Component(
		SHARED_LIB
		NAME fri-driver-rt
		DIRECTORY api_driver_fri
		EXPORTED 	DEFINITIONS __LINUX__
		INTERNAL 	COMPILER_OPTIONS -m64 ${xenomai_COMPILE_OPTIONS} -std=c++11
					LINKS ${xenomai_LINK_OPTIONS})

	declare_PID_Component_Dependency(COMPONENT fri-driver-rt LINKS SHARED ${posix_LINK_OPTIONS})
endif()
