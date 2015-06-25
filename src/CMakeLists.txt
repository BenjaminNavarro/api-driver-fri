
#declare library components
declare_PID_Component(SHARED_LIB NAME fri-driver DIRECTORY api_driver_fri)

declare_PID_Component_Dependency(
	COMPONENT fri-driver
	EXPORTED_DEFINITIONS __LINUX__)

#find_package (Threads)
declare_PID_Component_Dependency(
	COMPONENT fri-driver
	LINKS SHARED -lpthread)
	
#find_package (dl)
declare_PID_Component_Dependency(
	COMPONENT fri-driver
	LINKS SHARED -ldl)
