
#declare library components
declare_PID_Component(
	SHARED_LIB 
	NAME fri-driver 
	DIRECTORY api_driver_fri
	RUNTIME_RESOURCES 471-FRI-Driver-2ms.init 471-FRI-Driver-5ms.init 472-FRI-Driver-2ms.init 472-FRI-Driver-5ms.init)

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
