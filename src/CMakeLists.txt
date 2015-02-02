
#declare library components
declare_PID_Component(STATIC_LIB NAME fri-driver DIRECTORY api_driver_fri)

#find_package (Threads)
declare_PID_Component_Dependency(
	COMPONENT fri-driver
	LINKS STATIC -lpthread)
	
#find_package (dl)
declare_PID_Component_Dependency(
	COMPONENT fri-driver
	LINKS STATIC -ldl)
