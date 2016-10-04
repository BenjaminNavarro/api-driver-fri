CMAKE_MINIMUM_REQUIRED(VERSION 3.0.2)
set(WORKSPACE_DIR ${CMAKE_SOURCE_DIR}/../.. CACHE PATH "root of the packages workspace directory")
list(APPEND CMAKE_MODULE_PATH ${WORKSPACE_DIR}/share/cmake/system) # using generic scripts/modules of the workspace
include(Package_Definition NO_POLICY_SCOPE)

PROJECT(api-driver-fri)

declare_PID_Package(
			AUTHOR 		Benjamin Navarro
			INSTITUTION	LIRMM
			MAIL		navarro@lirmm.fr
			ADDRESS 	git@gite.lirmm.fr:kuka-lwr/api-driver-fri.git
			YEAR 		2015
			LICENSE 	CeCILL-C
			DESCRIPTION "Wrapper for the FRI library"
		)

set_PID_Package_Version(1 1 0)

check_PID_Platform(NAME xeno64 OS linux ARCH 64 CONFIGURATION xenomai posix)
check_PID_Platform(NAME xeno32 OS linux ARCH 32 CONFIGURATION xenomai posix)
check_All_PID_Default_Platforms(CONFIGURATION posix)

add_PID_Package_Author(AUTHOR Robin Passama INSTITUTION LIRMM)
add_PID_Package_Category(arm/kuka-lwr)


build_PID_Package()
