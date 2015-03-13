CMAKE_MINIMUM_REQUIRED(VERSION 2.8.12)
set(WORKSPACE_DIR ${CMAKE_SOURCE_DIR}/../.. CACHE PATH "root of the packages workspace directory")
list(APPEND CMAKE_MODULE_PATH ${WORKSPACE_DIR}/share/cmake/system) # using generic scripts/modules of the workspace
include(Package_Definition NO_POLICY_SCOPE)

PROJECT(api-driver-fri)

declare_PID_Package(	AUTHOR 		Benjamin Navarro
			INSTITUTION	LIRMM
			MAIL		navarro@lirmm.fr
			ADDRESS 	git@rob-dev.lirmm.fr:kuka-lwr/api-driver-fri.git
			YEAR 		2015
			LICENSE 	CeCILL-C
			DESCRIPTION "Wrapper for the FRI library"
		)

set_PID_Package_Version(0 1)

add_PID_Package_Category(driver/fri)

IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -D__MACOS__ m64")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D__MACOS__ -m64")
ELSE()
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -D__LINUX__ -m64")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D__LINUX__ -m64")
ENDIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")


build_PID_Package()

