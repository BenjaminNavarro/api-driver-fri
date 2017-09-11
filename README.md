
Overview
=========

Wrapper for the FRI library with some improvements:
 * New control modes:
  * `JOINT_TORQUE_CONTROL`: Joint torque control with gravity compensation on
  * `JOINT_DYNAMIC_CONTROL`: Joint torque control with gravity compensation off
 * The cycle time used by the KRC can be configured through FRI instead of being hardcoded in the KRL script.
 * Updated KRL script to make use of the first two points with an easier tool and gravity vector configuration (see `USER_GRAVITATION` and `USER_TOOL` variables at the top of the script).

The scripts to be loaded and executed on the KRC can be found in the `share` folder (`FRIGenericControl.src` and `FRIGenericControl.dat`).

The license that applies to the whole package content is **CeCILL-C**. Please look at the license.txt file at the root of this repository.

Installation and Usage
=======================

The procedures for installing the api-driver-fri package and for using its components is based on the [PID](http://pid.lirmm.net/pid-framework/index.html) build and deployment system called PID. Just follow and read the links to understand how to install, use and call its API and/or applications.


About authors
=====================

api-driver-fri has been developped by following authors:
+ Torsten Kroeger (Original author)
+ Benjamin Navarro (LIRMM)
+ Robin Passama (LIRMM)

Please contact Benjamin Navarro (navarro@lirmm.fr) - LIRMM for more information or questions.
