oadrive - The Open Autonomous Driving Package
===========================================================

Made by FZI IDS/TKS. http://www.fzi.de

Please consider the license information provided with this repository.
You should have received a copy (LICENSE.txt).


Core funtionality:
-----------------------------------------------------------
The package is used to control electrical model cars in the
AADC (Audi Autonomous Driving Cup). It attempts to abide to
the rules defined for the Cup.

Dependencies:
------------------------------------------------------------
- OpenCV:
    - At least Version 3.0 recommended. 2.4.11 is supported,
      but some features will be deactivated (mainly the semi-
      automatic camera calibration).
- Aruco
- redisclient (https://github.com/nekipelov/redisclient)
- icMaker

Installation:
-------------------------------------------------------------
Build and install OpenCV, Aruco, redisclient.


Usage:
------------------------------------------------------------
The package can be built independently of the ADTF development
environment (which is required by the AADC). The 
We defined an Interface class (oadrive_interface/Interface.h).
All communication with oadrive must be passed through this
interface.
For example usage, see src/test/test_oadrive_interface.cpp.

This allows the package to be used from within other contextes
as well, like an offline simulation.
If Interface::startDebugDumping() is called at startup, the
Interface will write out debug data which can then be used for
offline configuration.

Calibration:
-------------------------------------------------------------
Proper usage of the package requires calibration of the camera.
This camera calibration calculates a Bird-View perspective
transformation using a Matrix-Of-Circles pattern.
Use test_oadrive_birdViewCal.cpp to do the calibration.
Beforehand, the intrinsic camera calibration must be saved to
a file (which is then passed to the test_oadrive_birdViewCal).

