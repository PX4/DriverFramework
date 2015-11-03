# DriverFramework

Driver framework for POSIX based userspace drivers. 

## Overview

The top level namespace is DriverFramework.

The main classes are:

	* Framework: used to start and stop the driver framework

	* DevMgr: Used to
		- Register and unregister device drivers
		- get and release DevHandle objects

	* WorkMgr: used by drivers to:
		- schedule periodic tasks
		- create and destroy WorkHandles
 
    * DriverObj: the base class of all drivers
