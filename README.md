# DriverFramework

Driver framework for POSIX based userspace drivers. 

## Overview

The top level namespace is DriverFramework.

The main classes are:

Framework - used to start and stop the driver framework

DriverMgr - Used to
		1) Register and unregister drivers
		2) get and release DriverHandle objects

WorkMgr   - used by drivers to:
		1) schedule periodic tasks
		2) create and destroy WorkHandles
 
DriverObj - the base class of all drivers
