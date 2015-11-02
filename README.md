# DriverFramework

Driver framework for POSIX based userspace drivers. 

## Overview

The top level namespace is DriverFramework.

The main classes are:

Framework - used to start and stop the driver framework

DevMgr - Used to
		1) Register and unregister device drivers
		2) get and release DevHandle objects

WorkMgr   - used by drivers to:
		1) schedule periodic tasks
		2) create and destroy WorkHandles
 
DriverObj - the base class of all drivers
