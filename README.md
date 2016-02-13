# DriverFramework

Driver framework for POSIX based userspace drivers.

## Overview

The top level namespace is DriverFramework.

The main classes are:

	* Framework: Used to start and stop the driver framework

	* DevMgr: Used to:
		- Register and unregister device drivers
		- get and release DevHandle objects

	* WorkMgr: Used by drivers to:
		- schedule periodic tasks
		- create and destroy WorkHandles

	* DevObj: The base class of all drivers

The framework provides two intermediate driver classes as a base for new drivers:

	* VirtDevObj: Provides a base class for simulated drivers

	* I2CDevObj: Provides a base class for I2C drivers

### Framework Initialization

The framework must be initialized before the other classes can be used. There is no C++ static
initialization used in the framework to allow environements that do not support C++ static
initialization.
```
DriverFramework::initialize();
...
DriverFramework::shutdown();
```

If the exit condition is an async event and you need to block the calling thread you can use:
```
DriverFramework::waitForShutdown();
```
This will block until DriverFramework::shutdown() is called from another thread;


### Driver Initialization

When the driver is initialized, it will register with the framework and provide an instance
the base class path.

For instance, if the driver was created with:

```
#define GYRO_BASE_PATH "/dev/gyro"

class Gyro : public I2CDevObj
{
public:
	Gyro() :
		I2CDevObj("Gyro", GYRO_BASE_PATH, 1000)
	{}
...
};

void run()
{
	DriverFramework::initialize();
	Gyro myGyro();
	myGyro.init(); // register the driver

	DevHandle h;
	DevMgr::getHandle("/dev/gyro0", h); // Starts the driver running

	if (!h.isvalid()) {
		printf("failed to get device handle\n");
	}

	...

	DevMgr::releaseHandle(h);

	DriverFramework::waitForShutdown(); // handles are freed on shutdown

	// Otherwise h releases the handle when destructed
}

```

The init() call would create a virtual /dev/gyro0 node. A second driver also regsitering with
GYRO_BASE_PATH would create a virtual /dev/gyro1 node.

A handle to the device can be acquired using:

```
myGyro.init();
...
DevMgr::getHandle("/dev/gyro0", h);
```

DevHandle objects cannot be copied.

### Starting and Stopping the Driver

By default, the driver is started the first time a handle is opened to the device,
and stopped when the last handle is released.

The driver can be explicitly started or stopped using start() or stop():

```
myGyro.start();
...
myGyro.stop();
```

### Calling POSIX functions

The read, write and ioctl calls are wrapped and provided as calls via the DevHandle.

The functions follow the POSIX sematics and the errno value can be accessed via getError().
```
DevHandle h;
DevMgr::getHandle("/dev/gyro0", h); // Starts the driver running

SomeDataStruct data[3];
int ret = h.read(data, sizeof(data));
if (ret < 0) {
	printf("Error read failed (%d)\n", h.getError());
}

```
Errors can be checked by testing for errors after a

### Testing

To run the unit tests build it and run the test app:

```
make
build_linux/test/df_testapp
```
# Porting Navio drivers

