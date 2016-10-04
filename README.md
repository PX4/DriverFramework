# DriverFramework

[![Build Status](https://travis-ci.org/PX4/DriverFramework.svg?branch=master)](https://travis-ci.org/PX4/DriverFramework)

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
    - periodic callback method `virtual void _measure()`

The framework consists of one worker thread (class `HRTWorkQueue`) that
periodically executes the method `virtual void DevObj::_measure()`, that is
implemented by the corresponding device driver to update its data.

The framework provides three intermediate driver classes as a base for new drivers:
* VirtDevObj: Provides a base class for simulated drivers
* I2CDevObj: Provides a base class for I2C drivers
* SPIDevObj: Provides a base class for SPI drivers

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
of the base class path.

For instance, if the driver was created with:

```
#define GYRO_BASE_PATH "/dev/gyro"

class Gyro : public I2CDevObj
{
public:
	Gyro() :
		I2CDevObj("Gyro", GYRO_BASE_PATH, 1000)
	{}
protected:
	virtual void _measure();
...
};

...
void run()
{
	DriverFramework::initialize();
	Gyro myGyro();
	myGyro.init(); // register the driver

	...

	DevHandle h;
	DevMgr::getHandle("/dev/gyro0", h); // Starts the driver

	if (!h.isvalid()) {
		printf("failed to get device handle\n");
	}

	h.read(...), h.write(...), ...
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

DevHandle objects are used to access the driver via a device path from anywhere
in the code after the driver was started. DevHandle objects cannot be copied.

### Starting and Stopping the Driver

By default, the driver is initialized and started the first time a handle is
opened to the device (if it is not running already). It keeps running when the
last handle is released.

The above use case utilizes handles to communicate with the driver, but there is
also a second use case. The use of DevHandle is optional and the driver can be
explicitly started or stopped using start() or stop(). In this case there is
typically a project-specific wrapper class that inherits from the driver and
directly handles the data (e.g. by publishing a message).

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
DevMgr::getHandle("/dev/gyro0", h); // Starts the driver

SomeDataStruct data[3];
int ret = h.read(data, sizeof(data));
if (ret < 0) {
	printf("Error read failed (%d)\n", h.getError());
}

```

### Testing

To run the unit tests build it and run the test app:


#### Testing on the PC
```
make linux
build_linux/test/df_testapp
```

#### Testing on Snapdragon Flight

Attach the Snapdragon Flight board via adb.

```
$ make qurt
$ cd build_qurt
$ make df_testapp-load df_imu_test-load df_mag_test-load df_pressure_test-load
```

Run mini-dm to see the output from the DSP
```
${HEXAGON_SDK_ROOT}/tools/mini-dm/Linux_Debug/mini-dm
```

Open an new shell and run the application on the target.

```
$ adb shell
# cd /home/linaro
# ./df_testapp
# ./df_imu_test
# ./df_pressure_test
```

The following unit test seems to be failing
```
# ./df_mag_test
```

## Hardware Support

Please refer to http://dev.px4.io for an overview of currently supported targets, as PX4 is using DriverFramework. Late 2016 this included

  * Mac OS
  * Linux (RPI / Navio 2)
  * QuRT (Snapdragon)
