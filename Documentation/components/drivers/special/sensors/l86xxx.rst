=======
L86-XXX
=======

This driver provides support for the L86-XXX family of GNSS modules by
Quectel via the :doc:`uorb </components/drivers/special/sensors/sensors_uorb>` interface. 
Functionality for this driver was tested using the Quectel L86-M33.

.. warning::
   This driver only contains preliminary support for a handful of proprietary
   'PMTK' commands There is no support for the entire suite of commands yet
   CONSIDER THIS DRIVER EXPERIMENTAL.

Application Programming Interface
=================================

To register the device for use, you will need to enable the standard upper half
serial drivers (``CONFIG_STANDARD_SERIAL``), since the L86-XXX driver requires
the path to the UART interface the module is connected to. You will also need to 
ensure that the baud rate of the UART interface is set to 9600, which is the default 
baud rate of the L86-XXX series of GNSS modules. 

The driver supports changing the default baud rate and update rate of the GNSS module.
As a result, you will also need to enable serial TERMIOS support (``CONFIG_SERIAL_TERMIOS``).
The baud rate and update rate of the GNSS module can be configured using the ``L86_XXX_BAUD`` and ``L86_XXX_FIX_INT`` options respectively.
Note that a faster update rate will require a higher baud rate to support it and the supported baud rates for the L86-XXX series of GNSS modules are: 4800, 9600, 14400, 19200, 38400, 57600 and 115200
The baud rate and update rates of the module are changed at registration time.

.. code-block:: c

   #if defined(CONFIG_SENSORS_L86_XXX)
      #include <nuttx/sensors/l86xxx.h>
      
      /* Register L86-M33 on USART3 */

      ret = l86xxx_register("/dev/l86m33", "/dev/ttyS2", 0);
      if (ret < 0) {
         syslog(LOG_ERR, "Failed to register L86-M33: %d\n", ret);
      }
   #endif

One the driver is registered, it starts a thread that continuosly reads raw output from the specified UART device and
parses the output according to `NMEA <https://en.wikipedia.org/wiki/NMEA_0183>`_ standards using the 
`minmea <https://github.com/kosma/minmea>`_ library included in NuttX. The driver populates the ``sensor_gnss`` struct 
and pushes it to the appropriate event once all NMEA messages in its sequence have been read.


**uORB commands**
-----------------
The driver implements the ``orb_activate``, ``orb_set_interval`` and, ``orb_ioctl`` operations to interact with the device.
The latter is used to send proprietary 'PMTK' commands which are documented further below.

**Activate**

There are 4 modes that the L86-XXX GNSS modules can be in. Those are "Full On Mode", "Standby Mode", "Backup Mode", "Periodic Mode" and, "AlwaysLocateTM Mode".
Calling ``orb_activate`` with ``enable`` set to false will enter the module into "Standby Mode". 
In "Standby Mode", the module doesn't output any NMEA messages but the internal core and I/O power domain are still active.

The module can be re-enabled by calling ``orb_activate`` with ``enable`` set to true, which will hot start the module OR by
sending any 'PMTK' command.

**Set interval**

The L86-XXX GNSS modules support interval rates from 1Hz to 10Hz (100ms - 10000ms). When using ``orb_set_interval``, be aware that
increasing the interval of the module may also require and increase in baud rate. An example of how this is performed can be found in
source code of this driver in the register function.

Any interval rate outside of the supported range will result in a failed call to this function.

**Control**

The ``orb_ioctl`` interface allows one to send proprietary 'PMTK' commands to the L86-XXX GNSS module. It effectively works
as a wrapper for the command framework outlined by Quectel. The return value of calls to ``orb_ioctl`` mimic the return values
from command acknowledgements from the GNSS module:
- 0: Invalid packet
- 1: Unsupported packet type
- 2: Valid packet, but action failed
- 3: Valid packet, action succeeded  

The supported commands are their arguments are listed below.

``CMD_HOT_START``
-----------------
Used to "Hot start" the GNSS module. There is no acknowledgement for this command.

.. code-block:: c

   orb_ioctl(sensor, CMD_HOT_START);

``CMD_WARM_START``
------------------
Used to "Warm start" the GNSS module. There is no acknowledgement for this command.

.. code-block:: c

   orb_ioctl(sensor, CMD_WARM_START);

``CMD_COLD_START``
------------------
Used to "Cold start" the GNSS module. There is no acknowledgement for this command.

.. code-block:: c

   orb_ioctl(sensor, CMD_COLD_START);

``CMD_FULL_COLD_START``
-----------------------
Used to "Full cold start" the GNSS module. There is no acknowledgement for this command.

.. code-block:: c

   orb_ioctl(sensor, CMD_FULL_COLD_START);

``CMD_STANDBY_MODE``
--------------------
Used to enter "Standby Mode". This command must be called with ``0`` as an argument. The acknowledgement for this command is handled by the ``ioctl`` call.

.. code-block:: c

   orb_ioctl(sensor, CMD_STANDBY_MODE, 0);

``SET_POS_FIX``
---------------
Used to modify the position fix interval of the GNSS module. The argument is an integer between 100 and 10000, default value is 1000. The acknowledgement for this command is handled by the ``ioctl`` call.

.. code-block:: c

   orb_ioctl(sensor, SET_POS_FIX, 1000);

``SET_NMEA_BAUDRATE``
---------------------
Used to modify the baud rate of the GNSS module. The argument is an integer representing a supported baud rate, default value is 9600. There is no acknowledgement for this command.
Upon sending this command, the baud rate of the UART interface used to communicate with the module is also modified.

.. code-block:: c

   orb_ioctl(sensor, SET_NMEA_BAUDRATE, 9600);

``FR_MODE``
-----------
Used to set the navigation mode of the GNSS module. The argument is an ``L86XXX_FR_MODE_OPTIONS`` L86XXX_FR_MODE_OPTIONS. The acknowledgement for this command is handled by the ``ioctl`` call.

.. code-block:: c

   orb_ioctl(sensor, FR_MODE, NORMAL);


