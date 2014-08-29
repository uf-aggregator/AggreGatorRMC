#Workspace

This is the ROS workspace for all of the packages.

------------------------------------------------

##Compilation Notice

Due to the interpackage dependencies and our current CMakeList setup, the workspace must be `catkin_make`'d several times, until the dependencies are generated.
<br>
The source of this is likely due to the lack of dependency exporting in the `motor_controller` package.
<br>
__Note:__ If any of the code has compilation errors, then the workspace will not compile at all. Even the message dependencies.
<p style="padding-bottom: 1em;"></p>

------------------------------------------------
###hardware_interface
This package contains the drivers for reading and writing to various hardware, including: 
- I2C 
- AdaFruit
- Analog-to-Digital Converters 
 
<br>
###ladar
This package contains the processing logic for the `/scan` topic feeding on the Hokuyo Laser Range Finder, our LADAR. The purposes for this package is providing condition and visual info for other packages.


<br>
###mission_control
This package contains the "brains," e.g. the control logic, for the entire system.  Any changes to how the robot makes decisions, will be decided here.

<br>
###motor_controller
This package contains the driver logic for the motor controllers. Currently, they are customized for the Pololu's we are using.

<br>
###power_monitoring
This package monitors power usage and will eventually be able to shut down processes to save battery power.

<br>
###remote_control
This is the remote control logic for the AggreGator. 
The current setup uses a Microsoft wifi adapter that syncs up to an Xbox 360 controller. This relies on the `joy` library to function.

<br>
###third_party_pkgs
This contains all third party packages used in our system. They were all obtained from their source repositories and are included to reduce the need to download the packages on whatever development board we use.