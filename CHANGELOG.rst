^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package husky_bringup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.5 (2013-10-05)
------------------
* Acknowledge the ROBOT_SETUP env variable in the install script.

0.0.4 (2013-10-03)
------------------
* Remove the other launchfile check until we get a chance to fix the config location issue.
* adding installation of ekf yaml file to install script
* better parameters for husky compass calibration based on standard husky configurations
* combining both ekf launchers into one and relying on a config file to to pick whether we want an outdoor or indoor ekf to start
* allowing the user to scale the gps data if desired
* adding parameter to lock the altitude at 0
* set invalid covariance value for enu to really high, instead of -1

0.0.3 (2013-10-01)
------------------
* Add sicktoolbox_wrapper in advance of a config for standard LIDARs.
* Parameterize from environment variables the IMU and GPS ports, and network interface to launch from.

0.0.2 (2013-09-23)
------------------
* Compass startup and inertial ekf
* adding magnetometer configuration file to husky_bringup
* added static transform to um6 launcher
* Set namespace to navsat, baud rate to 9600.
* Depend on robot_upstart.
* Add automatic launchfile checks.

0.0.1 (2013-09-13)
------------------
* Catkinize package.
* First cut of a new install script.
