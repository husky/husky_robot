^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package husky_bringup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.1 (2021-09-16)
------------------
* Fix the name of teh VLP16 launch file that gets included
* Add a missing colon to the install script
* Add VLP16, secondary LMS1xx support (`#164 <https://github.com/husky/husky/issues/164>`_)
  * Minimal refactor to add VLP16 + secondary LMS1xx support. Update defaults for the laser_enabled and realsense_enabled args to refer to the underlying envars to improve consistency when launching simulations. Modify the sensor bar to allow it to be positioned in the center by default, but with configurable xyz and rpy offsets
  * Add the new run dependencies
  * Remove the prefix's trailing underscore in the vlp16 mount to make it consistent. Fix an inconsistent envar for the sensor arch, add an arg to explicitly enable it, to stay internally consistent with the rest of Husky.
  * Fix the envars; its just HUSKY_LMS1XX, not HUSKY_LASER_LMS1XX
  * Revert to enabling the main laser by default in the simulations, add the velodyne_gazebo_plugins dependency
* Contributors: Chris I-B

0.5.0 (2021-08-23)
------------------
* [husky_bringup] Switched microstrain_mips to ros_mscl.
* Fix the python-scipy dependency to refer to the python-3 version; python-scipy isn't installable on Focal
* Fix a bug where the UM7 and UM6 launch files don't work when installed to /etc/ros/*/ros.d; they fail to find the mag config files.
* Contributors: Chris Iverach-Brereton, Tony Baltovski

0.4.4 (2020-08-13)
------------------
* Set default for optenv HUSKY_MAG_CONFIG
* Removed env-hooks
* Removed Paul Bovbel as maintainer.
* Add support for some environment variables to override realsense defaults
* Sort the dependencies alphabetically
* Finish adding the simulated realsense to the topbar, add support for the physical realsense. Tidy up some parameters that were copied in last night but not yet configured.
* Mark the Kinect for Xbox 360 as deprecated, start adding support for the Intel Realsense D400 series as a replacement
* Contributors: Chris I-B, Dave Niewinski, Tony Baltovski

0.4.3 (2020-04-20)
------------------
* Add the other product ID for the PS4 controller to the udev rule
* Update the udev rules to map the controllers to appropriate symlinks instead of relying on device enumeration to save us
* Contributors: Chris I-B

0.4.2 (2019-12-11)
------------------
* [husky_bringup] Installed udev rule for Logitech controller.
* Contributors: Tony Baltovski

0.4.1 (2019-09-30)
------------------
* [husky_bringup] Enabled using MagnenticField message for UM6 and UM7 since imu_filter_madgwick now uses it by default.
* Added Udev rule for Logitech joy. (`#116 <https://github.com/husky/husky/issues/116>`_)
* Contributors: Tony Baltovski

0.4.0 (2019-08-01)
------------------

0.3.4 (2019-08-01)
------------------
* Properly support GX5.
* Contributors: Dave Niewinski

0.3.3 (2019-04-18)
------------------

0.3.2 (2019-03-25)
------------------
* [husky_bringup] Disabled the use of magnetic field msgs in imu_filter_madgwick.
* Contributors: Tony Baltovski

0.3.1 (2018-08-02)
------------------
* Renamed udev so it is installed
* Contributors: Dave Niewinski, Tony Baltovski

0.3.0 (2018-04-11)
------------------
* Remove defunct email address
* Re-added microstrain_3dmgx2_imu as run  dependency.
* Re-added imu_transformer and um7 as run dependencies.
* Updated maintainers.
* Changed the name of robot_upstart job to ros.
* Added the UM6 as a run dep.
* Purge more UR; Implement urdf_extras
* Move packages into monorepo for kinetic; strip out ur packages
* Contributors: Paul Bovbel, Tony Baltovski

0.6.8 (2022-12-01)
------------------
* Removed manual definition of base_frame_id
* [husky_bringup] Updated compute_calibration to use MagneticField message.
* Fix MagneticField msg reading
* Contributors: Luis Camero, Saurav Agarwal, Tony Baltovski

0.6.7 (2022-06-16)
------------------
* Fixes for velodyne prefix
* Secondary sensors (`#24 <https://github.com/husky/husky_robot/issues/24>`_)
  * Added second blackfly to launch
  * Fixed prefix in UST10 launch
  * Added remap to launch file to get desired topics
  * Launch file now launches 2 Reaslenses directly using Nodelet launch
  * Added second 3D laser to launch file
  * Added entries for secondary sensors
  * Added new line at EOF
* Use configurable laser prefix
  husky_description supports changing the frame ID, but without being able to change the frame ID in the launch file you wind up with a broken tf
  - https://github.com/husky/husky/blob/2c46d3b0d4815bdf4a8973d62439657155f831da/husky_description/urdf/husky.urdf.xacro#L35
  - https://github.com/husky/husky/blob/2c46d3b0d4815bdf4a8973d62439657155f831da/husky_description/urdf/husky.urdf.xacro#L42
  Also, the existing default "base_laser" is inconsistent with the latest husky_description.
  IndoorNav, because of how Otto's implemented parts of it, currently requires the front & rear lidar frames to be `front_laser` and `rear_laser` respectively, so fixing this will also make IndoorNav easier to develop/maintain.
* Contributors: Chris I-B, Luis Camero, luis-camero

0.6.6 (2022-05-17)
------------------

0.6.5 (2022-05-17)
------------------
* Added Blackfly entry to install script
* Added Blackfly launch file
* Added spinnaker_camera_driver to package.xml
* Add HUSKY_REALSENSE_TOPIC envar for choosing prefix namespace for all realsense topics
* Update realsense launch file based on changes from realsense2_camera
* Contributors: Joey Yang, Luis Camero

0.6.4 (2022-03-21)
------------------
* [husky_bringup] Updated compute_calibration script to explicitly use Python3.
* [husky_bringup] Updated install script to explicitly use Python3.
* Contributors: Tony Baltovski

0.6.3 (2022-02-17)
------------------
* [husky_bringup] Removed udev folder from CMakeLists.txt.
* Contributors: Tony Baltovski

0.6.2 (2022-02-15)
------------------
* Removed HUSKY_IMU_LINK since it can be strictly imu_link
* Removed udev rules from bringup package
* Removed references to microstrain_mips, now use ros_mscl
* Bump CMake version to avoid CMP0048 warning.
* Remove unnecessary PS4, Logitech udev rules.
  These were previously removed from Melodic; not sure why they were re-added for Noetic, but I suspect it was a copy-paste error
* Contributors: Chris I-B, Luis Camero, Tony Baltovski

0.6.1 (2022-01-18)
------------------
* Added Hokuyo
* Contributors: Luis Camero

0.6.0 (2021-09-28)
------------------
* Re-added husky_robot from husky.
* Contributors: Tony Baltovski

0.2.6 (2016-10-03)
------------------
* Adding support for the UM7 IMU.
* Added new ur_modern_driver
* Added param for laser frame_id.
* Contributors: TheDash, Tony Baltovski

0.2.5 (2015-12-31)
------------------

0.2.4 (2015-07-08)
------------------
* Fix laser path
* Contributors: Paul Bovbel

0.2.3 (2015-04-08)
------------------
* Integrate husky_customization workflow
* Contributors: Paul Bovbel

0.2.2 (2015-03-23)
------------------
* Fix package urls
* Contributors: Paul Bovbel

0.2.1 (2015-03-23)
------------------

0.2.0 (2015-03-23)
------------------
* Add UR5 bringup
* Contributors: Paul Bovbel, Devon Ash

0.1.2 (2015-02-12)
------------------
* Namespace fixes
* Contributors: Paul Bovbel

0.1.1 (2015-01-30)
------------------
* Update website and authors
* Add transform to transfer IMU data to base_link frame
* Make ROBOT_NETWORK optional
* Switch to robot_upstart python API
* Switch to debhelper install method for udeb rules
* Switch to env-hook for file storage
* Switch to new calibration method for um6; switch to imu_filter_magwick
* Contributors: Paul Bovbel

0.1.0 (2015-01-13)
------------------
* Port to robot_localization, gyro only pending um6 fixes
* changed the launch file to match parameter namespace changes in the imu_compass node
* ported kingfisher compass calibration to husky
* Added Microstrain device condition - Looks for an attached Microstrain device and installs the necessary launch files from the microstrain_config directory.
* Update sick.launch - Fixed binary name
* Change default IP for LIDAR to 192.168.1.14
* Add launcher for sick LIDAR.
* Added Microstrain launch file and udev rule.
* Contributors: Jeff Schmidt, Mike Purvis, Paul Bovbel, Prasenjit Mukherjee

0.0.6 (2013-10-12)
------------------
* Restore leading slash in checking the joystick path.
  This was removed by mistake in an earlier commit.

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
