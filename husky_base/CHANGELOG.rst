^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package husky_base
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.2 (2015-03-23)
------------------
* Fix package urls
* Contributors: Paul Bovbel

0.2.1 (2015-03-23)
------------------
* Add missing dependencies
* Contributors: Paul Bovbel

0.2.0 (2015-03-23)
------------------
* Add UR5_ENABLED envvar
* Contributors: Paul Bovbel

0.1.5 (2015-02-19)
------------------
* Fix duration cast
* Contributors: Paul Bovbel

0.1.4 (2015-02-13)
------------------
* Correct issues with ROS time discontinuities - now using monotonic time source
* Implement a sane retry policy for communication with MCU
* Contributors: Paul Bovbel

0.1.3 (2015-01-30)
------------------
* Update description and maintainers
* Contributors: Paul Bovbel

0.1.2 (2015-01-20)
------------------
* Fix library install location
* Contributors: Paul Bovbel

0.1.1 (2015-01-13)
------------------
* Add missing description dependency
* Contributors: Paul Bovbel

0.1.0 (2015-01-12)
------------------
* Fixed encoder overflow issue
* Ported to ros_control for Indigo release
* Contributors: Mike Purvis, Paul Bovbel, finostro

0.0.5 (2013-10-04)
------------------
* Mark the config directory to install.

0.0.4 (2013-10-03)
------------------
* Parameterize husky port in env variable.

0.0.3 (2013-09-24)
------------------
* Add launchfile check.
* removing imu processing by dead_reckoning.py
* removing dynamic reconfigure from dead_reckoning because it was only there for handling gyro correction
* adding diagnostic aggregator and its related config file under config/diag_agg.yaml

0.0.2 (2013-09-11)
------------------
* Fix diagnostic_msgs dependency.

0.0.1 (2013-09-11)
------------------
* New husky_base package for Hydro, which contains the nodes
  formerly in husky_bringup.
