^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package husky_base
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
