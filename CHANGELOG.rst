^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package LMS1xx
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.1 (2023-06-30)
------------------
* Added statemachine to prevent driver from blocking executor.
* Contributors: Roni Kreinin

1.0.0 (2023-06-15)
------------------
* Removed TravisCI badge.
* Fixed CMakeLists.txt finding deleted packages
* Fixed formatting issues and dependencies
* Update ROS2CI.yml
* Create ROS2CI.yml
* Create depends.repos
* Improvements on connecting to lidars
* add back sprintf but dont printf
* comment out sprintf
* Fix min max angle
* Fix qos for gazebo simulation
* Make gazebo simulation work
* Add humble support for Husky LMS_111 lidars
* migration to ros2 foxy
* Contributors: Daniel Duran-Rojas, Fredrik, SRai22, Samuel Lindgren, Tony Baltovski, fazzrazz

0.3.0 (2021-01-24)
------------------
* Changed shebang to python3.
* Fixed typo in find_sick script.
* Update the python scripts so they'll work with python3
* Switched to industrial_ci for TravisCI.
* Contributors: Chris I-B, Tony Baltovski

0.2.0 (2019-07-16)
------------------
* Updates for console bridge deprecated macros
* Fixed warning about inconsistent namespace redefinitions for xmlns:xacro.
* Fixed formatting
* Added inf to be set for min range
* Contributors: Dave Niewinski, Tony Baltovski

0.1.6 (2017-05-04)
------------------
* Added max/min angle to gazebo plugin.
* [ros params] Adding 'port' parameter. (`#35 <https://github.com/clearpathrobotics/LMS1xx/issues/35>`_)
* Add robot namespace to lidar plugin
* Exposed parameters min_range and max_range
* add parameters 'sample_size' and 'update_rate' for gazebo's ray plugin
* Contributors: Achim, CyrillePierre, Mike Purvis, Paul Bovbel, Tony Baltovski, bikramak@aandkrobotics.com

0.1.5 (2015-12-18)
------------------
* Rework startup/reconnection logic.
* Add roslaunch, roslint checks.
* Other fixes, including a new buffer class with tests.
* Add travis.
* Apply astyle to original package.
* Remove screen attribute from example launcher.
* Contributors: Mike Purvis

0.1.4 (2015-09-04)
------------------
* Another startup bugfix.
* Contributors: Tony Baltovski

0.1.3 (2015-07-16)
------------------
* Added URDF with simulation plugin.
* Added script to set static IP of LMS1xx
* Fixed startup publishing issue
* Removed one second delay in login
* Contributors: Mike Purvis, Mustafa Safri, Tony Baltovski

0.1.2 (2015-01-20)
------------------
* More robust startup for LMS1xs, retries instead of just dying.
* Switch to console_bridge logouts, separate lib.
* Remove buffer flush at conclusion.
* Use LMS1xx reported configuration instead of capabilities for angle_min, max, etc...
* Use output range query for min & max angles, num_values, and time_increment.
  Previously, the code would query the LMS1xx for its capabilities, which
  might exceed its current configuration (in terms of angle range).  Now, we
  query the LMS1xx for its configuration when setting scan parameters such
  as min & max angle, number of values reported, and time increment.
* Add getScanOutputRange() to read outputRange data from the LMS1xx.
* Contributors: Mike Purvis, Patrick Doyle

0.1.1 (2014-08-18)
------------------
* Changed the pkg and name so that the launch file is able to find the package.
* Fix assignment in if() to comparison.
* Contributors: Jeff Schmidt, Mike Purvis

0.1.0 (2014-02-27)
------------------
* Fix example launch file and install it.
* Fix project name in CMakeLists.
* Adding original author, readme
* Contributors: Mike Purvis

0.0.1 (2014-02-27)
------------------
* Initial release
* Contributors: MLefebvre, Mike Purvis, Prasenjit Mukherjee
