^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package LMS1xx
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.3 (2015-06-15)
------------------
* Added functionality for user to specify start and stop angles to limit field of view. If
  values outside of the sensor's configured max & min angles are used the node will default
  to the configured values.
* Contributors: Nick Charabaruk

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
