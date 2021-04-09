^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package system_modes_examples
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
0.6.0 (2020-03-16)
-----------

* Introduced mode observer https://github.com/micro-ROS/system_modes/issues/59
* Mode manager prevents redundant mode changes https://github.com/micro-ROS/system_modes/pull/67
* Minor bugfix in inference

0.5.0 (2020-03-16)
-----------
* Atomic parameter setting https://github.com/micro-ROS/system_modes/issues/59
* Bug fixing
* More tests

0.4.2 (2020-12-17)
-----------
* Error handling and rules feature no longer experimental
* Fixed bugs in monitor and tests

0.4.1 (2020-10-29)
-----------
* Include experimental error handling and rules feature
* https://github.com/micro-ROS/system_modes/issues/13
* CI for ubuntu 20.04 ROS 2 rolling

0.4.0 (2020-09-30)
-----------
* publish inferred state and mode transitions
* https://github.com/micro-ROS/system_modes/issues/42

0.4.0 (2020-09-30)
-----------
* mode event now including start and goal mode
* publish inferred state and mode transitions
* https://github.com/micro-ROS/system_modes/issues/42

0.3.0 (2020-07-23)
-----------
* removed boost dependencies (was: program options)
* changed mode service specifications (less redundancy)
* https://github.com/micro-ROS/system_modes/issues/24

0.2.3 (2020-07-23)
-----------
* improved StateAndMode struct
* testing

0.2.2 (2020-07-13)
-----------
* introduced StateAndMode struct to bundle lifecycle state and system mode

0.2.0 (2020-02-13)
-----------
* integration with ROS 2 launch
* updated docs

0.1.6 (2019-10-31)
-------------------
* fixed QoS configuration for parameter event subscribers

0.1.5 (2019-10-21)
-------------------
* migration to ROS 2 eloquent elusor

0.1.2 (2019-03-18)
-------------------
* fixed dependencies in package.xml

0.1.1 (2019-03-08)
-------------------
* first public release for ROS 2 system modes
