^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rotors_hil_interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.2.3 (2018-12-13)
------------------

2.2.2 (2018-12-12)
------------------
* Merge branch 'feature/hil_mavros_fix' into feature/gazebo9-autobackport
* fix not including message types
* Contributors: Zachary Taylor, michaelpantic

2.2.0 (2018-12-10)
------------------
* Merge pull request `#430 <https://github.com/ethz-asl/rotors_simulator/issues/430>`_ from ethz-asl/fix/hil_eigen
  Fix compile error with new Eigen on implicit Eigen typecasts.
* Fix compile error with new Eigen on implicit Eigen typecasts.
* Merge pull request `#397 <https://github.com/ethz-asl/rotors_simulator/issues/397>`_ from ethz-asl/v2.1.1
  update to 2.1.1
* Contributors: Helen Oleynikova, Mina Kamel

2.1.1 (2017-04-27)
-----------
* Update maintainers.
* Contributors: Timo Hinzmann

2.1.0 (2017-04-08)
-----------
* fixed typo in rotors_hil_interface CMakeLists.txt
* Fix HIL interface compilation.
* Fixed erroneous child variable name changes (unit postfixes) for MAVlink variable types.
* Added even more unit prefixes to variables names.
* Updated more variable names with unit prefixes.
* Added unit suffixes to variable names.
* Updated a u_int64_t to uint64_t. Also check to see if I have write privliages to repo.
* Added slightly more descriptive reason as to why HIL interface would be skipped.
* Replacing some default topic names with more general mav_comm defaults
* Refactoring fixes
* Fixing rotors_hil installation
* Adding rosinstall file for HIL configuration
* Adding the hil_interface package
* Contributors: Fadri Furrer, Geoffrey Hunter, Helen Oleynikova, Pavel
