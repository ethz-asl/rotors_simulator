^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rotors_gazebo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.1 (2015-08-10)
------------------
* fixed the bag plugin and the evaluation
* Contributors: Fadri Furrer

2.0.0 (2015-08-09)
------------------
* Change to use the new datatypes defined in mav_comm.
* Update hummingbird.yaml
  The original 0.68 did not involve the mass of rotors (0.009 each). Now change to 0.68 + 0.009*4
* Added subscriber for MultiDOFJointTrajectory messages and extended waypoint_publisher_file to test it. resolves `#243 <https://github.com/ethz-asl/rotors_simulator/issues/243>`_
* Contributors: Haoyao Chen, Helen Oleynikova, Markus Achtelik

1.1.6 (2015-06-11)
------------------

1.1.5 (2015-06-09)
------------------
* added install targets

1.1.4 (2015-05-28)
------------------

1.1.3 (2015-05-28)
------------------

1.1.2 (2015-05-27)
------------------

1.1.1 (2015-04-24)
------------------

1.1.0 (2015-04-24)
------------------
* initial Ubuntu package release
