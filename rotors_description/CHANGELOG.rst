^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rotors_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

4.0.6 (2019-01-04)
------------------

4.0.5 (2018-12-17)
------------------

4.0.4 (2018-09-30)
------------------

4.0.3 (2018-06-04)
-------------------
* An if else structure has been added to the the crazyflie_base.xacro file. Such structure allows to able or unable thw real sensors based on the simulation request. In other words, when the complementary filter is unable, the ideal sensors are simulated and vice-versa.
* Contributors: Giuseppe Silano

4.0.2 (2018-02-23)
-------------------
* added the Crazyflie 2.0 IMU (MPU-9250) inside the component_snippest.xacro file. The IMU substitutes the ideal sensor previously employed in the crazyfie_base.xacro
* Contributors: Giuseppe Silano, Luigi Iannelli

4.0.1 (2018-02-02)
------------------
* initial Ubuntu package release
* added Crazyflie 2.0 urdf: crazyflie2.xacro and crazyflie2_base.xacro. The MPU-9250 IMU plugin is under developing.
* added Crazyflie 2.0 mesh file
* Contributors: Giuseppe Silano, Emanuele Aucone, Benjamin Rodriguez, Luigi Iannelli

