^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rotors_control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

4.0.6 (2019-01-04)
------------------
* Changes in the position controller libray to fix issue in the building process
* Contributors: Giuseppe Silano

4.0.5 (2018-12-17)
------------------
* The position_controller_node_with_stateEstimator and position_controller_node_without_stateEstimator files were rewritten as an unique controller: the position_controller_node
* The comments have been reviewed improving the readability of the code
* Some bugs have been fixed
* Contributors: Giuseppe Silano

4.0.4 (2018-09-30)
------------------
* Names update. Some cpp files have been renamed in a more general way making them independent of Crazyflie (e.g., the complementary filter).
* Contributors: Giuseppe Silano

4.0.3 (2018-06-04)
------------------
* bug fixing in the controller position node and in the controller position, complementary filter and sensfusion6 algorithms
* the controller position node algorithm has been split into two parts: with and without state estimator
* the position controller algorithm now is able to handle both ideal and real simulations (with and without real sensors)
* the control algorithm has been split into two parts: high and low level. In this way, the high level part can be restructured without changes in the low one. Indeed, the Crazyflie on board control architectured does not change unless the firmware is changed
* Contributors: Giuseppe Silano, Luigi Iannelli

4.0.2 (2018-02-23)
------------------
* added the Crazyflie 2.0 default state estimator: complementary filter, in according to the last firmware release 2018.01.01
* modified the position controller in order to take into account the state estimator
* Contributors: Giuseppe Silano, Luigi Iannelli

4.0.1 (2018-02-02)
------------------
* added Crazyflie 2.0 position controller. The lower level controller is the same of the Crazyflie 2.0 firmware (released 2018.01.01)
* started from the 2.1.1 (2017-04-27) release version of RotorS
* Contributors: Giuseppe Silano, Emanuele Aucone, Benjamin Rodriguez, Luigi Iannelli

