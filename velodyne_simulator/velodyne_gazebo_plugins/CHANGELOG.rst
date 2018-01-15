^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package velodyne_gazebo_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.6 (2017-10-17)
------------------
* Use robotNamespace as prefix for PointCloud2 topic frame_id by default
* Use Gazebo LaserScan message instead of direct LaserShape access, fixes timestamp issue
* Contributors: Kevin Hallenbeck, Max Schwarz, Micho Radovnikovich

1.0.5 (2017-09-05)
------------------
* Fixed ground plane projection by removing interpolation
* Contributors: Kevin Hallenbeck, Micho Radovnikovich

1.0.4 (2017-04-24)
------------------
* Updated package.xml format to version 2
* Removed gazebo_plugins dependency
* Contributors: Kevin Hallenbeck

1.0.3 (2016-08-13)
------------------
* Gazebo7 integration
* Contributors: Kevin Hallenbeck, Konstantin Sorokin

1.0.2 (2016-02-03)
------------------
* Display laser count when loading gazebo plugin
* Don't reverse ring for newer gazebo versions
* Changed to PointCloud2. Handle min and max range. Noise. General cleanup.
* Start from block laser
* Contributors: Kevin Hallenbeck
