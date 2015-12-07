^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ompl_visual_tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.3.1 (2015-12-07)
------------------
* catkin lint cleanup
* Contributors: Dave Coleman

2.3.0 (2015-12-05)
------------------
* Fixed API changes in rviz_visual_tools
* Contributors: Dave Coleman

2.2.1 (2015-01-07)
------------------
* Fix typo
* Added missing images
* Fix install space
* Contributors: Dave Coleman

2.2.0 (2014-10-31)
------------------
* Fix for RvizVisualTools
* Upgrade to new moveit_visual_tools API
* API changes for moveit_visual_tools
* Added gitignore
* New publishState() functions
* New publishRobotState function
* Formatting, new callback parameter
* Deprecated publishSamples functions
* Added publishSpheres functions to correspond to moveit_visual_tools ones
* Improved visualizationStateCallback
* New publishRobotGraph function
* New convertRobotStatesToTipPoints function
* Fixing display database mode
* Made publish functions return bool
* Disable 3D option
* publish cost map with non-static ID numbers
* New publishSampleIDs function. Restructured publishSamples interface to use moveit_visual_tools version
* Updated publishSampels() functions
* Reduced size of published spheres
* Contributors: Dave Coleman

2.1.1 (2014-08-11)
------------------
* Removed debug output
* Removed hard-coded base_frame name
* Improved memory usage
* Cost ptr bug fix
* Removed setOptimizationMethod()
* Contributors: Dave Coleman

2.1.0 (2014-08-08)
------------------
* Removed getSolutionPlannerName that is not available in OMPL released version
* Updated README
* Renamed file to rrt_demo.cpp
* Cleanup since change to moveit_visual_tools
* Specify OMPL version
* Fixed marker topic path
* Contributors: Dave Coleman

2.0.1 (2014-08-07)
------------------
* Updated README
* Removed lightning dependencies
* Renamed ompl_rviz_viewer to ompl_visual_tools
* Initial
* Contributors: Dave Coleman
