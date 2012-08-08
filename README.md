OMPL RViz Viewer
==========
* Author: Dave Coleman <davetcoleman@gmail.com>
* Website: http://ros.org/wiki/ompl_rviz_viewer
* License: BSD License
* Inception Date: 8/1/2012

3-D visualizer for cost-based planning algorithms implemented with the Open Motion Planning Library (OMPL). Publishes ROS messages to the RViz visualizer software using RViz markers. Displays all explored states, connecting edges and the final solution path. Great for learning and testing robotic motion planning algorithsm.


BUILDING
---------

    rosmake ompl_rviz_viewer

RUN
---------

    rosrun ompl_rviz_viewer ompl_rviz_viewer


USAGE
---------

A default cost map image will be used, located in the resources/ folder, for running the algorithm. Optionally one can pass in their own cost map image through a command line argument, as shown in the following example:

    rosrun ompl_rviz_viewer ompl_rviz_viewer cost_map.ppm

The image must be in the PPM "Netpbm color image" format. To convert a jpg, png or any other image into this format on Linux simply use the "convert" command, as shown in the following example:

    convert cost_map.png cost_map.ppm

This convert package can be found using apt-get in Ubuntu.


DEVELOP
---------

You are encouraged to fork this package on GitHub and test your own cost-based planning algorithms using this visualizer!