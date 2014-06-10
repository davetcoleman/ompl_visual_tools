OMPL RViz Viewer
==========

## What does the OMPL RViz Viewer do?

It is a tool for developing cost-based algorithms for the [Open Motion Planning Library](http://ompl.kavrakilab.org/) 
that allows you to easily visualize the 
algorithm in a two dimensional space. The space is specified as a grey scale image that can be passed in to the program. 
The lighter (closer to white) each pixel of the image is, the "higher the cost" is considered to be. Black is considered no cost. 
Additionally, absolute obstacles can be specified by defining a max limit to the cost, such that any value above that threshold is considered an obstacle.

This little program is similar to the OMPL.app that is distributed with OMPL, but instead uses RViz for visualization and is more 
streamlined for considering costs.

<img align="right" src="https://raw.githubusercontent.com/davetcoleman/ompl_rviz_viewer/master/resources/ompl_rviz_viewer.png" />

## How to Build

Clone into your [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) and rebuild.

## How to Run

Start Rviz using the included launch file:

```
roslaunch ompl_rviz_viewer ompl_rviz.launch
```

Then run one of the OMPL RViz planner:

### Cost-based planner with Transition-based RRT:
```
roslaunch ompl_rviz_viewer ompl_rviz_planner.launch
```

### Experienced based-planning with Lightning Framework

```
roslaunch ompl_rviz_viewer ompl_rviz_lightning.launch
```

## Cost Map Usage

A default cost map image will be used, located in the resources/ folder, for running the algorithm. 
Optionally one can pass in their own cost map image through a command line argument, as shown in the following example:

```
rosrun ompl_rviz_viewer ompl_rviz_planner cost_map.ppm
```

The image must be in the PPM "Netpbm color image" format. To convert a jpg, png or any other image into this format on Linux simply use the "convert" command, as shown in the following example:

```
sudo apt-get install convert
convert cost_map.png cost_map.ppm
```

In general, a 100x100 pixel image is a decent space size, and larger dimensions will require much more computational resources or may hang your computer. GIMP is a good editor for scaling images down.

## TODO

- Remove PPM library and instead depend on OMPL's version

## Develop

You are encouraged to fork this package on GitHub and test your own cost-based planning algorithms using this visualizer!