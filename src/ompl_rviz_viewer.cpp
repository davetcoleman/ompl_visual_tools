/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman */


#include <ros/ros.h>
#include <ros/package.h> // for getting file path for loading images
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "ompl_rviz_viewer/ppm.h" // for reading image files
// OMPL
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/config.h>
#include <iostream>

namespace ob = ompl::base;
namespace og = ompl::geometric;

const unsigned int DIMENSIONS = 2;

// *********************************************************************************************************
// Decide if path is valid
// *********************************************************************************************************
bool isStateValid(const ob::State *state)
{
  /*
  // cast the abstract state type to the type we expect
  const ob::SE3StateSpace::StateType *se3state = state->as<ob::SE3StateSpace::StateType>();

  // extract the first component of the state and cast it to what we expect
  const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

  // extract the second component of the state and cast it to what we expect
  const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

  // check validity of state defined by pos & rot


  // return a value that is always true but uses the two variables we define, so we avoid compiler warnings
  return (void*)rot != (void*)pos;
  */

  // There are no obstacles in this world...
  return true;
}

// *********************************************************************************************************
// Plan
// *********************************************************************************************************
std::vector<std::pair<double, double> > planWithSimpleSetup(void)
{
  // The returned result
  std::vector<std::pair<double, double> > coordinates;

  // construct the state space we are planning in
  ob::StateSpacePtr space( new ob::RealVectorStateSpace( DIMENSIONS ));

  // set the bounds for the R^2
  ob::RealVectorBounds bounds( DIMENSIONS );
  bounds.setLow(0);
  bounds.setHigh(99);
  //  space->as<ob::SE3StateSpace>()->setBounds(bounds);
  space->as<ob::RealVectorStateSpace>()->setBounds(bounds);



  // Define a simple setup class ---------------------------------------
  og::SimpleSetup simple_setup(space);

  // Set the setup planner (TRRT)
  simple_setup.setPlanner(ob::PlannerPtr(new og::TRRT(simple_setup.getSpaceInformation())));

  // Set state validity checking for this space
  simple_setup.setStateValidityChecker(boost::bind(&isStateValid, _1));


  // Start and Goal State ---------------------------------------------

  // Create start space
  ob::ScopedState<> start(space);
  //start.random();

  // Manually set the start location
  start[0] = 95;
  start[1] = 10;

  // create a goal state
  ob::ScopedState<> goal(space);
  //goal.random();
  // Manually set the start location
  goal[0] = 10;
  goal[1] = 40;

  // set the start and goal states
  simple_setup.setStartAndGoalStates(start, goal);


  // Debug -----------------------------------------------------------

  // this call is optional, but we put it in to get more output information
  simple_setup.setup();
  simple_setup.print();

  std::cout << "\n\nSolve\n" << std::endl;




  // Solve -----------------------------------------------------------

  // attempt to solve the problem within one second of planning time
  ob::PlannerStatus solved = simple_setup.solve(1.0);

  if (solved)
  {
    // print the path to screen
    simple_setup.simplifySolution();

    std::cout << "\nFOUND SOLUTION:" << std::endl;
    simple_setup.getSolutionPath().print(std::cout);

    og::PathGeometric path = simple_setup.getSolutionPath();
    std::vector<ob::State*> states = path.getStates();

    std::cout << "\nSOLUTION:" << std::endl;
    for( std::vector<ob::State*>::const_iterator state_it = states.begin();
         state_it != states.end(); ++state_it )
    {
      const ob::State *state = *state_it;

      if (!state)
        continue; // no data?
      
      // Convert to RealVectorStateSpace
      const ob::RealVectorStateSpace::StateType *real_state =
        static_cast<const ob::RealVectorStateSpace::StateType*>(state);

      // Add to vector of results
      coordinates.push_back( std::pair<double,double>( real_state->values[1], real_state->values[0] ) );
    }
  }
  else
  {
    std::cout << "NO SOLUTION FOUND" << std::endl;
  }

  return coordinates;
}

// *********************************************************************************************************
// Visualize Results
// *********************************************************************************************************
void displayCubes( PPMImage *image )
{
  ros::NodeHandle n;
  ros::Rate rate(40);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);

  visualization_msgs::MarkerArray marker_array;

  const int scale = 5;

  // Visualize Results -------------------------------------------------------------------------------------------------
  for( int marker_id = 0; marker_id < image->x * image->y; ++marker_id )
  {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/my_frame";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = marker_id;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::CUBE;

    // Set the marker action.  Options are ADD and DELETE
    marker.action = visualization_msgs::Marker::ADD;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = (200 - image->data[ marker_id ].red) / 255.0;
    marker.color.g = (200 - image->data[ marker_id ].green) / 255.0;
    marker.color.b = (100 - image->data[ marker_id ].blue) / 255.0;
    marker.color.a = 1.0;

    int cost = (image->data[ marker_id ].red ) / scale;
    // prevent cost from being 0
    if( !cost )
      cost = 1;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = image->getX( marker_id );    // Map index back to coordinates
    marker.pose.position.y = image->getY( marker_id );    // Map index back to coordinates
    marker.pose.position.z = cost / 2;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = cost;

    //    std::cout << "#" << marker.id << " COST: " << cost
    //      << " COORD: "
    //      << marker.pose.position.x << " "
    //      << marker.pose.position.y << " "
    //      << marker.pose.position.z << " "
    //         << " COLOR: "
    //         << marker.color.r << " "
    //         << marker.color.g << " "
    //         << marker.color.b << " "
    //              << std::endl;
    ros::Duration(0.00001).sleep();

    //marker.lifetime = ros::Duration(10.0);
    //marker.lifetime = ros::Duration();

    //marker_pub.publish(marker);
    marker_array.markers.push_back( marker );

    //   if( marker_id > 100 )
    //      break;
  }

  // Publish the marker array
  marker_pub.publish( marker_array );

  ros::Duration(1).sleep();
}

// *********************************************************************************************************
// Main
// *********************************************************************************************************
int main( int argc, char** argv )
{
  ros::init(argc, argv, "basic_shapes");

  /*
    std::string image_path = ros::package::getPath("ompl_rviz_viewer");
    if( image_path.empty() )
    {
    std::cout << "Unable to get OMPL RViz Viewer package path " << std::endl;
    exit(0);
    }
    image_path.append( "/resources/height_map1.ppm" );
    std::cout << image_path << std::endl;
  */

  // Image file path
  std::string image_path = "../resources/mountains.ppm";

  // Load cost map from image file
  PPMImage *image = readPPM( image_path.c_str() );

  // Error check
  if( !image )
  {
    ROS_ERROR( "No image data loaded " );
    return false;
  }

  std::cout << "Image height: " << image->y << " width: " << image->x << std::endl;


  // OMPL Processing -------------------------------------------------------------------------------------------------

  std::cout << "OMPL version: " << OMPL_VERSION << " ----------------------- " << std::endl;
  std::cout << std::endl << std::endl;

  // Get list of coordinates
  std::vector<std::pair<double, double> > coordinates = planWithSimpleSetup();

  // Convert path coordinates to red
  for( std::vector<std::pair<double, double> >::const_iterator coord_it = coordinates.begin();
       coord_it != coordinates.end(); ++coord_it )
  {
    int id = image->getID( int( coord_it->first ), int(coord_it->second) );
    image->data[ id ].red = 255;
    image->data[ id ].green = 0;
    image->data[ id ].blue = 0;
  }

  // Manually override path colors for first and last
  if( coordinates.size() > 1 )
  {
    // Start
    image->data[ 0 ].red = 100;
    image->data[ 0 ].green = 255;
    image->data[ 0 ].blue = 0;

    // End
    image->data[ image->getMaxID() ].red = 150;
    image->data[ image->getMaxID() ].green = 0;
    image->data[ image->getMaxID() ].blue = 0;
  }

  displayCubes( image );

  std::cout << "SUCCESS ------------------------------ " << std::endl << std::endl;
}


