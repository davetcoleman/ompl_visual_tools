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

//ROS
#include <ros/ros.h>
#include <ros/package.h> // for getting file path for loading images
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
// This package
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
// Custom State Validity Checker with cost function
// *********************************************************************************************************
class TwoDimensionalValidityChecker : public ob::StateValidityChecker
{
private:
  std::vector<int> cost_;
  double max_threshold_;

public:

  /** \brief Constructor */
  TwoDimensionalValidityChecker( const ob::SpaceInformationPtr& si, std::vector<int> cost  ) :
    StateValidityChecker(si)
  {
    cost_ = cost;
    max_threshold_ = 10;
  }

  /** \brief Constructor */
  TwoDimensionalValidityChecker( ob::SpaceInformation* si) :
    StateValidityChecker(si)
  {
    ROS_ERROR("???");
    exit(0);
  }

  /** \brief Always return true (all states are considered valid) */
  virtual bool isValid(const ob::State * state ) const
  {
    return cost(state) < max_threshold_;
  }

  virtual double cost(const ob::State *state) const
  {
    //    const double *dims = state->as<ob::RealVectorStateSpace::StateType>()->values;
    //    dims[0] ...
    return 10;
  }

};


// *********************************************************************************************************
// Plan
// *********************************************************************************************************
std::vector<std::pair<double, double> > planWithSimpleSetup( std::vector<int> cost )
{
  // The returned result
  std::vector<std::pair<double, double> > coordinates;

  // construct the state space we are planning in
  ob::StateSpacePtr space( new ob::RealVectorStateSpace( DIMENSIONS ));
  /*
    ob::StateSpacePtr space1( new ob::DiscreteStateSpace());
    ob::StateSpacePtr space2( new ob::DiscreteStateSpace());
    ob::StateSpacePtr space = space1 + space2;

    state->as<ob::CompoundState>()->components[0]->as<ob::DiscreteStateSpace::StateType>()->value


    state->as<ob::CompoundState>()->as<ob::DiscreteStateSpace::StateType>(0)->value
  */

  // set the bounds for the R^2
  ob::RealVectorBounds bounds( DIMENSIONS );
  bounds.setLow(0);
  bounds.setHigh(99);
  //  space->as<ob::SE3StateSpace>()->setBounds(bounds);
  space->as<ob::RealVectorStateSpace>()->setBounds(bounds);



  // Define a simple setup class ---------------------------------------
  og::SimpleSetup simple_setup(space);

  // Set the setup planner (TRRT)
  simple_setup.setPlanner(ob::PlannerPtr(new og::TRRT( simple_setup.getSpaceInformation() )));

  // Set state validity checking for this space
  simple_setup.setStateValidityChecker( ob::StateValidityCheckerPtr( new TwoDimensionalValidityChecker( simple_setup.getSpaceInformation(), cost ) ) );



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
    simple_setup.getSolutionPath().interpolate();
    simple_setup.getSolutionPath().print(std::cout);

    og::PathGeometric path = simple_setup.getSolutionPath();
    std::vector<ob::State*> states = path.getStates();

    // Get planner data info
    //    ob::PlannerData data1;
    //    simple_setup.getPlannerData( data1 );


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
void displayCubes( PPMImage *image, std::vector<int> cost )
{
  // ROS Publishing stuff
  ros::NodeHandle n;
  ros::Rate rate(40);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
  visualization_msgs::MarkerArray marker_array;

  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "/my_frame";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "basic_shapes";

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = visualization_msgs::Marker::CUBE;

  // Set the marker action.  Options are ADD and DELETE
  marker.action = visualization_msgs::Marker::ADD;


  // Visualize Results -------------------------------------------------------------------------------------------------
  for( int marker_id = 0; marker_id < image->getSize(); ++marker_id )
  {
    marker.id = marker_id;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = image->data[ marker_id ].red / 255.0;
    marker.color.g = image->data[ marker_id ].green / 255.0;
    marker.color.b = image->data[ marker_id ].blue / 255.0;
    marker.color.a = 1.0;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = marker_id % image->x;    // Map index back to coordinates
    marker.pose.position.y = marker_id / image->x;    // Map index back to coordinates
    marker.pose.position.z = cost[ marker_id ] / 2;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = cost[ marker_id ];

    /*
    std::cout << "#" << marker.id
              << " COORD: "
              << marker.pose.position.x << " "
              << marker.pose.position.y << " "
              << marker.pose.position.z
              << " COLOR: "
              << marker.color.r << " "
              << marker.color.g << " "
              << marker.color.b
              << " COST: " << cost[ marker.id ]
              << std::endl;
    */
    ros::Duration(0.00001).sleep();

    //marker.lifetime = ros::Duration(10.0);
    //marker.lifetime = ros::Duration();

    marker_array.markers.push_back( marker );
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

  // Get image path based on package name
  std::string image_path = ros::package::getPath("ompl_rviz_viewer");
  if( image_path.empty() )
  {
    std::cout << "Unable to get OMPL RViz Viewer package path " << std::endl;
    exit(0);
  }
  image_path.append( "/resources/mountains.ppm" );

  // Load cost map from image file
  PPMImage *image = readPPM( image_path.c_str() );

  // Error check
  if( !image )
  {
    ROS_ERROR( "No image data loaded " );
    return false;
  }

  // Create an array of ints that represent the cost of every pixel
  std::vector<int> cost( image->getSize(), 0 );

  const int scale = 5;

  // Preprocess the pixel data for cost and give it a nice colored tint
  for( int i = 0; i < image->getSize(); ++i )
  {
    // Calculate cost
    cost[ i ]  = (image->data[ i ].red ) / scale;

    // prevent cost from being 0
    if( !cost[ i ] )
      cost[ i ] = 1;

    // Invert colors and give tint
    image->data[ i ].red = 200 - image->data[ i ].red;
    image->data[ i ].green = 255 - image->data[ i ].green;
    image->data[ i ].blue = 200 - image->data[ i ].blue;
  }


  /*

    std::cout << "Image height: " << image->y << " width: " << image->x << std::endl;


    // OMPL Processing -------------------------------------------------------------------------------------------------
    std::cout << "OMPL version: " << OMPL_VERSION << " ----------------------- " << std::endl;
    std::cout << std::endl << std::endl;

    // Get list of coordinates
    std::vector<std::pair<double, double> > coordinates = planWithSimpleSetup( cost );

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
    image->data[ image->getSize() ].red = 150;
    image->data[ image->getSize() ].green = 0;
    image->data[ image->getSize() ].blue = 0;
    }
  */

  displayCubes( image, cost );

  std::cout << "SUCCESS ------------------------------ " << std::endl << std::endl;
}

