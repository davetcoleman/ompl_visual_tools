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
//#include <ompl/base/PlannerDataGraph.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/config.h>
#include <iostream>
// Boost
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>
//#include <boost/graph/graph_traits.hpp> // ??
//#include <boost/graph/adjacency_list.hpp> // ??
//#include <boost/graph/graphviz.hpp> // remove
//#include <boost/graph/graphml.hpp> // remove
//#include <boost/graph/prim_minimum_spanning_tree.hpp> // ??

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace bnu = boost::numeric::ublas;

// *********************************************************************************************************
// *********************************************************************************************************
// Custom State Validity Checker with cost function
// *********************************************************************************************************
// *********************************************************************************************************
class TwoDimensionalValidityChecker : public ob::StateValidityChecker
{
private:
  bnu::matrix<int> cost_;
  double max_threshold_;

public:

  /** \brief Constructor */
  TwoDimensionalValidityChecker( const ob::SpaceInformationPtr& si, const bnu::matrix<int> cost  ) :
    StateValidityChecker(si)
  {
    cost_ = cost;
    max_threshold_ = 255;
  }

  /** \brief Constructor */
  TwoDimensionalValidityChecker( ob::SpaceInformation* si) :
    StateValidityChecker(si)
  {
    ROS_ERROR("NOT IMPLEMENTED constructor");
    exit(0);
  }

  /** \brief Always return true (all states are considered valid) */
  virtual bool isValid(const ob::State * state ) const
  {
    return cost(state) < max_threshold_;
  }

  virtual double cost(const ob::State *state) const
  {
    //    const double *coords = state->as<ob::RealVectorStateSpace::StateType>()->values;

    // Return the cost from the matrix at the current dimensions
    //return cost_( int(coords[0]), int(coords[1]) );
    return 0;
  }

};

// *********************************************************************************************************
// *********************************************************************************************************
// Rviz Visualizer Class
// *********************************************************************************************************
// *********************************************************************************************************
class OmplRvizViewer
{

private:
  /// The RGB image data
  PPMImage *image_;

  /// The cost for each x,y - which is derived from the RGB data
  bnu::matrix<int> cost_;

  /// The resulting graph that was searched
  ob::PlannerData *planner_data_;

public:

  // *********************************************************************************************************
  // Constructor
  // *********************************************************************************************************
  OmplRvizViewer() {}

  // *********************************************************************************************************
  // Deconstructor
  // *********************************************************************************************************
  ~OmplRvizViewer()
  {
    delete image_;
    delete planner_data_;
  }

  // *********************************************************************************************************
  // Main Function
  // *********************************************************************************************************
  void runImage( std::string image_path )
  {
    // Load cost map from image file
    image_ = readPPM( image_path.c_str() );

    // Error check
    if( !image_ )
    {
      ROS_ERROR( "No image data loaded " );
      return;
    }

    // Create an array of ints that represent the cost of every pixel
    cost_.resize( image_->x, image_->y );

    const int scale = 5;
    int max_cost = 0;

    // Preprocess the pixel data for cost and give it a nice colored tint
    for( int i = 0; i < image_->getSize(); ++i )
    {
      // Calculate cost
      cost_.data()[i]  = ( image_->data[ i ].red ) / scale;

      // Prevent cost from being zero
      if( !cost_.data()[i] )
        cost_.data()[i] = 1;

      // Remember largest cost
      if( cost_.data()[i] > max_cost )
        max_cost = cost_.data()[i];

      // Invert colors and give tint
      /*    image->data[ i ].red = 200 - image->data[ i ].red;
            image->data[ i ].green = 200 - image->data[ i ].green;
            image->data[ i ].blue = 100 - image->data[ i ].blue;*/
      image_->data[ i ].red = image_->data[ i ].red;
      image_->data[ i ].green = image_->data[ i ].green;
      image_->data[ i ].blue = image_->data[ i ].blue;
    }


    std::cout << "MAX COST: " << max_cost << std::endl;


    std::cout << "Map height: " << image_->y << " width: " << image_->x << std::endl;

    // OMPL Processing -------------------------------------------------------------------------------------------------
    std::cout << "OMPL version: " << OMPL_VERSION << " ----------------------- " << std::endl;
    std::cout << std::endl << std::endl;

    // Get list of coordinates
    std::vector<std::pair<double, double> > coordinates = planWithSimpleSetup();

    // Convert path coordinates to red
    for( std::vector<std::pair<double, double> >::const_iterator coord_it = coordinates.begin();
         coord_it != coordinates.end(); ++coord_it )
    {
      int id = image_->getID( int( coord_it->first ), int(coord_it->second) );
      image_->data[ id ].red = 255;
      image_->data[ id ].green = 0;
      image_->data[ id ].blue = 0;
    }

    // Manually override path colors for first and last
    if( coordinates.size() > 1 )
    {
      // Start
      image_->data[ 0 ].red = 0;
      image_->data[ 0 ].green = 0;
      image_->data[ 0 ].blue = 255;

      // End
      image_->data[ image_->getSize() ].red = 150;
      image_->data[ image_->getSize() ].green = 0;
      image_->data[ image_->getSize() ].blue = 0;
    }

    // Actually render the map
    displayTriangles();

    // Now visualize the explored space
    //    boost::concepts::Graph boost_graph = planner_data.toBoostGraph();
    //unsigned int ompl::base::PlannerData::getEdges (unsigned int v, std::vector<unsigned int>& edgeList) const

    // Get the starting vertex
    ob::PlannerDataVertex pd_vertex = planner_data_->getStartVertex(0); // there is only 1 start vertex

    // Get its coordinates
    const ob::State *state = pd_vertex.getState();

    if (state)
    {
      // Convert to RealVectorStateSpace
      const ob::RealVectorStateSpace::StateType *real_state =
        static_cast<const ob::RealVectorStateSpace::StateType*>(state);
      std::cout << "Before " << std::endl;

      std::cout << "X: " << real_state->values[0] << std::endl;
      std::cout <<" Y: " << real_state->values[1] << std::endl;
    }

    std::cout << "SUCCESS ------------------------------ " << std::endl << std::endl;
  }

  // *********************************************************************************************************
  // Private
  // *********************************************************************************************************
private:
  static const unsigned int DIMENSIONS = 2;

  // *********************************************************************************************************
  // Plan
  // *********************************************************************************************************
  std::vector<std::pair<double, double> > planWithSimpleSetup()
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
    simple_setup.setPlanner(ob::PlannerPtr(new og::RRT( simple_setup.getSpaceInformation() )));

    // Set state validity checking for this space
    simple_setup.setStateValidityChecker( ob::StateValidityCheckerPtr( new TwoDimensionalValidityChecker( simple_setup.getSpaceInformation(), cost_ ) ) );



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


    // Solve -----------------------------------------------------------
    std::cout << "\n\nSolve\n" << std::endl;

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

      // Get information about the exploration data structure the motion planner used.
      planner_data_ = new ob::PlannerData( simple_setup.getSpaceInformation() );
      simple_setup.getPlannerData( *planner_data_ );

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
  void displayCubes()
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
    for( int marker_id = 0; marker_id < image_->getSize(); ++marker_id )
    {
      marker.id = marker_id;

      // Set the color -- be sure to set alpha to something non-zero!
      marker.color.r = image_->data[ marker_id ].red / 255.0;
      marker.color.g = image_->data[ marker_id ].green / 255.0;
      marker.color.b = image_->data[ marker_id ].blue / 255.0;
      marker.color.a = 1.0;

      // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
      marker.pose.position.x = marker_id % image_->x;    // Map index back to coordinates
      marker.pose.position.y = marker_id / image_->x;    // Map index back to coordinates
      marker.pose.position.z = cost_.data()[ marker_id ] / 2;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;

      // Set the scale of the marker -- 1x1x1 here means 1m on a side
      marker.scale.x = 1.0;
      marker.scale.y = 1.0;
      marker.scale.z = cost_.data()[ marker_id ];

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
        << " COST: " << cost_[ marker.id ]
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
  // Helper Function to displayTriangles
  // *********************************************************************************************************
  void addPoint( int x, int y, visualization_msgs::Marker* marker )
  {
    // Point
    geometry_msgs::Point point;
    point.x = x;
    point.y = y;
    point.z = cost_( y, x ) / 2;
    marker->points.push_back( point );

    //  std::cout << "Point " << x << " " << y << " " << point.z << std::endl;

    // Color
    std_msgs::ColorRGBA color;
    color.r = image_->data[ image_->getID( x, y ) ].red / 255.0;
    color.g = image_->data[ image_->getID( x, y ) ].green / 255.0;
    color.b = image_->data[ image_->getID( x, y ) ].blue / 255.0;
    color.a = 1.0;
    marker->colors.push_back( color );
  }

  // *********************************************************************************************************
  // Visualize Results
  // *********************************************************************************************************
  void displayTriangles()
  {
    // ROS Publishing stuff
    ros::NodeHandle n;
    ros::Rate rate(40);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/my_frame";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";

    // Set the marker type.
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;

    // Set the marker action.  Options are ADD and DELETE
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = 1;

    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.r = 255;
    marker.color.g = 0;
    marker.color.b = 0;
    marker.color.a = 1.0;

    // Visualize Results -------------------------------------------------------------------------------------------------
    for( int marker_id = 0; marker_id < image_->getSize(); ++marker_id )
    {

      int x = marker_id % image_->x;    // Map index back to coordinates
      int y = marker_id / image_->x;    // Map index back to coordinates

      // Make right and down triangle
      // Check that we are not on the far right or bottom
      if( ! (x + 1 >= image_->x ||  y + 1 >= image_->y ) )
      {
        //std::cout << " ---- " << std::endl;

        addPoint( x,   y, &marker );
        addPoint( x+1, y, &marker );
        addPoint( x,   y+1, &marker );
      }

      // Make back and down triangle
      // Check that we are not on the far left or bottom
      if( ! (x - 1 < 0 ||  y + 1 >= image_->y ) )
      {
        //std::cout << " ---- " << std::endl;

        addPoint( x,   y, &marker );
        addPoint( x,   y+1, &marker );
        addPoint( x-1, y+1, &marker );
      }


      //marker.lifetime = ros::Duration(10.0);
      //marker.lifetime = ros::Duration();
      ros::Duration(0.000001).sleep();

    }

    // Publish the marker array
    marker_pub.publish( marker );

    ros::Duration(1).sleep();
  }

};

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
    return false;
  }
  image_path.append( "/resources/mountains.ppm" );
  //image_path.append( "/resources/height_map1.ppm" );


  // Run the program
  OmplRvizViewer viewer;
  viewer.runImage( image_path );

  return true;
}

