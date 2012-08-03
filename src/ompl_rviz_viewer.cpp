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
  ob::PlannerDataPtr planner_data_;

  /// Save the simple setup until the program ends so that the planner data is not lost
  og::SimpleSetupPtr simple_setup_;

  /// A shared ROS publisher
  ros::Publisher marker_pub_;

  /// A shared node handle
  ros::NodeHandle n_;

public:

  // *********************************************************************************************************
  // Constructor
  // *********************************************************************************************************
  OmplRvizViewer()
  {

    // ROS Publishing stuff
    marker_pub_ = n_.advertise<visualization_msgs::Marker>("visualization_marker", 0);


    /*
    ros::Duration(1).sleep();

    std::cout << "here " << std::endl;


    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/my_frame";
    marker.header.stamp = ros::Time();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "lines";

    // Set the marker type.
    //    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    //marker.type = visualization_msgs::Marker::SPHERE;

    // Set the marker action.  Options are ADD and DELETE
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = 0;

    std::cout << "here " << std::endl;
    marker.pose.position.x = 1.0;
    marker.pose.position.y = 1.0;
    marker.pose.position.z = 1.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;


    // Create first point
    geometry_msgs::Point this_point;
    this_point.x = 10.0;
    this_point.y = 10.0;
    this_point.z = 1.0;
    marker.points.push_back( this_point );

    // Color 1
    std_msgs::ColorRGBA color;
    color.r = 1.0;
    color.g = 0.0;
    color.b = 0.0;
    color.a = 1.0;
    marker.colors.push_back( color );

    // Create a second point
    geometry_msgs::Point next_point;
    next_point.x = 50.0;
    next_point.y = 50.0;
    next_point.z = 1.0;
    marker.points.push_back( next_point );

    // Color 2
    std_msgs::ColorRGBA color2;
    color2.r = 1.0;
    color2.g = 1.0;
    color2.b = 0.0;
    color2.a = 1.0;
    marker.colors.push_back( color2 );

    ros::Duration(1).sleep();
    std::cout << "here " << std::endl;
    // Publish the marker
    marker_pub_.publish( marker );

    ros::Duration(1).sleep();
    */
  }

  // *********************************************************************************************************
  // Deconstructor
  // *********************************************************************************************************
  ~OmplRvizViewer()
  {
    delete image_;
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
    displayGraph();

    // Done
    std::cout << "SUCCESS ------------------------------ " << std::endl << std::endl;
  }

  // *********************************************************************************************************
  // Private
  // *********************************************************************************************************
private:
  static const unsigned int DIMENSIONS = 2;


  // *********************************************************************************************************
  // Helper Function: gets the x,y coordinates for a given vertex id
  // *********************************************************************************************************
  std::pair<unsigned int, unsigned int> getCoordinates( int vertex_id )
  {
    ob::PlannerDataVertex vertex = planner_data_->getVertex( vertex_id );

    // Get this vertex's coordinates
    const ob::State *state = vertex.getState();

    if (!state)
    {
      ROS_ERROR("No state found for a vertex");
      exit(0);
    }

    // Convert to RealVectorStateSpace
    const ob::RealVectorStateSpace::StateType *real_state =
      static_cast<const ob::RealVectorStateSpace::StateType*>(state);

    //std::cout << "X: " << real_state->values[0] << std::endl;
    //std::cout <<" Y: " << real_state->values[1] << std::endl;

    return std::pair<unsigned int, unsigned int>( real_state->values[0], real_state->values[1] );
  }


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
    simple_setup_ = og::SimpleSetupPtr( new og::SimpleSetup(space) );

    // Set the setup planner (TRRT)
    simple_setup_->setPlanner(ob::PlannerPtr(new og::RRT( simple_setup_->getSpaceInformation() )));

    // Set state validity checking for this space
    simple_setup_->setStateValidityChecker( ob::StateValidityCheckerPtr( new TwoDimensionalValidityChecker( simple_setup_->getSpaceInformation(), cost_ ) ) );



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
    simple_setup_->setStartAndGoalStates(start, goal);


    // Debug -----------------------------------------------------------

    // this call is optional, but we put it in to get more output information
    simple_setup_->setup();
    simple_setup_->print();


    // Solve -----------------------------------------------------------
    std::cout << "\n\nSolve\n" << std::endl;

    // attempt to solve the problem within one second of planning time
    ob::PlannerStatus solved = simple_setup_->solve(1.0);

    if (solved)
    {
      // print the path to screen
      simple_setup_->simplifySolution();

      std::cout << "\nFOUND SOLUTION:" << std::endl;
      simple_setup_->getSolutionPath().interpolate();
      simple_setup_->getSolutionPath().print(std::cout);

      og::PathGeometric path = simple_setup_->getSolutionPath();
      std::vector<ob::State*> states = path.getStates();

      // Get information about the exploration data structure the motion planner used.
      planner_data_.reset( new ob::PlannerData( simple_setup_->getSpaceInformation() ) );
      simple_setup_->getPlannerData( *planner_data_ );

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
    marker_pub_.publish( marker_array );

    ros::Duration(1).sleep();
  }

  // *********************************************************************************************************
  // Helper Function to display triangles
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
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/my_frame";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "cost_map";

    // Set the marker type.
    //    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    marker.type = visualization_msgs::Marker::LINE_LIST;

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


      ros::Duration(0.000001).sleep();

      //      if( marker_id > 9502 )
      //        break;
      //      std::cout << marker_id << " ";
      //      break;
    }

    //    ros::Duration(0.000001).sleep();
    //    marker.lifetime = ros::Duration(4.0);

    // Publish the marker array
    marker_pub_.publish( marker );

    ros::Duration(1).sleep();
  }

  // *********************************************************************************************************
  // Display Explored Space
  // *********************************************************************************************************
  void displayGraph()
  {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/my_frame";
    marker.header.stamp = ros::Time();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "space_exploration";

    // Set the marker type.
    //    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    //    marker.type = visualization_msgs::Marker::CUBE_LIST;

    // Set the marker action.  Options are ADD and DELETE
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = 0;

    marker.pose.position.x = 1.0;
    marker.pose.position.y = 1.0;
    marker.pose.position.z = 1.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    /*
    // Create first point
    geometry_msgs::Point this_point;
    this_point.x = 10.0;
    this_point.y = 10.0;
    this_point.z = 1.0;
    marker.points.push_back( this_point );

    // Color 1
    std_msgs::ColorRGBA color;
    color.r = 1.0;
    color.g = 0.0;
    color.b = 0.0;
    color.a = 1.0;
    marker.colors.push_back( color );

    // Create a second point
    geometry_msgs::Point next_point;
    next_point.x = 50.0;
    next_point.y = 50.0;
    next_point.z = 1.0;
    marker.points.push_back( next_point );

    // Color 2
    std_msgs::ColorRGBA color2;
    color2.r = 1.0;
    color2.g = 1.0;
    color2.b = 0.0;
    color2.a = 1.0;
    marker.colors.push_back( color2 );

    ros::Duration(1).sleep();

    // Publish the marker
    marker_pub_.publish( marker );

    ros::Duration(1).sleep();
    */

    std::pair<unsigned int, unsigned int> this_vertex;
    std::pair<unsigned int, unsigned int> next_vertex;

    // Loop through all verticies
    for( int vertex_id = 0; vertex_id < int( planner_data_->numVertices() ); ++vertex_id )
    {

      this_vertex = getCoordinates( vertex_id );

      // Get the out edges from the current vertex
      std::vector<unsigned int> edge_list;
      planner_data_->getEdges( vertex_id, edge_list );

      // Now loop through each edge
      for( std::vector<unsigned int>::const_iterator edge_it = edge_list.begin();
           edge_it != edge_list.end(); ++edge_it)
      {
        // Convert vertex id to next coordinates
        next_vertex = getCoordinates( *edge_it );

        // Create first point
        geometry_msgs::Point this_point;
        this_point.x = this_vertex.first;
        this_point.y = this_vertex.second;
        this_point.z = 10;

        // Create a second point
        geometry_msgs::Point next_point;
        next_point.x = next_vertex.first;
        next_point.y = next_vertex.second;
        next_point.z = 10;

        std::cout << "FROM: (" << this_point.x << "," << this_point.y << ") TO (";
        std::cout << next_point.x << "," << next_point.y << ")" << std::endl;

        // Add the point pair to the line message
        marker.points.push_back( this_point );

        // Color 1
        std_msgs::ColorRGBA color;
        color.r = 1.0;
        color.g = 0;
        color.b = 0;
        color.a = 1.0;
        marker.colors.push_back( color );

        marker.points.push_back( next_point );

        // Color 2
        std_msgs::ColorRGBA color2;
        color2.r = 1.0;
        color2.g = 0.0;
        color2.b = 0.0;
        color2.a = 1.0;
        marker.colors.push_back( color2 );

        ros::Duration(0.00001).sleep();
      }
    }

    // Publish the marker
    marker_pub_.publish( marker );

    ros::Duration(1).sleep();


    /*
      visualization_msgs::Marker marker;
      // Set the frame ID and timestamp.  See the TF tutorials for information on these.
      marker.header.frame_id = "/my_frame";
      marker.header.stamp = ros::Time::now();

      // Set the namespace and id for this marker.  This serves to create a unique ID
      // Any marker sent with the same namespace and id will overwrite the old one
      marker.ns = "basic_shapes";

      // Set the marker type.
      marker.type = visualization_msgs::Marker::LINE_LIST;

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

      // Get the starting vertex
      std::cout << "Verticies: " << planner_data_->numVertices() << std::endl;
      std::cout << "Start S: " << planner_data_->numStartVertices() << std::endl;

      std::pair<unsigned int, unsigned int> this_vertex;
      std::pair<unsigned int, unsigned int> next_vertex;

      // Loop through all verticies
      for( int vertex_id = 0; vertex_id < int( planner_data_->numVertices() ); ++vertex_id )
      {

      this_vertex = getCoordinates( vertex_id );

      // Create first point
      geometry_msgs::Point this_point;
      this_point.x = this_vertex.first;
      this_point.y = this_vertex.second;
      this_point.z = 10;

      // Get the out edges from the current vertex
      std::vector<unsigned int> edge_list;
      planner_data_->getEdges( vertex_id, edge_list );

      // Now loop through each edge
      for( std::vector<unsigned int>::const_iterator edge_it = edge_list.begin();
      edge_it != edge_list.end(); ++edge_it)
      {
      // Convert vertex id to next coordinates
      next_vertex = getCoordinates( *edge_it );

      // Create a second point
      geometry_msgs::Point next_point;
      next_point.x = next_vertex.first;
      next_point.y = next_vertex.second;
      next_point.z = 10;

      std::cout << "FROM: (" << this_point.x << "," << this_point.y << ") TO (";
      std::cout << next_point.x << "," << next_point.y << ")" << std::endl;


      // Add the point pair to the line message
      marker.points.push_back( this_point );
      marker.points.push_back( next_point );

      // Color 1
      std_msgs::ColorRGBA color;
      color.r = 255;
      color.g = 0;
      color.b = 0;
      color.a = 1.0;
      marker.colors.push_back( color );

      // Color 2
      std_msgs::ColorRGBA color2;
      color2.r = 0;
      color2.g = 100;
      color2.b = 255;
      color2.a = 1.0;
      marker.colors.push_back( color2 );

      ros::Duration(0.00001).sleep();
      }
      }

      // Publish the marker array
      marker_pub_.publish( marker );

      ros::Duration(1).sleep();
    */
  }

}; // end of class

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

