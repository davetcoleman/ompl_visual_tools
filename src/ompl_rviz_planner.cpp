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
// This package
#include "utilities/ppm.h" // for reading image files
// OMPL
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/config.h>
#include <iostream>
// Boost
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace bnu = boost::numeric::ublas;

namespace ompl_rviz_viewer
{

static const std::string BASE_FRAME = "/world";

// *********************************************************************************************************
// Nat_Rounding helper function to make readings from cost map more accurate
// *********************************************************************************************************
int nat_round(double x)
{
  return static_cast<int>(floor(x + 0.5f));
}

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
  TwoDimensionalValidityChecker( const ob::SpaceInformationPtr& si, const bnu::matrix<int>& cost, 
				 double max_threshold ) :
    StateValidityChecker(si)
  {
    cost_ = cost;
    max_threshold_ = max_threshold;
  }

  /** \brief Obstacle checker */
  virtual bool isValid(const ob::State * state ) const
  {
    return cost(state) < max_threshold_;
  }

  virtual double cost(const ob::State *state) const
  {
    const double *coords = state->as<ob::RealVectorStateSpace::StateType>()->values;

    // Return the cost from the matrix at the current dimensions
    double cost = cost_( nat_round(coords[1]), nat_round(coords[0]) );

    return cost;
  }

};

// *********************************************************************************************************
// *********************************************************************************************************
// Rviz Visualizer Class
// *********************************************************************************************************
// *********************************************************************************************************
class OmplRvizPlanner
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

  /// Remember the min and max cost from the cost_ matrix for color visualization
  int max_cost_;
  int min_cost_;

  /// The cost at which it becomes an obstacle
  double max_threshold_;

  // The percentage of the top min/max cost value that is considered an obstacle, e.g. 10 is top 10% of peaks
  //  static const double MAX_THRESHOLD_PERCENTAGE_ = 40;
  static const double MAX_THRESHOLD_PERCENTAGE_ = 1;

  // The number of dimensions - always 2 for images
  static const unsigned int DIMENSIONS = 2;

  // Use random goal and start locations
  static const bool USE_RANDOM_STATES = true;

public:

  // *********************************************************************************************************
  // Constructor
  // *********************************************************************************************************
  OmplRvizPlanner()
  {
    image_ = NULL;

    // ROS Publishing stuff
    marker_pub_ = n_.advertise<visualization_msgs::Marker>("ompl_rviz_markers", 1);
    ros::Duration(1).sleep();
  }

  // *********************************************************************************************************
  // Deconstructor
  // *********************************************************************************************************
  ~OmplRvizPlanner()
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

    // Disallow non-square
    if( image_->x != image_->y )
    {
      ROS_ERROR( "Does not currently support non-square images because of some weird bug. Feel free to fork and fix!" );
      return;
    }

    ROS_INFO_STREAM( "Map Height: " << image_->y << " Map Width: " << image_->x );

    // Create an array of ints that represent the cost of every pixel
    cost_.resize( image_->x, image_->y );

    // gets the min and max values of the cost map
    getMinMaxCost();

    // Generate the cost map
    createCostMap();

    ROS_INFO_STREAM( "OMPL version: " << OMPL_VERSION );

    // OMPL Processing -------------------------------------------------------------------------------------------------
    // Run OMPL and display
    if( planWithSimpleSetup() )
    {
      // Setup
      std::vector<std::pair<double, double> > coordinates;

      // Make line color
      std_msgs::ColorRGBA color;
      color.a = 1.0;

      // Render the map ---------------------------------------------------
      displayTriangles();
      ros::Duration(0.1).sleep();

      // Visualize the explored space ---------------------------------------
      displayGraph();
      ros::Duration(0.1).sleep();

      // Visualize the sample locations -----------------------------------
      displaySamples();
      ros::Duration(0.1).sleep();

      // Show basic solution ----------------------------------------
      // Get solution coordinates
      if( false )
      {
        coordinates = convertSolutionToVector();

        // Visualize the chosen path
        color.r = 1.0;
        color.g = 0.0;
        color.b = 0.0;
        displayResult( coordinates, &color );
        ros::Duration(0.25).sleep();
      }

      // Interpolate -------------------------------------------------------
      if( true )
      {
        simple_setup_->getSolutionPath().interpolate();

        // Get solutippon coordinates
        coordinates = convertSolutionToVector();      // Get basic solution

        // Visualize the chosen path
        color.r = 0.0;
        color.g = 1.0;
        color.b = 0.0;
        displayResult( coordinates, &color );
        //      ros::Duration(0.25).sleep();
      }

      // Simplify solution ------------------------------------------------------
      if( false )
      {
        simple_setup_->simplifySolution();

        // Get solution coordinates
        coordinates = convertSolutionToVector();      // Get basic solution

        // Visualize the chosen path
        color.r = 0.0;
        color.g = 0.5;
        color.b = 0.5;
        displayResult( coordinates, &color );
        ros::Duration(0.25).sleep();
      }
    }
    else // just show the map
    {
      // Render the map
      displayTriangles();
      ros::Duration(0.25).sleep();
    }

  }

  // *********************************************************************************************************
  // Private
  // *********************************************************************************************************
private:

  // *********************************************************************************************************
  // Helper Function: calculate cost map
  // *********************************************************************************************************
  void createCostMap()
  {
    // This factor is the author's visual preference for scaling a cost map in Rviz
    const double artistic_scale = 2.0;

    // This scale adapts that factor depending on the cost map min max
    const double scale = (max_cost_ - min_cost_ ) / ( image_->x / artistic_scale );

    // Dynamically calculate the obstacle threshold
    max_threshold_ = max_cost_ - ( MAX_THRESHOLD_PERCENTAGE_ / 100 * (max_cost_ - min_cost_) );

    // Preprocess the pixel data for cost and give it a nice colored tint
    for( size_t i = 0; i < image_->getSize(); ++i )
    {
      // Calculate cost
      cost_.data()[i]  = ( image_->data[ i ].red ) / scale;

      // Prevent cost from being zero
      if( !cost_.data()[i] )
        cost_.data()[i] = 1;

      // Color different if it is an obstacle
      if( cost_.data()[i] > max_threshold_ )
      {
        image_->data[ i ].red = 255; //image_->data[ i ].red;
        image_->data[ i ].green = image_->data[ i ].green;
        image_->data[ i ].blue = image_->data[ i ].blue;
      }

    }




  }

  // *********************************************************************************************************
  // Helper Function: gets the min and max values of the cost map
  // *********************************************************************************************************
  void getMinMaxCost()
  {
    // Find the min and max cost from the image
    min_cost_ = image_->data[ 0 ].red;
    max_cost_ = image_->data[ 0 ].red;

    for( size_t i = 0; i < image_->getSize(); ++i )
    {
      // Max
      if( image_->data[ i ].red > max_cost_ )
        max_cost_ = image_->data[ i ].red;
      // Min
      else if( image_->data[ i ].red < min_cost_ )
        min_cost_ = image_->data[ i ].red;
    }
  }

  // *********************************************************************************************************
  // Helper Function: gets the x,y coordinates for a given vertex id
  // *********************************************************************************************************
  std::pair<double, double> getCoordinates( int vertex_id )
  {
    ob::PlannerDataVertex vertex = planner_data_->getVertex( vertex_id );

    // Get this vertex's coordinates
    const ob::State *state = vertex.getState();

    if (!state)
    {
      ROS_ERROR("No state found for a vertex");
      exit(1);
    }

    // Convert to RealVectorStateSpace
    const ob::RealVectorStateSpace::StateType *real_state =
      static_cast<const ob::RealVectorStateSpace::StateType*>(state);

    return std::pair<double, double>( real_state->values[0], real_state->values[1] );
  }

  // *********************************************************************************************************
  // After running the algorithm, this converts the results to a vector of coordinates
  // *********************************************************************************************************
  std::vector<std::pair<double, double> > convertSolutionToVector()
  {
    // The returned result
    std::vector<std::pair<double, double> > coordinates;

    // Get data
    og::PathGeometric path = simple_setup_->getSolutionPath();
    const std::vector<ob::State*>& states = path.getStates();

    // Convert solution to coordinate vector
    //for( std::vector<ob::State*>::const_iterator state_it = states.begin();
    //         state_it != states.end(); ++state_it )
    for( size_t state_id = 0; state_id < states.size(); ++state_id )
    {
      const ob::State *state = states[ state_id ];

      if (!state)
        continue; // no data?

      // Convert to RealVectorStateSpace
      const ob::RealVectorStateSpace::StateType *real_state =
        static_cast<const ob::RealVectorStateSpace::StateType*>(state);

      // Add to vector of results
      coordinates.push_back( std::pair<double,double>( real_state->values[0], real_state->values[1] ) );
    }

    return coordinates;
  }

  // *********************************************************************************************************
  // Plan
  // *********************************************************************************************************
  bool planWithSimpleSetup()
  {
    // Construct the state space we are planning in
    ob::StateSpacePtr space( new ob::RealVectorStateSpace( DIMENSIONS ));

    // Set the bounds for the R^2
    ob::RealVectorBounds bounds( DIMENSIONS );
    bounds.setLow( 0 ); // both dimensions start at 0
    //    bounds.setHigh( 99 ); // both dimensions start at 0
    bounds.setHigh( 0, image_->x - 1 ); // allow for non-square images
    bounds.setHigh( 1, image_->y - 1 ); // allow for non-square images
    space->as<ob::RealVectorStateSpace>()->setBounds( bounds );

    // Define a simple setup class ---------------------------------------
    simple_setup_ = og::SimpleSetupPtr( new og::SimpleSetup(space) );

    // Set the setup planner (TRRT)
    og::TRRT *trrt = new og::TRRT( simple_setup_->getSpaceInformation() );
    //og::RRT *trrt = new og::RRT( simple_setup_->getSpaceInformation() );

    simple_setup_->setPlanner(ob::PlannerPtr(trrt));

    // Set state validity checking for this space
    simple_setup_->setStateValidityChecker( ob::StateValidityCheckerPtr( new TwoDimensionalValidityChecker( simple_setup_->getSpaceInformation(), cost_, max_threshold_ ) ) );


    // Start and Goal State ---------------------------------------------

    // Create start space
    ob::ScopedState<> start(space);
    if( USE_RANDOM_STATES )
    {
      start.random();
    }
    else // Manually set the start location
    {
      start[0] = 95;
      start[1] = 10;
    }

    // create a goal state
    ob::ScopedState<> goal(space);
    if( USE_RANDOM_STATES )
    {
      goal.random();
    }
    else // Manually set the start location
    {
      goal[0] = 40;
      goal[1] = 300;
    }

    // Modify the map to show start and end locations
    image_->data[ image_->getID( start[0], start[1] ) ].red = 50;
    image_->data[ image_->getID( start[0], start[1] ) ].blue = 50;
    image_->data[ image_->getID( start[0], start[1] ) ].green = 255;

    image_->data[ image_->getID( goal[0], goal[1] ) ].red = 255;
    image_->data[ image_->getID( goal[0], goal[1] ) ].blue = 255;
    image_->data[ image_->getID( goal[0], goal[1] ) ].green = 10;

    // set the start and goal states
    simple_setup_->setStartAndGoalStates(start, goal);

    // Setup -----------------------------------------------------------

    // Auto setup parameters
    simple_setup_->setup();
    // The interval in which obstacles are checked for between states
    //    simple_setup_->getSpaceInformation()->setStateValidityCheckingResolution(0.005);

    // Debug -----------------------------------------------------------
    // this call is optional, but we put it in to get more output information
    //simple_setup_->print();


    // Solve -----------------------------------------------------------
    ROS_INFO( "Starting OMPL motion planner..." );

    // attempt to solve the problem within x seconds of planning time
    ob::PlannerStatus solved = simple_setup_->solve( 10.0 );

    if (solved)
    {
      ROS_INFO("Solution Found");

      // Get information about the exploration data structure the motion planner used. Used later in visualizing
      planner_data_.reset( new ob::PlannerData( simple_setup_->getSpaceInformation() ) );
      simple_setup_->getPlannerData( *planner_data_ );
    }
    else
    {
      ROS_INFO("No Solution Found");
    }

    return solved;
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
    marker.header.frame_id = BASE_FRAME;
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    marker.ns = "cost_map";

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
    for( size_t marker_id = 0; marker_id < image_->getSize(); ++marker_id )
    {

      unsigned int x = marker_id % image_->x;    // Map index back to coordinates
      unsigned int y = marker_id / image_->x;    // Map index back to coordinates

      // Make right and down triangle
      // Check that we are not on the far right or bottom
      if( ! (x + 1 >= image_->x ||  y + 1 >= image_->y ) )
      {
        addPoint( x,   y, &marker );
        addPoint( x+1, y, &marker );
        addPoint( x,   y+1, &marker );
      }

      // Make back and down triangle
      // Check that we are not on the far left or bottom
      if( ! ( int(x) - 1 < 0 ||  y + 1 >= image_->y ) )
      {
        addPoint( x,   y, &marker );
        addPoint( x,   y+1, &marker );
        addPoint( x-1, y+1, &marker );
      }

    }

    marker_pub_.publish( marker );
  }

  // *********************************************************************************************************
  // Helper Function for Display Graph that makes the exploration lines follow the curvature of the map
  // *********************************************************************************************************
  void interpolateLine( double x1, double y1, double x2, double y2, visualization_msgs::Marker* marker, std_msgs::ColorRGBA* color )
  {
    // Switch the coordinates such that x1 < x2
    if( x1 > x2 )
    {
      // Swap the coordinates
      double x_temp = x1;
      double y_temp = y1;
      x1 = x2;
      y1 = y2;
      x2 = x_temp;
      y2 = y_temp;
    }

    // Points
    geometry_msgs::Point point_a;
    geometry_msgs::Point point_b;

    // Show the straight line --------------------------------------------------------------------
    if( false )
    {
      // First point
      point_a.x = x1;
      point_a.y = y1;
      point_a.z = 20.0;

      // Create a second point
      point_b.x = x2;
      point_b.y = y2;
      point_b.z = 20.0;

      // Change Color
      color->g = 0.8;

      // Add the point pair to the line message
      marker->points.push_back( point_a );
      marker->points.push_back( point_b );
      marker->colors.push_back( *color );
      marker->colors.push_back( *color );
    }


    // Interpolate the line ----------------------------------------------------------------------
    color->g = 0.0;

    // Calculate slope between the lines
    double m = (y2 - y1)/(x2 - x1);

    // Calculate the y-intercep
    double b = y1 - m * x1;

    // Define the interpolation interval
    double interval = 0.5;

    // Remember the previous points
    double x_a = x1;
    double y_a = y1;
    double x_b;
    double y_b;

    // Loop through the line adding segements along the cost map
    for( x_b = x1 + interval; x_b < x2; x_b += interval )
    {
      // Find the y coordinate
      y_b = m*x_b + b;

      // Create first point
      point_a.x = float(x_a);
      point_a.y = float(y_a);
      point_a.z = cost_( nat_round(point_a.y), nat_round(point_a.x) ) / 2 + 2;

      // Create a second point
      point_b.x = float(x_b);
      point_b.y = float(y_b);
      point_b.z = cost_( nat_round(point_b.y), nat_round(point_b.x) ) / 2 + 2;

      // Add the point pair to the line message
      marker->points.push_back( point_a );
      marker->points.push_back( point_b );

      // Add colors
      marker->colors.push_back( *color );
      marker->colors.push_back( *color );

      // Remember the last coordiante for next iteration
      x_a = x_b;
      y_a = y_b;
    }

    // Finish the line for non-even interval lengths

    // Create first point
    point_a.x = float(x_a);
    point_a.y = float(y_a);
    point_a.z = cost_( nat_round(point_a.y), nat_round(point_a.x) ) / 2 + 2;

    // Create a second point
    point_b.x = float(x2);
    point_b.y = float(y2);
    point_b.z = cost_( nat_round(point_b.y), nat_round(point_b.x) ) / 2 + 2;

    // Add the point pair to the line message
    marker->points.push_back( point_a );
    marker->points.push_back( point_b );

    // Add colors
    marker->colors.push_back( *color );
    marker->colors.push_back( *color );

  }

  // *********************************************************************************************************
  // Display Explored Space
  // *********************************************************************************************************
  void displayGraph()
  {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = BASE_FRAME;
    marker.header.stamp = ros::Time();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    marker.ns = "space_exploration";

    // Set the marker type.
    marker.type = visualization_msgs::Marker::LINE_LIST;

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

    marker.scale.x = 0.2;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    std::pair<double, double> this_vertex;
    std::pair<double, double> next_vertex;

    // Make line color
    std_msgs::ColorRGBA color;
    color.r = 0.0;
    color.g = 0.0;
    color.b = 1.0;
    color.a = 1.0;

    ROS_INFO("Publishing Graph");

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

        interpolateLine( this_vertex.first, this_vertex.second, next_vertex.first, next_vertex.second, &marker, &color );
      }

    }

    ROS_INFO_STREAM_NAMED("temp","marker:\b" << marker);
    ros::Duration(5.0).sleep();

    // Publish the marker
    marker_pub_.publish( marker );
  }

  // *********************************************************************************************************
  // Display Sample Points
  // *********************************************************************************************************
  void displaySamples()
  {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.
    marker.header.frame_id = BASE_FRAME;
    marker.header.stamp = ros::Time();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    marker.ns = "sample_locations";

    // Set the marker type.
    marker.type = visualization_msgs::Marker::SPHERE_LIST;

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

    marker.scale.x = 0.4;
    marker.scale.y = 0.4;
    marker.scale.z = 0.4;

    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    std::pair<double, double> this_vertex;

    // Make line color
    std_msgs::ColorRGBA color;
    color.r = 0.8;
    color.g = 0.1;
    color.b = 0.1;
    color.a = 1.0;

    // Point
    geometry_msgs::Point point_a;

    ROS_INFO("Publishing Spheres");

    // Loop through all verticies
    for( int vertex_id = 0; vertex_id < int( planner_data_->numVertices() ); ++vertex_id )
    {

      this_vertex = getCoordinates( vertex_id );

      // First point
      point_a.x = this_vertex.first;
      point_a.y = this_vertex.second;
      point_a.z = cost_( nat_round(point_a.y), nat_round(point_a.x) ) / 2 + 2;

      // Add the point pair to the line message
      marker.points.push_back( point_a );
      marker.colors.push_back( color );
    }

    ROS_INFO_STREAM_NAMED("temp","marker:\b" << marker);
    ros::Duration(5.0).sleep();

    // Publish the marker
    marker_pub_.publish( marker );
  }

  // *********************************************************************************************************
  // Display Result Path
  // *********************************************************************************************************
  void displayResult( const std::vector<std::pair<double, double> > coordinates, std_msgs::ColorRGBA* color )
  {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.
    marker.header.frame_id = BASE_FRAME;
    marker.header.stamp = ros::Time();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    marker.ns = "result_path";

    // Set the marker type.
    marker.type = visualization_msgs::Marker::LINE_LIST;

    // Set the marker action.  Options are ADD and DELETE
    marker.action = visualization_msgs::Marker::ADD;

    // Provide a new id every call to this function
    static int result_id = 0;
    marker.id = result_id++;

    marker.pose.position.x = 1.0;
    marker.pose.position.y = 1.0;
    marker.pose.position.z = 1.0;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.4;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    ROS_INFO("Publishing result");

    // Get the initial points
    double x1 = coordinates[0].first;
    double y1 = coordinates[0].second;
    // Declare the second points
    double x2;
    double y2;

    // Convert path coordinates to red line
    for( unsigned int i = 1; i < coordinates.size(); ++i )
    {
      x2 = coordinates[i].first;
      y2 = coordinates[i].second;

      // Points
      geometry_msgs::Point point_a;
      geometry_msgs::Point point_b;

      // First point
      point_a.x = x1;
      point_a.y = y1;
      point_a.z = cost_( nat_round(point_a.y), nat_round(point_a.x) ) / 2 + 3;

      // Create a second point
      point_b.x = x2;
      point_b.y = y2;
      point_b.z = cost_( nat_round(point_b.y), nat_round(point_b.x) ) / 2 + 3;

      // Add the point pair to the line message
      marker.points.push_back( point_a );
      marker.points.push_back( point_b );
      marker.colors.push_back( *color );
      marker.colors.push_back( *color );

      // Save these coordinates for next line
      x1 = x2;
      y1 = y2;
    }

    ROS_INFO_STREAM_NAMED("temp","marker:\b" << marker);
    ros::Duration(5.0).sleep();

    // Publish the marker
    marker_pub_.publish( marker );
  }

}; // end of class

} // namespace


// *********************************************************************************************************
// Main
// *********************************************************************************************************
int main( int argc, char** argv )
{
  ros::init(argc, argv, "ompl_rviz_viewer");
  ROS_INFO( "OMPL RViz Viewer ----------------------------------------- " );

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(1);
  spinner.start();

  std::string image_path;

  // Check if user has passed in an image to read
  if( argc > 1 )
  {
    image_path = argv[1];
  }
  else // use default image
  {
    // Get image path based on package name
    image_path = ros::package::getPath("ompl_rviz_viewer");
    if( image_path.empty() )
    {
      ROS_ERROR( "Unable to get OMPL RViz Viewer package path " );
      return false;
    }

    // Seed random
    srand ( time(NULL) );

    // Choose random image
    switch( 1 ) //rand() % 4 )
    {
    case 0:
      image_path.append( "/resources/grand_canyon.ppm" );
      break;
    case 1:
      image_path.append( "/resources/height_map0.ppm" );
      break;
    case 2:
      image_path.append( "/resources/height_map1.ppm" );
      break;
    case 3:
      image_path.append( "/resources/height_map2.ppm" );
      break;
    }
  }

  // Run the program
  ompl_rviz_viewer::OmplRvizPlanner planner;
  ROS_INFO_STREAM_NAMED("main","Loading image " << image_path);
  planner.runImage( image_path );

  // Wait to let anything still being published finish
  ros::Duration(1).sleep();

  ROS_INFO_STREAM("Shutting down.");

  return 0;
}


