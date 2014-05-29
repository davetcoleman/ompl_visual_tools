/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, University of Colorado, Boulder
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
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
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

/* Author: Dave Coleman <dave@dav.ee>
   Desc:   Tools for displaying OMPL components in Rviz
*/

// ROS
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

// Boost
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>

// OMPL
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/PlannerData.h>
#include <ompl/config.h>

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

// Helper function for converting a point to the correct cost
double getCost(const geometry_msgs::Point &point, bnu::matrix<int> &cost)
{
  return double(cost( nat_round(point.y), nat_round(point.x) )) / 2.0;
}

double getCostHeight(const geometry_msgs::Point &point, bnu::matrix<int> &cost)
{
  //std::cout << "------------------------" << std::endl;

  // check if whole number
  if (floor(point.x) == point.x && floor(point.y) == point.y)
    return getCost(point, cost);

  // else do Bilinear Interpolation
  // From http://supercomputingblog.com/graphics/coding-bilinear-interpolation/

  // top left
  geometry_msgs::Point a;
  a.x = floor(point.x);
  a.y = ceil(point.y);
  a.z = getCost(a, cost);

  // bottom left
  geometry_msgs::Point b;
  b.x = floor(point.x);
  b.y = floor(point.y);
  b.z = getCost(b, cost);

  // bottom right
  geometry_msgs::Point c;
  c.x = ceil(point.x);
  c.y = floor(point.y);
  c.z = getCost(c, cost);

  // top right
  geometry_msgs::Point d;
  d.x = ceil(point.x);
  d.y = ceil(point.y);
  d.z = getCost(d, cost);


  //std::cout << "a: " << a << std::endl;
  //std::cout << "b: " << b << std::endl;
  //std::cout << "c: " << c << std::endl;
  //std::cout << "d: " << d << std::endl;

  double R1 = ((c.x - point.x)/(c.x - b.x))*b.z + ((point.x - b.x)/(c.x - b.x))*c.z;
  double R2 = ((c.x - point.x)/(c.x - b.x))*a.z + ((point.x - b.x)/(c.x - b.x))*d.z;

  //std::cout << "R1: " << R1 << std::endl;
  //std::cout << "R2: " << R2 << std::endl;

  // After the two R values are calculated, the value of P can finally be calculated by a weighted average of R1 and R2.  
  double val;

  if ( a.y - b.y == 0) // division by zero
    val = R1;
  else
    val = ((a.y - point.y)/(a.y - b.y))*R1 + ((point.y - b.y)/(a.y - b.y))*R2;

  //std::cout << "val: " << val << std::endl;
  return val + 0.2;
}


class OmplRvizViewer
{
private:

  // A shared node handle
  ros::NodeHandle nh_;

  // Show more visual and console output, with general slower run time.
  bool verbose_;

  // A shared ROS publisher
  ros::Publisher marker_pub_;

public:

  /**
   * \brief Constructor
   * \param verbose - run in debug mode
   */
  OmplRvizViewer(bool verbose)
    : verbose_(verbose)
  {
    // ROS Publishing stuff
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("ompl_rviz_markers", 1);
    ros::Duration(1).sleep();

    ROS_INFO_STREAM_NAMED("ompl_rviz_viewer","OmplRvizViewer Ready.");
  }

  /**
   * \brief Destructor
   */
  ~OmplRvizViewer()
  {

  }

  /**
   * \brief After running the algorithm, this converts the results to a vector of coordinates
   */
  std::vector<std::pair<double, double> > convertSolutionToVector(og::PathGeometric& path)
  {
    // The returned result
    std::vector<std::pair<double, double> > coordinates;

    // Get data
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

  /**
   * \brief Visualize Results
   */
  void displayTriangles(PPMImage *image, bnu::matrix<int> &cost)
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
    for( size_t marker_id = 0; marker_id < image->getSize(); ++marker_id )
    {

      unsigned int x = marker_id % image->x;    // Map index back to coordinates
      unsigned int y = marker_id / image->x;    // Map index back to coordinates

      // Make right and down triangle
      // Check that we are not on the far right or bottom
      if( ! (x + 1 >= image->x ||  y + 1 >= image->y ) )
      {
        addPoint( x,   y, &marker, image, cost );
        addPoint( x+1, y, &marker, image, cost );
        addPoint( x,   y+1, &marker, image, cost );
      }

      // Make back and down triangle
      // Check that we are not on the far left or bottom
      if( ! ( int(x) - 1 < 0 ||  y + 1 >= image->y ) )
      {
        addPoint( x,   y, &marker, image, cost );
        addPoint( x,   y+1, &marker, image, cost );
        addPoint( x-1, y+1, &marker, image, cost );
      }

    }

    marker_pub_.publish( marker );
  }

  // *********************************************************************************************************
  // Helper Function to display triangles
  // *********************************************************************************************************
  void addPoint( int x, int y, visualization_msgs::Marker* marker, PPMImage *image, bnu::matrix<int> &cost )
  {
    // Point
    geometry_msgs::Point point;
    point.x = x;
    point.y = y;
    point.z = getCost(point, cost); // to speed things up, we know is always whole number
    marker->points.push_back( point );

    // Color
    std_msgs::ColorRGBA color;
    color.r = image->data[ image->getID( x, y ) ].red / 255.0;
    color.g = image->data[ image->getID( x, y ) ].green / 255.0;
    color.b = image->data[ image->getID( x, y ) ].blue / 255.0;
    color.a = 1.0;
    marker->colors.push_back( color );
  }

  // *********************************************************************************************************
  // Helper Function for Display Graph that makes the exploration lines follow the curvature of the map
  // *********************************************************************************************************
  void interpolateLine( const geometry_msgs::Point &p1, const geometry_msgs::Point &p2, visualization_msgs::Marker* marker, 
    std_msgs::ColorRGBA &color, bnu::matrix<int> &cost )
  {
    // Copy to non-const
    geometry_msgs::Point point_a = p1;
    geometry_msgs::Point point_b = p2;

    // Get the heights
    point_a.z = getCostHeight(point_a, cost);
    point_b.z = getCostHeight(point_b, cost);

    ROS_INFO_STREAM("a is: " << point_a);
    ROS_INFO_STREAM("b is: " << point_b);

    // Switch the coordinates such that x1 < x2
    if( point_a.x > point_b.x )
    {
      // Swap the coordinates      
      geometry_msgs::Point point_temp = point_a;      
      point_a = point_b;
      point_b = point_temp;
    }

    // temp
    std_msgs::ColorRGBA color2 = color;
    color2.r = 1;

    // Show the straight line --------------------------------------------------------------------
    if( false )
    {
      // Change Color
      color.g = 0.8;

      // Add the point pair to the line message
      marker->points.push_back( point_a );
      marker->points.push_back( point_b );
      marker->colors.push_back( color );
      marker->colors.push_back( color );

      // Show start and end point
      publishSphere(point_a, color2);
      publishSphere(point_b, color2);
    }

    // Interpolate the line ----------------------------------------------------------------------
    color.g = 0.0;

    // Calculate slope between the lines
    double m = (point_b.y - point_a.y)/(point_b.x - point_a.x);

    // Calculate the y-intercep
    double b = point_a.y - m * point_a.x;

    // Define the interpolation interval
    double interval = 0.25; //0.5;

    // Make new locations
    geometry_msgs::Point temp_a = point_a; // remember the last point
    geometry_msgs::Point temp_b = point_a; // move along this point

    // Loop through the line adding segements along the cost map
    for( temp_b.x = point_a.x + interval; temp_b.x < point_b.x; temp_b.x += interval )
    {
      publishSphere(temp_a, color2);

      // Find the y coordinate
      temp_b.y = m*temp_b.x + b;

      // Add the new heights
      temp_a.z = getCostHeight(temp_a, cost);
      temp_b.z = getCostHeight(temp_b, cost);

      // Add the point pair to the line message
      marker->points.push_back( temp_a );
      marker->points.push_back( temp_b );

      // Add colors
      marker->colors.push_back( color );
      marker->colors.push_back( color );

      // Remember the last coordiante for next iteration
      temp_a = temp_b;
    }

    // Finish the line for non-even interval lengths
    marker->points.push_back( temp_a );
    marker->points.push_back( point_b );

    // Add colors
    marker->colors.push_back( color );
    marker->colors.push_back( color );

  }

  // *********************************************************************************************************
  // Display Explored Space
  // *********************************************************************************************************
  void displayGraph(bnu::matrix<int> &cost, ob::PlannerDataPtr planner_data)
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

    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;

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


    // TEMP
    geometry_msgs::Point p1;
    p1.x = 0;
    p1.y = 0;
    geometry_msgs::Point p2;
    p2.x = 6;
    p2.y = 0;
    interpolateLine( p1, p2, &marker, color, cost );

    /*
    // Loop through all verticies
    for( int vertex_id = 0; vertex_id < int( planner_data->numVertices() ); ++vertex_id )
    {

      this_vertex = getCoordinates( vertex_id, planner_data );

      // Get the out edges from the current vertex
      std::vector<unsigned int> edge_list;
      planner_data->getEdges( vertex_id, edge_list );

      // Now loop through each edge
      for( std::vector<unsigned int>::const_iterator edge_it = edge_list.begin();
           edge_it != edge_list.end(); ++edge_it)
      {
        // Convert vertex id to next coordinates
        next_vertex = getCoordinates( *edge_it, planner_data );

        interpolateLine( this_vertex.first, this_vertex.second, next_vertex.first, next_vertex.second, &marker, &color, cost );
      }

    }
    */

    // Publish the marker
    marker_pub_.publish( marker );
  }

  // *********************************************************************************************************
  // Display Sample Points
  // *********************************************************************************************************
  void displaySamples(bnu::matrix<int> &cost, ob::PlannerDataPtr planner_data)
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
    for( int vertex_id = 0; vertex_id < int( planner_data->numVertices() ); ++vertex_id )
    {

      this_vertex = getCoordinates( vertex_id, planner_data );

      // First point
      point_a.x = this_vertex.first;
      point_a.y = this_vertex.second;
      point_a.z = getCostHeight(point_a, cost); 

      // Add the point pair to the line message
      marker.points.push_back( point_a );
      marker.colors.push_back( color );
    }

    // Publish the marker
    marker_pub_.publish( marker );
  }

  void publishSphere(const geometry_msgs::Point &point, const std_msgs::ColorRGBA &color)
  {
    visualization_msgs::Marker sphere_marker;

    sphere_marker.header.frame_id = BASE_FRAME;

    // Set the namespace and id for this marker.  This serves to create a unique ID
    sphere_marker.ns = "Sphere";
    // Set the marker type.
    sphere_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    // Set the marker action.  Options are ADD and DELETE
    sphere_marker.action = visualization_msgs::Marker::ADD;
    // Marker group position and orientation
    sphere_marker.pose.position.x = 0;
    sphere_marker.pose.position.y = 0;
    sphere_marker.pose.position.z = 0;
    sphere_marker.pose.orientation.x = 0.0;
    sphere_marker.pose.orientation.y = 0.0;
    sphere_marker.pose.orientation.z = 0.0;
    sphere_marker.pose.orientation.w = 1.0;

    // Add the point pair to the line message
    sphere_marker.points.push_back( point );
    sphere_marker.colors.push_back( color );
    // Lifetime
    //sphere_marker.lifetime = ros

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    sphere_marker.header.stamp = ros::Time::now();

    static int sphere_id_ = 0;
    sphere_marker.id = ++sphere_id_;
    sphere_marker.color = color;
    sphere_marker.scale.x = 0.3;
    sphere_marker.scale.y = 0.3;
    sphere_marker.scale.z = 0.3;

    // Update the single point with new pose
    sphere_marker.points[0] = point;
    sphere_marker.colors[0] = color;

    // Publish
    marker_pub_.publish( sphere_marker );
    ros::spinOnce();
  }

  
  // *********************************************************************************************************
  // Display Result Path
  // *********************************************************************************************************
  void displayResult( og::PathGeometric& path, std_msgs::ColorRGBA* color, bnu::matrix<int> &cost )
  {
    const std::vector<std::pair<double, double> > coordinates = convertSolutionToVector(path);

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
      point_a.z = getCostHeight(point_a, cost); 

      // Create a second point
      point_b.x = x2;
      point_b.y = y2;
      point_b.z = getCostHeight(point_b, cost);

      // Add the point pair to the line message
      marker.points.push_back( point_a );
      marker.points.push_back( point_b );
      marker.colors.push_back( *color );
      marker.colors.push_back( *color );

      // Save these coordinates for next line
      x1 = x2;
      y1 = y2;
    }

    // Publish the marker
    marker_pub_.publish( marker );
  }

  // *********************************************************************************************************
  // Helper Function: gets the x,y coordinates for a given vertex id
  // *********************************************************************************************************
  std::pair<double, double> getCoordinates( int vertex_id, ob::PlannerDataPtr planner_data )
  {
    ob::PlannerDataVertex vertex = planner_data->getVertex( vertex_id );

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
  

}; // end class

typedef boost::shared_ptr<OmplRvizViewer> OmplRvizViewerPtr;
typedef boost::shared_ptr<const OmplRvizViewer> OmplRvizViewerConstPtr;

} // end namespace
