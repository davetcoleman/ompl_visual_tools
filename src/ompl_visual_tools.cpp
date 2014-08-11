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

// This library
#include <ompl_visual_tools/ompl_visual_tools.h>

// ROS
#include <ros/ros.h>

// Boost
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>

// OMPL
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/PlannerData.h>
#include <ompl/base/ScopedState.h>

#include <ompl/config.h>

// Custom validity checker that accounts for cost
#include <ompl_visual_tools/costs/cost_map_2d_optimization_objective.h>

// For converting OMPL state to a MoveIt robot state
#include <moveit/ompl_interface/model_based_planning_context.h>
#include <moveit/robot_state/robot_state.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace bnu = boost::numeric::ublas;

using namespace moveit_visual_tools;

namespace ompl_visual_tools
{

OmplVisualTools::OmplVisualTools(const std::string& base_link, const std::string& marker_topic, robot_model::RobotModelConstPtr robot_model)
  : VisualTools(base_link, marker_topic, robot_model)
{
}

void OmplVisualTools::setStateSpace(ompl::base::StateSpacePtr space)
{
  si_.reset(new ompl::base::SpaceInformation(space));
}

void OmplVisualTools::setSpaceInformation(ompl::base::SpaceInformationPtr si)
{
  si_ = si;
}

void OmplVisualTools::setCostMap(intMatrixPtr cost)
{
  cost_ = cost;
}

double OmplVisualTools::getCost(const geometry_msgs::Point &point)
{
  // Check that a cost map has been passed in
  if (cost_)
  {
    return double((*cost_)( natRound(point.y), natRound(point.x) )) / 2.0;
  }
  else
    return 1;
}

double OmplVisualTools::getCostHeight(const geometry_msgs::Point &point)
{
  // TODO make faster

  // check if whole number
  if (floor(point.x) == point.x && floor(point.y) == point.y)
    return getCost(point) + COST_HEIGHT_OFFSET;

  // else do Bilinear Interpolation

  // top left
  geometry_msgs::Point a;
  a.x = floor(point.x);
  a.y = ceil(point.y);
  a.z = getCost(a);

  // bottom left
  geometry_msgs::Point b;
  b.x = floor(point.x);
  b.y = floor(point.y);
  b.z = getCost(b);

  // bottom right
  geometry_msgs::Point c;
  c.x = ceil(point.x);
  c.y = floor(point.y);
  c.z = getCost(c);

  // top right
  geometry_msgs::Point d;
  d.x = ceil(point.x);
  d.y = ceil(point.y);
  d.z = getCost(d);

  double R1;
  double R2;

  // check if our x axis is the same
  if (c.x == b.x)
  {
    // just choose either
    R1 = b.z;
    R2 = a.z;
  }
  else
  {
    R1 = ((c.x - point.x)/(c.x - b.x))*b.z + ((point.x - b.x)/(c.x - b.x))*c.z;
    R2 = ((c.x - point.x)/(c.x - b.x))*a.z + ((point.x - b.x)/(c.x - b.x))*d.z;
  }

  // After the two R values are calculated, the value of P can finally be calculated by a weighted average of R1 and R2.
  double val;

  // check if y axis is the same
  if ( a.y - b.y == 0) // division by zero
    val = R1;
  else
    val = ((a.y - point.y)/(a.y - b.y))*R1 + ((point.y - b.y)/(a.y - b.y))*R2;

  return val + COST_HEIGHT_OFFSET;
}

void OmplVisualTools::publishCostMap(PPMImage *image)
{
  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = base_frame_;
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
  marker.color = getColor( RED );

  // Visualize Results -------------------------------------------------------------------------------------------------
  for( size_t marker_id = 0; marker_id < image->getSize(); ++marker_id )
  {

    unsigned int x = marker_id % image->x;    // Map index back to coordinates
    unsigned int y = marker_id / image->x;    // Map index back to coordinates

    // Make right and down triangle
    // Check that we are not on the far right or bottom
    if( ! (x + 1 >= image->x ||  y + 1 >= image->y ) )
    {
      publishTriangle( x,   y, &marker, image);
      publishTriangle( x+1, y, &marker, image);
      publishTriangle( x,   y+1, &marker, image);
    }

    // Make back and down triangle
    // Check that we are not on the far left or bottom
    if( ! ( int(x) - 1 < 0 ||  y + 1 >= image->y ) )
    {
      publishTriangle( x,   y, &marker, image );
      publishTriangle( x,   y+1, &marker, image );
      publishTriangle( x-1, y+1, &marker, image );
    }

  }

  // Send to Rviz
  pub_rviz_marker_.publish( marker );
  ros::spinOnce();;
}

/**
 * \brief Helper Function to display triangles
 */
void OmplVisualTools::publishTriangle( int x, int y, visualization_msgs::Marker* marker, PPMImage *image )
{
  // Point
  temp_point_.x = x;
  temp_point_.y = y;
  temp_point_.z = getCost(temp_point_); // to speed things up, we know is always whole number
  marker->points.push_back( temp_point_ );

  std_msgs::ColorRGBA color;
  color.r = image->data[ image->getID( x, y ) ].red / 255.0;
  color.g = image->data[ image->getID( x, y ) ].green / 255.0;
  color.b = image->data[ image->getID( x, y ) ].blue / 255.0;
  color.a = 1.0;

  marker->colors.push_back( color );
}

void OmplVisualTools::interpolateLine( const geometry_msgs::Point &p1, const geometry_msgs::Point &p2, visualization_msgs::Marker* marker,
                                      const std_msgs::ColorRGBA color )
{
  // Copy to non-const
  geometry_msgs::Point point_a = p1;
  geometry_msgs::Point point_b = p2;

  // Get the heights
  point_a.z = getCostHeight(point_a);
  point_b.z = getCostHeight(point_b);

  //ROS_INFO_STREAM("a is: " << point_a);
  //ROS_INFO_STREAM("b is: " << point_b);

  // Switch the coordinates such that x1 < x2
  if( point_a.x > point_b.x )
  {
    // Swap the coordinates
    geometry_msgs::Point point_temp = point_a;
    point_a = point_b;
    point_b = point_temp;
  }

  // Show the straight line --------------------------------------------------------------------
  if( false )
  {
    // Add the point pair to the line message
    marker->points.push_back( point_a );
    marker->points.push_back( point_b );
    marker->colors.push_back( color );
    marker->colors.push_back( color );

    // Show start and end point
    publishSphere(point_a, GREEN);
    publishSphere(point_b, RED);
  }

  // Interpolate the line ----------------------------------------------------------------------

  // Calculate slope between the lines
  double m = (point_b.y - point_a.y)/(point_b.x - point_a.x);

  // Calculate the y-intercep
  double b = point_a.y - m * point_a.x;

  // Define the interpolation interval
  double interval = 0.1; //0.5;

  // Make new locations
  geometry_msgs::Point temp_a = point_a; // remember the last point
  geometry_msgs::Point temp_b = point_a; // move along this point

  // Loop through the line adding segements along the cost map
  for( temp_b.x = point_a.x + interval; temp_b.x <= point_b.x; temp_b.x += interval )
  {
    //publishSphere(temp_b, color2);

    // Find the y coordinate
    temp_b.y = m*temp_b.x + b;

    // Add the new heights
    temp_a.z = getCostHeight(temp_a);
    temp_b.z = getCostHeight(temp_b);

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

void OmplVisualTools::publishStartGoalSpheres(ob::PlannerDataPtr planner_data, const std::string& ns)
{
  for (std::size_t i = 0; i < planner_data->numStartVertices(); ++i)
  {
    publishSphere( stateToPointMsg( planner_data->getStartVertex(i).getState() ), GREEN, REGULAR, ns);
  }
  for (std::size_t i = 0; i < planner_data->numGoalVertices(); ++i)
  {
    publishSphere( stateToPointMsg( planner_data->getGoalVertex(i).getState() ), RED, REGULAR, ns );
  }
}

void OmplVisualTools::publishGraph(ob::PlannerDataPtr planner_data, const rviz_colors& color, const double thickness, const std::string& ns)
{
  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = base_frame_;
  marker.header.stamp = ros::Time();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  marker.ns = ns;

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

  marker.scale.x = thickness;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;

  marker.color = getColor( color );

  if (verbose_)
    ROS_INFO("Publishing Graph");

  geometry_msgs::Point this_vertex;
  geometry_msgs::Point next_vertex;

  // Loop through all verticies
  for( int vertex_id = 0; vertex_id < int( planner_data->numVertices() ); ++vertex_id )
  {
    this_vertex = stateToPointMsg( vertex_id, planner_data );

    // Get the out edges from the current vertex
    std::vector<unsigned int> edge_list;
    planner_data->getEdges( vertex_id, edge_list );

    // Now loop through each edge
    for( std::vector<unsigned int>::const_iterator edge_it = edge_list.begin();
         edge_it != edge_list.end(); ++edge_it)
    {
      // Convert vertex id to next coordinates
      next_vertex = stateToPointMsg( *edge_it, planner_data );

      interpolateLine( this_vertex, next_vertex, &marker, marker.color );
    }

  }

  // Send to Rviz
  pub_rviz_marker_.publish( marker );
  ros::spinOnce();;
}

void OmplVisualTools::publishSamples(const ob::PlannerDataPtr& plannerData)
{
  og::PathGeometric path(si_);
  convertPlannerData(plannerData, path);

  //std::size_t beforeInterpolateCount = path.getStateCount();
  //path.interpolate();
  //ROS_INFO_STREAM_NAMED("publishResult","Interpolation of path increased states count from "
  //    << beforeInterpolateCount << " to " << path.getStateCount());

  publishSamples(path);
}

void OmplVisualTools::publishSamples( og::PathGeometric& path )
{
  std::vector<geometry_msgs::Point> points;
  for (std::size_t i = 0; i < path.getStateCount(); ++i)
  {
    points.push_back( stateToPointMsg( path.getState(i) ) );
  }

  publishSpheres(points, RED, SMALL, "sample_locations");
}

void OmplVisualTools::convertPlannerData(const ob::PlannerDataPtr plannerData, og::PathGeometric &path)
{
  // Convert the planner data verticies into a vector of states
  for (std::size_t i = 0; i < plannerData->numVertices(); ++i)
  {
    path.append(plannerData->getVertex(i).getState());
  }
}

void OmplVisualTools::publishStates(std::vector<const ompl::base::State*> states)
{
  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.
  marker.header.frame_id = base_frame_;
  marker.header.stamp = ros::Time();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  marker.ns = "states";

  // Set the marker type.
  marker.type = visualization_msgs::Marker::SPHERE_LIST;

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

  marker.scale.x = 0.4;
  marker.scale.y = 0.4;
  marker.scale.z = 0.4;

  marker.color = getColor( RED );

  // Make line color
  std_msgs::ColorRGBA color = getColor( RED );

  // Point
  geometry_msgs::Point point_a;

  if (verbose_)
    ROS_INFO("Publishing Spheres");

  // Loop through all verticies
  for( int vertex_id = 0; vertex_id < int( states.size() ); ++vertex_id )
  {
    // First point
    point_a = stateToPointMsg( states[vertex_id] );

    // Add the point pair to the line message
    marker.points.push_back( point_a );
    marker.colors.push_back( color );
  }

  // Send to Rviz
  pub_rviz_marker_.publish( marker );
  ros::spinOnce();;
}

void OmplVisualTools::publishRobotPath( const ompl::base::PlannerDataPtr &path, robot_model::JointModelGroup* joint_model_group,
                                       const std::vector<const robot_model::LinkModel*> &tips, bool show_trajectory_animated)
{
  // Make sure a robot state is available
  loadSharedRobotState();

  // Vars
  Eigen::Affine3d pose;
  std::vector< std::vector<geometry_msgs::Point> > paths_msgs(tips.size()); // each tip has its own path of points
  robot_trajectory::RobotTrajectoryPtr robot_trajectory;

  ompl_interface::ModelBasedStateSpacePtr model_state_space =
    boost::static_pointer_cast<ompl_interface::ModelBasedStateSpace>(si_->getStateSpace());

  // Optionally save the trajectory
  if (show_trajectory_animated)
  {
    robot_trajectory.reset(new robot_trajectory::RobotTrajectory(robot_model_, joint_model_group->getName()));
  }

  // Each state in the path
  for (std::size_t state_id = 0; state_id < path->numVertices(); ++state_id)
  {
    // Check if program is shutting down
    if (!ros::ok())
      return;

    // Convert to robot state
    model_state_space->copyToRobotState( *shared_robot_state_, path->getVertex(state_id).getState() );
    //shared_robot_state_->update(true); // force update so that the virtual joint is updated to the grounded foot

    //publishRobotState(shared_robot_state_);

    // Each tip in the robot state
    for (std::size_t tip_id = 0; tip_id < tips.size(); ++tip_id)
    {
      // Forward kinematics
      pose = shared_robot_state_->getGlobalLinkTransform(tips[tip_id]);

      // Optionally save the trajectory
      if (show_trajectory_animated)
      {
        robot_state::RobotState robot_state_copy = *shared_robot_state_;
        robot_trajectory->addSuffixWayPoint(robot_state_copy, 0.05); // 1 second interval
      }

      // Debug pose
      //std::cout << "Pose: " << state_id << " of link " << tips[tip_id]->getName() << ": \n" << pose.translation() << std::endl;

      paths_msgs[tip_id].push_back( convertPose(pose).position );

      // Show goal state arrow
      if (state_id == path->numVertices() -1)
      {
        publishArrow( pose, moveit_visual_tools::BLACK );
      }
    }
  } // for each state

  for (std::size_t tip_id = 0; tip_id < tips.size(); ++tip_id)
  {
    VisualTools::publishPath( paths_msgs[tip_id], moveit_visual_tools::RAND, moveit_visual_tools::SMALL );
    ros::Duration(0.05).sleep();
    publishSpheres( paths_msgs[tip_id], moveit_visual_tools::ORANGE, moveit_visual_tools::SMALL );
    ros::Duration(0.05).sleep();
  }

  // Debugging - Convert to trajectory
  if (show_trajectory_animated)
  {
    publishTrajectoryPath(*robot_trajectory, true);
  }
}

void OmplVisualTools::publishPath( const ob::PlannerDataPtr& plannerData, const rviz_colors color,
                                  const double thickness, const std::string& ns )
{
  og::PathGeometric path(si_);
  convertPlannerData(plannerData, path);

  publishPath(path, color, thickness, ns);
}

void OmplVisualTools::publishPath( const og::PathGeometric& path, const rviz_colors color, const double thickness, const std::string& ns )
{
  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.
  marker.header.frame_id = base_frame_;
  marker.header.stamp = ros::Time();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  marker.ns = ns;

  std_msgs::ColorRGBA this_color = getColor( color );

  // Set the marker type.
  marker.type = visualization_msgs::Marker::LINE_LIST;

  // Set the marker action.  Options are ADD and DELETE
  marker.action = visualization_msgs::Marker::ADD;

  // Provide a new id every call to this function
  static int result_id = 0;
  marker.id = result_id++;

  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;

  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = thickness;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;

  marker.color = this_color;

  if (verbose_)
    ROS_INFO("Publishing Path ");

  geometry_msgs::Point last_vertex;
  geometry_msgs::Point this_vertex;

  if (path.getStateCount() <= 0)
  {
    ROS_WARN_STREAM_NAMED("publishPath","No states found in path");
    return;
  }

  // Initialize first vertex
  last_vertex = stateToPointMsg(path.getState(0));

  // Convert path coordinates
  for( std::size_t i = 1; i < path.getStateCount(); ++i )
  {
    // Get current coordinates
    this_vertex = stateToPointMsg(path.getState(i));

    // Publish line with interpolation
    interpolateLine( last_vertex, this_vertex, &marker, marker.color );

    // Save these coordinates for next line
    last_vertex = this_vertex;
  }

  // Send to Rviz
  pub_rviz_marker_.publish( marker );
  ros::spinOnce();
}

geometry_msgs::Point OmplVisualTools::stateToPointMsg( int vertex_id, ob::PlannerDataPtr planner_data )
{
  ob::PlannerDataVertex *vertex = &planner_data->getVertex( vertex_id );

  // Get this vertex's coordinates
  return stateToPointMsg(vertex->getState());
}

geometry_msgs::Point OmplVisualTools::stateToPointMsg( const ob::State *state )
{
  if (!state)
  {
    ROS_FATAL("No state found for a vertex");
    exit(1);
  }

  // Convert to RealVectorStateSpace
  const ob::RealVectorStateSpace::StateType *real_state =
    static_cast<const ob::RealVectorStateSpace::StateType*>(state);

  // Create point
  temp_point_.x = real_state->values[0];
  temp_point_.y = real_state->values[1];
  temp_point_.z = getCostHeight(temp_point_);
  return temp_point_;
}

geometry_msgs::Point OmplVisualTools::stateToPointMsg( const ob::ScopedState<> state )
{
  // Create point
  temp_point_.x = state[0];
  temp_point_.y = state[1];
  temp_point_.z = getCostHeight(temp_point_);
  return temp_point_;
}

int OmplVisualTools::natRound(double x)
{
  return static_cast<int>(floor(x + 0.5f));
}

void OmplVisualTools::publishState(const ob::ScopedState<> state, const rviz_colors &color, const rviz_scales scale, const std::string& ns)
{
  publishSphere( stateToPointMsg( state ), color, scale, ns);
}

void OmplVisualTools::publishSampleRegion(const ob::ScopedState<>& state_area, const double& distance)
{
  temp_point_.x = state_area[0];
  temp_point_.y = state_area[1];
  temp_point_.z = getCostHeight(temp_point_);

  publishSphere(temp_point_, BLACK, REGULAR, "sample_region"); // mid point
  // outer sphere (x2 b/c its a radius, x0.1 to make it look nicer)
  publishSphere(temp_point_, TRANSLUCENT, REGULAR, "sample_region");
}

bool OmplVisualTools::publishText(const std::string &text, const rviz_colors &color)
{
  geometry_msgs::Pose text_pose;
  text_pose.position.x = 0;
  text_pose.position.y = 0;
  text_pose.position.z = 0;
  publishText(text, text_pose, color);
}

bool OmplVisualTools::publishText(const std::string &text, const geometry_msgs::Pose &pose, const rviz_colors &color)
{
  visualization_msgs::Marker text_marker;
  text_marker.header.frame_id = base_frame_;
  // Set the namespace and id for this marker.  This serves to create a unique ID
  text_marker.ns = "text";
  // Set the marker action.  Options are ADD and DELETE
  text_marker.action = visualization_msgs::Marker::ADD;
  // Set the marker type.
  text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

  text_marker.id = 0;

  text_marker.header.stamp = ros::Time::now();
  text_marker.text = text;
  text_marker.pose = pose;
  text_marker.color = getColor( color );
  if (cost_)
    text_marker.scale.z = ceil(cost_->size1() / 20.0);    // only z is required (size of an "A")
  else
    text_marker.scale.z = 10;

  // Send to Rviz
  pub_rviz_marker_.publish( text_marker );
  ros::spinOnce();;

  return true;
}


} // end namespace
