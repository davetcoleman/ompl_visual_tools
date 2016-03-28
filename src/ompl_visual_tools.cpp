/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, University of Colorado, Boulder
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
#include <graph_msgs/GeometryGraph.h>
#include <graph_msgs/Edges.h>

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
#include <moveit_ompl/model_based_state_space.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/macros/deprecation.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace bnu = boost::numeric::ublas;

using namespace moveit_visual_tools;

namespace ompl_visual_tools
{
OmplVisualTools::OmplVisualTools(const std::string& base_link, const std::string& marker_topic,
                                 robot_model::RobotModelConstPtr robot_model)
  : MoveItVisualTools(base_link, marker_topic, robot_model)
  , disable_3d_(false)
  , max_edge_cost_(100.0)
  , min_edge_cost_(0.0)
  , min_edge_radius_(0.1)
  , max_edge_radius_(0.5)
  , invert_edge_cost_(false)
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

bool OmplVisualTools::checkSpaceInformation()
{
  if (!si_)
  {
    ROS_ERROR_STREAM_NAMED(name_, "No space information has been setup for ompl_visual_tools. Unable to visualize");
    return false;
  }
  return true;
}

void OmplVisualTools::setCostMap(intMatrixPtr cost)
{
  cost_ = cost;
}

/**
 * \brief Getter for disable 3d option in Rviz. This indicated there are no costs on the height map
 */
bool OmplVisualTools::getDisable3D()
{
  return disable_3d_;
}

/**
 * \brief Setter for disable 3d option in Rviz. This indicated there are no costs on the height map
 */
void OmplVisualTools::setDisable3D(bool disable_3d)
{
  disable_3d_ = disable_3d;
}

double OmplVisualTools::getCost(const geometry_msgs::Point& point)
{
  // Check that a cost map has been passed in
  if (cost_)
  {
    return double((*cost_)(natRound(point.y), natRound(point.x))) / 2.0;
  }
  else
    return 1;
}

double OmplVisualTools::getCostHeight(const geometry_msgs::Point& point)
{
  if (disable_3d_)
    return 0.5;  // all costs become almost zero in flat world

  // TODO make faster the following math

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
    R1 = ((c.x - point.x) / (c.x - b.x)) * b.z + ((point.x - b.x) / (c.x - b.x)) * c.z;
    R2 = ((c.x - point.x) / (c.x - b.x)) * a.z + ((point.x - b.x) / (c.x - b.x)) * d.z;
  }

  // After the two R values are calculated, the value of P can finally be calculated by a weighted average of R1 and R2.
  double val;

  // check if y axis is the same
  if (a.y - b.y == 0)  // division by zero
    val = R1;
  else
    val = ((a.y - point.y) / (a.y - b.y)) * R1 + ((point.y - b.y) / (a.y - b.y)) * R2;

  return val + COST_HEIGHT_OFFSET;
}

bool OmplVisualTools::publishCostMap(PPMImage* image, bool static_id)
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

  static std::size_t cost_map_id = 0;
  if (static_id)
  {
    marker.id = 0;
  }
  else
  {
    cost_map_id++;
    marker.id = cost_map_id;
  }

  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = -0.25;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.color = getColor(rviz_visual_tools::RED);

  // Visualize Results -------------------------------------------------------------------------------------------------
  for (size_t marker_id = 0; marker_id < image->getSize(); ++marker_id)
  {
    unsigned int x = marker_id % image->x;  // Map index back to coordinates
    unsigned int y = marker_id / image->x;  // Map index back to coordinates

    // Make right and down triangle
    // Check that we are not on the far right or bottom
    if (!(x + 1 >= image->x || y + 1 >= image->y))
    {
      publishTriangle(x, y, &marker, image);
      publishTriangle(x + 1, y, &marker, image);
      publishTriangle(x, y + 1, &marker, image);
    }

    // Make back and down triangle
    // Check that we are not on the far left or bottom
    if (!(int(x) - 1 < 0 || y + 1 >= image->y))
    {
      publishTriangle(x, y, &marker, image);
      publishTriangle(x, y + 1, &marker, image);
      publishTriangle(x - 1, y + 1, &marker, image);
    }
  }

  // Send to Rviz
  return publishMarker(marker);
}

/**
 * \brief Helper Function to display triangles
 */
bool OmplVisualTools::publishTriangle(int x, int y, visualization_msgs::Marker* marker, PPMImage* image)
{
  // Point
  temp_point_.x = x;
  temp_point_.y = y;
  if (disable_3d_)
    temp_point_.z = 0;  // all costs become zero in flat world
  else
    temp_point_.z = getCost(temp_point_);  // to speed things up, we know is always whole number

  marker->points.push_back(temp_point_);

  std_msgs::ColorRGBA color;
  color.r = image->data[image->getID(x, y)].red / 255.0;
  color.g = image->data[image->getID(x, y)].green / 255.0;
  color.b = image->data[image->getID(x, y)].blue / 255.0;
  color.a = 1.0;

  marker->colors.push_back(color);

  return true;
}

bool OmplVisualTools::interpolateLine(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2,
                                      visualization_msgs::Marker* marker, const std_msgs::ColorRGBA color)
{
  // Copy to non-const
  geometry_msgs::Point point_a = p1;
  geometry_msgs::Point point_b = p2;

  // Get the heights
  // point_a.z = getCostHeight(point_a);
  // point_b.z = getCostHeight(point_b);

  // ROS_INFO_STREAM("a is: " << point_a);
  // ROS_INFO_STREAM("b is: " << point_b);

  // Switch the coordinates such that x1 < x2
  if (point_a.x > point_b.x)
  {
    // Swap the coordinates
    geometry_msgs::Point point_temp = point_a;
    point_a = point_b;
    point_b = point_temp;
  }

  // Show the straight line --------------------------------------------------------------------
  if (false)
  {
    // Add the point pair to the line message
    marker->points.push_back(point_a);
    marker->points.push_back(point_b);
    marker->colors.push_back(color);
    marker->colors.push_back(color);

    // Show start and end point
    publishSphere(point_a, rviz_visual_tools::GREEN);
    publishSphere(point_b, rviz_visual_tools::RED);
  }

  // Interpolate the line ----------------------------------------------------------------------

  // Calculate slope between the lines
  double m = (point_b.y - point_a.y) / (point_b.x - point_a.x);

  // Calculate the y-intercep
  double b = point_a.y - m * point_a.x;

  // Define the interpolation interval
  double interval = 0.1;  // 0.5;

  // Make new locations
  geometry_msgs::Point temp_a = point_a;  // remember the last point
  geometry_msgs::Point temp_b = point_a;  // move along this point

  // Loop through the line adding segements along the cost map
  for (temp_b.x = point_a.x + interval; temp_b.x <= point_b.x; temp_b.x += interval)
  {
    // publishSphere(temp_b, color2);

    // Find the y coordinate
    temp_b.y = m * temp_b.x + b;

    // Add the new heights
    temp_a.z = getCostHeight(temp_a);
    temp_b.z = getCostHeight(temp_b);

    // Add the point pair to the line message
    marker->points.push_back(temp_a);
    marker->points.push_back(temp_b);
    // Add colors
    marker->colors.push_back(color);
    marker->colors.push_back(color);

    // Remember the last coordiante for next iteration
    temp_a = temp_b;
  }

  // Finish the line for non-even interval lengths
  marker->points.push_back(temp_a);
  marker->points.push_back(point_b);
  // Add colors
  marker->colors.push_back(color);
  marker->colors.push_back(color);

  return true;
}

bool OmplVisualTools::publishStartGoalSpheres(ob::PlannerDataPtr planner_data, const std::string& ns)
{
  for (std::size_t i = 0; i < planner_data->numStartVertices(); ++i)
  {
    publishSphere(convertPoint(stateToPoint(planner_data->getStartVertex(i).getState())), rviz_visual_tools::GREEN,
                  rviz_visual_tools::REGULAR, ns);
  }
  for (std::size_t i = 0; i < planner_data->numGoalVertices(); ++i)
  {
    publishSphere(convertPoint(stateToPoint(planner_data->getGoalVertex(i).getState())), rviz_visual_tools::RED,
                  rviz_visual_tools::REGULAR, ns);
  }

  return true;
}

bool OmplVisualTools::publishGraph(ob::PlannerDataPtr planner_data, const rviz_visual_tools::colors& color,
                                   const double thickness, const std::string& ns)
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

  marker.color = getColor(color);

  geometry_msgs::Point this_vertex;
  geometry_msgs::Point next_vertex;

  // Loop through all verticies
  for (std::size_t vertex_id = 0; vertex_id < planner_data->numVertices(); ++vertex_id)
  {
    this_vertex = convertPoint(stateToPoint(vertex_id, planner_data));

    // Get the out edges from the current vertex
    std::vector<unsigned int> edge_list;
    planner_data->getEdges(vertex_id, edge_list);

    // Now loop through each edge
    for (std::vector<unsigned int>::const_iterator edge_it = edge_list.begin(); edge_it != edge_list.end(); ++edge_it)
    {
      // Convert vertex id to next coordinates
      next_vertex = convertPoint(stateToPoint(*edge_it, planner_data));

      interpolateLine(this_vertex, next_vertex, &marker, marker.color);
    }
  }

  // Send to Rviz
  return publishMarker(marker);
}

bool OmplVisualTools::publishEdge(const ob::State* stateA, const ob::State* stateB, const std_msgs::ColorRGBA& color,
                                  const double radius)
{
  return RvizVisualTools::publishCylinder(stateToPoint(stateA), stateToPoint(stateB), color, radius);
}

bool OmplVisualTools::publishSampleIDs(const og::PathGeometric& path, const rviz_visual_tools::colors& color,
                                       const rviz_visual_tools::scales scale, const std::string& ns)
{
  // Create a small scale for font size
  geometry_msgs::Vector3 scale_msg;
  if (cost_)
  {
    int size = ceil(cost_->size1() / 30.0);  // only z is required (size of an "A")
    scale_msg.x = size;
    scale_msg.y = size;
    scale_msg.z = size;
  }
  else
    scale_msg = getScale(scale);  // TODO tune this

  std::string text;
  // Publish all
  for (std::size_t i = 0; i < path.getStateCount(); ++i)
  {
    text = boost::lexical_cast<std::string>(i + 2);

    // send to moveit_visual_tools
    MoveItVisualTools::publishText(convertPointToPose(stateToPoint(path.getState(i))), text, color, scale_msg, false);
  }

  return true;
}

bool OmplVisualTools::publishSpheres(const ob::PlannerDataPtr& planner_data, const rviz_visual_tools::colors& color,
                                     const rviz_visual_tools::scales scale, const std::string& ns)
{
  // Error check
  if (!checkSpaceInformation())
    return false;

  og::PathGeometric path(si_);
  convertPlannerData(planner_data, path);

  return publishSpheres(path, color, scale, ns);
}

bool OmplVisualTools::publishSpheres(const og::PathGeometric& path, const rviz_visual_tools::colors& color, double scale,
                                     const std::string& ns)
{
  geometry_msgs::Vector3 scale_vector;
  scale_vector.x = scale;
  scale_vector.y = scale;
  scale_vector.z = scale;
  publishSpheres(path, color, scale_vector, ns);
}

bool OmplVisualTools::publishSpheres(const og::PathGeometric& path, const rviz_visual_tools::colors& color,
                                     const rviz_visual_tools::scales scale, const std::string& ns)
{
  publishSpheres(path, color, getScale(scale, false, 0.25), ns);
}

bool OmplVisualTools::publishSpheres(const og::PathGeometric& path, const rviz_visual_tools::colors& color,
                                     const geometry_msgs::Vector3& scale, const std::string& ns)
{
  std::vector<geometry_msgs::Point> points;
  for (std::size_t i = 0; i < path.getStateCount(); ++i)
    points.push_back(convertPoint(stateToPoint(path.getState(i))));

  return MoveItVisualTools::publishSpheres(points, color, scale, ns);

  // Show the vertex ids
  // publishSampleIDs( path, rviz_visual_tools::BLACK );
}

void OmplVisualTools::convertPlannerData(const ob::PlannerDataPtr planner_data, og::PathGeometric& path)
{
  // Convert the planner data verticies into a vector of states
  for (std::size_t i = 0; i < planner_data->numVertices(); ++i)
  {
    path.append(planner_data->getVertex(i).getState());
  }
}

bool OmplVisualTools::publishStates(std::vector<const ompl::base::State*> states)
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

  marker.color = getColor(rviz_visual_tools::RED);

  // Make line color
  std_msgs::ColorRGBA color = getColor(rviz_visual_tools::RED);

  // Point
  geometry_msgs::Point point_a;

  // Loop through all verticies
  for (int vertex_id = 0; vertex_id < int(states.size()); ++vertex_id)
  {
    // First point
    point_a = convertPoint(stateToPoint(states[vertex_id]));

    // Add the point pair to the line message
    marker.points.push_back(point_a);
    marker.colors.push_back(color);
  }

  // Send to Rviz
  return publishMarker(marker);
}

bool OmplVisualTools::publishRobotState(const ompl::base::State* state)
{
  // Error check
  if (!checkSpaceInformation())
    return false;

  // Make sure a robot state is available
  loadSharedRobotState();

  moveit_ompl::ModelBasedStateSpacePtr mb_state_space =
    boost::static_pointer_cast<moveit_ompl::ModelBasedStateSpace>(si_->getStateSpace());

  // Convert to robot state
  mb_state_space->copyToRobotState(*shared_robot_state_, state);

  // Show the robot visualized in Rviz
  MoveItVisualTools::publishRobotState(shared_robot_state_);
}

bool OmplVisualTools::publishRobotPath(const ompl::base::PlannerDataPtr& path,
                                       robot_model::JointModelGroup* jmg,
                                       const std::vector<const robot_model::LinkModel*>& tips,
                                       bool show_trajectory_animated)
{
  // Error check
  if (!checkSpaceInformation())
    return false;

  // Make sure a robot state is available
  loadSharedRobotState();

  // Vars
  Eigen::Affine3d pose;
  std::vector<std::vector<geometry_msgs::Point> > paths_msgs(tips.size());  // each tip has its own path of points
  robot_trajectory::RobotTrajectoryPtr robot_trajectory;

  moveit_ompl::ModelBasedStateSpacePtr mb_state_space =
    boost::static_pointer_cast<moveit_ompl::ModelBasedStateSpace>(si_->getStateSpace());

  // Optionally save the trajectory
  if (show_trajectory_animated)
  {
    robot_trajectory.reset(new robot_trajectory::RobotTrajectory(robot_model_, jmg->getName()));
  }

  // Each state in the path
  for (std::size_t state_id = 0; state_id < path->numVertices(); ++state_id)
  {
    // Check if program is shutting down
    if (!ros::ok())
      return false;

    // Convert to robot state
    mb_state_space->copyToRobotState(*shared_robot_state_, path->getVertex(state_id).getState());
    ROS_WARN_STREAM_NAMED("temp", "updateStateWithFakeBase disabled");
    // shared_robot_state_->updateStateWithFakeBase();

    MoveItVisualTools::publishRobotState(shared_robot_state_);

    // Each tip in the robot state
    for (std::size_t tip_id = 0; tip_id < tips.size(); ++tip_id)
    {
      // Forward kinematics
      pose = shared_robot_state_->getGlobalLinkTransform(tips[tip_id]);

      // Optionally save the trajectory
      if (show_trajectory_animated)
      {
        robot_state::RobotState robot_state_copy = *shared_robot_state_;
        robot_trajectory->addSuffixWayPoint(robot_state_copy, 0.05);  // 1 second interval
      }

      // Debug pose
      // std::cout << "Pose: " << state_id << " of link " << tips[tip_id]->getName() << ": \n" << pose.translation() <<
      // std::endl;

      paths_msgs[tip_id].push_back(convertPose(pose).position);

      // Show goal state arrow
      if (state_id == path->numVertices() - 1)
      {
        publishArrow(pose, rviz_visual_tools::BLACK);
      }
    }
  }  // for each state

  for (std::size_t tip_id = 0; tip_id < tips.size(); ++tip_id)
  {
    MoveItVisualTools::publishPath(paths_msgs[tip_id], rviz_visual_tools::RAND, rviz_visual_tools::SMALL);
    ros::Duration(0.05).sleep();
    MoveItVisualTools::publishSpheres(paths_msgs[tip_id], rviz_visual_tools::ORANGE, rviz_visual_tools::SMALL);
    ros::Duration(0.05).sleep();
  }

  // Debugging - Convert to trajectory
  if (show_trajectory_animated)
  {
    publishTrajectoryPath(*robot_trajectory, true);
  }

  return true;
}

bool OmplVisualTools::publishRobotPath(const og::PathGeometric& path,
                                       const robot_model::JointModelGroup* jmg,
                                       const bool wait_for_trajetory)
{
  // Convert to MoveIt! format
  robot_trajectory::RobotTrajectoryPtr traj;
  if (!convertPath(path, jmg, traj))
  {
    return false;
  }

  return MoveItVisualTools::publishTrajectoryPath(*traj, wait_for_trajetory);
}

bool OmplVisualTools::publishRobotGraph(const ompl::base::PlannerDataPtr& graph,
                                        const std::vector<const robot_model::LinkModel*>& tips)
{
  // Make sure a robot state is available
  loadSharedRobotState();

  // Turn into multiple graphs (one for each tip)
  std::vector<graph_msgs::GeometryGraph> graphs(tips.size());

  // Convert states to points
  std::vector<std::vector<geometry_msgs::Point> > vertex_tip_points;
  convertRobotStatesToTipPoints(graph, tips, vertex_tip_points);

  // Copy tip points to each resulting tip graph
  for (std::size_t vertex_id = 0; vertex_id < vertex_tip_points.size(); ++vertex_id)
  {
    for (std::size_t tip_id = 0; tip_id < tips.size(); ++tip_id)
    {
      graphs[tip_id].nodes.push_back(vertex_tip_points[vertex_id][tip_id]);
    }
  }

  // Now just copy the edges into each structure
  for (std::size_t vertex_id = 0; vertex_id < graph->numVertices(); ++vertex_id)
  {
    // Get the out edges from the current vertex
    std::vector<unsigned int> edge_list;
    graph->getEdges(vertex_id, edge_list);

    // Do for each tip
    for (std::size_t tip_id = 0; tip_id < tips.size(); ++tip_id)
    {
      // Create new edge with all the node ids listed
      graph_msgs::Edges edge;
      edge.node_ids = edge_list;
      graphs[tip_id].edges.push_back(edge);

    }  // for each tip
  }    // for each vertex

  // Now publish each tip graph
  for (std::size_t tip_id = 0; tip_id < tips.size(); ++tip_id)
  {
    const rviz_visual_tools::colors color = getRandColor();
    //std::cout << "Color is  " << color << std::endl;
    MoveItVisualTools::publishGraph(graphs[tip_id], color, 0.005);
    ros::Duration(0.1).sleep();

    MoveItVisualTools::publishSpheres(graphs[tip_id].nodes, rviz_visual_tools::ORANGE, rviz_visual_tools::SMALL);
    ros::Duration(0.1).sleep();
  }

  return true;
}

// Deprecated
bool OmplVisualTools::publishPath(const ob::PlannerDataPtr& planner_data, const rviz_visual_tools::colors& color,
                                  const double thickness, const std::string& ns)
{
  // Error check
  if (!checkSpaceInformation())
    return false;

  og::PathGeometric path(si_);
  convertPlannerData(planner_data, path);

  return publishPath(path, color, thickness, ns);
}

// TODO: deprecate
bool OmplVisualTools::publishPath(const og::PathGeometric& path, const rviz_visual_tools::colors& color,
                                  const double thickness, const std::string& ns)
{
  return publish2DPath(path, color, thickness, ns);
}

bool OmplVisualTools::publish2DPath(const og::PathGeometric& path, const rviz_visual_tools::colors& color,
                                    const double thickness, const std::string& ns)
{
  // Error check
  if (path.getStateCount() <= 0)
  {
    ROS_WARN_STREAM_NAMED(name_, "No states found in path");
    return false;
  }

  // Initialize first vertex
  Eigen::Vector3d prev_vertex = stateToPoint(path.getState(0));
  Eigen::Vector3d this_vertex;

  // Convert path coordinates
  for (std::size_t i = 1; i < path.getStateCount(); ++i)
  {
    // Get current coordinates
    this_vertex = stateToPoint(path.getState(i));

    // Create line
    publishCylinder(prev_vertex, this_vertex, color, thickness, ns);

    // Save these coordinates for next line
    prev_vertex = this_vertex;
  }

  return true;
}

Eigen::Vector3d OmplVisualTools::stateToPoint(std::size_t vertex_id, ob::PlannerDataPtr planner_data)
{
  ob::PlannerDataVertex* vertex = &planner_data->getVertex(vertex_id);

  // Get this vertex's coordinates
  return stateToPoint(vertex->getState());
}

Eigen::Vector3d OmplVisualTools::stateToPoint(const ob::ScopedState<> state)
{
  return stateToPoint(state.get());
}

Eigen::Vector3d OmplVisualTools::stateToPoint(const ob::State* state)
{
  // Error check
  if (!checkSpaceInformation())
    exit(-1);

  if (!state)
  {
    ROS_FATAL_NAMED(name_, "No state found for vertex");
    exit(1);
  }

  // Handle 2D world
  if (si_->getStateSpace()->getDimension() <= 3)
  {
    return stateToPoint2D(state);
  }

  // Handle robot world
  return stateToPointRobot(state);
}

Eigen::Vector3d OmplVisualTools::stateToPoint2D(const ob::State* state)
{
  // Convert to RealVectorStateSpace
  const ob::RealVectorStateSpace::StateType* real_state =
    static_cast<const ob::RealVectorStateSpace::StateType*>(state);

  // Create point
  temp_eigen_point_.x() = real_state->values[0];
  temp_eigen_point_.y() = real_state->values[1];
  temp_eigen_point_.z() = real_state->values[2] * 10;
  return temp_eigen_point_;
}

Eigen::Vector3d OmplVisualTools::stateToPointRobot(const ob::State* state)
{
  // Make sure a robot state is available
  loadSharedRobotState();

  moveit_ompl::ModelBasedStateSpacePtr mb_state_space =
    boost::static_pointer_cast<moveit_ompl::ModelBasedStateSpace>(si_->getStateSpace());

  // Convert to robot state
  mb_state_space->copyToRobotState(*shared_robot_state_, state);

  // Get pose
  // TODO(davetcoleman): do not hard code
  Eigen::Affine3d pose = shared_robot_state_->getGlobalLinkTransform("right_gripper_target");
  return pose.translation();
}

int OmplVisualTools::natRound(double x)
{
  return static_cast<int>(floor(x + 0.5f));
}

bool OmplVisualTools::publishState(const ob::State* state, const rviz_visual_tools::colors& color,
                                   const rviz_visual_tools::scales scale, const std::string& ns)
{
  return publishSphere(stateToPoint(state), color, scale, ns);
}

bool OmplVisualTools::publishState(const ob::State* state, const rviz_visual_tools::colors& color,
                                   const double scale, const std::string& ns)
{
  return publishSphere(stateToPoint(state), color, scale, ns);
}

bool OmplVisualTools::publishState(const ob::ScopedState<> state, const rviz_visual_tools::colors& color,
                                   const rviz_visual_tools::scales scale, const std::string& ns)
{
  return publishSphere(stateToPoint(state), color, scale, ns);
}

bool OmplVisualTools::publishState(const ob::ScopedState<> state, const rviz_visual_tools::colors& color, double scale,
                                   const std::string& ns)
{
  return publishSphere(stateToPoint(state), color, scale, ns);
}

bool OmplVisualTools::publishState(const ob::ScopedState<> state, const rviz_visual_tools::colors& color,
                                   const geometry_msgs::Vector3& scale, const std::string& ns)
{
  return publishSphere(stateToPoint(state), color, scale.x, ns);
}

bool OmplVisualTools::publishSampleRegion(const ob::ScopedState<>& state_area, const double& distance)
{
  temp_point_.x = state_area[0];
  temp_point_.y = state_area[1];
  temp_point_.z = state_area[2];  // getCostHeight(temp_point_);

  publishSphere(temp_point_, rviz_visual_tools::BLACK, rviz_visual_tools::REGULAR, "sample_region");  // mid point
  // outer sphere (x2 b/c its a radius, x0.1 to make it look nicer)
  return publishSphere(temp_point_, rviz_visual_tools::TRANSLUCENT, rviz_visual_tools::REGULAR, "sample_region");
}

bool OmplVisualTools::publishText(const geometry_msgs::Point& point, const std::string& text,
                                  const rviz_visual_tools::colors& color, bool static_id)
{
  geometry_msgs::Pose text_pose;
  text_pose.position = point;

  return publishText(text_pose, text, color, static_id);
}

bool OmplVisualTools::publishText(const geometry_msgs::Pose& pose, const std::string& text,
                                  const rviz_visual_tools::colors& color, bool static_id)
{
  geometry_msgs::Vector3 scale;
  if (cost_)
  {
    int size = ceil(cost_->size1() / 20.0);  // only z is required (size of an "A")
    scale.x = size;
    scale.y = size;
    scale.z = size;
  }
  else
    scale = getScale(rviz_visual_tools::REGULAR);  // TODO tune this

  // send to moveit_visual_tools
  return RvizVisualTools::publishText(pose, text, color, scale, static_id);
}

bool OmplVisualTools::convertRobotStatesToTipPoints(const ompl::base::PlannerDataPtr& graph,
                                                    const std::vector<const robot_model::LinkModel*>& tips,
                                                    std::vector<std::vector<geometry_msgs::Point> >& vertex_tip_points)
{
  // Error check
  if (!checkSpaceInformation())
    return false;

  // Make sure a robot state is available
  loadSharedRobotState();

  // Vars
  Eigen::Affine3d pose;

  // Load information about the robot's geometry
  moveit_ompl::ModelBasedStateSpacePtr mb_state_space =
    boost::static_pointer_cast<moveit_ompl::ModelBasedStateSpace>(si_->getStateSpace());

  // Rows correspond to robot states
  vertex_tip_points.clear();
  vertex_tip_points.resize(graph->numVertices());

  // Each state in the path
  for (std::size_t state_id = 0; state_id < graph->numVertices(); ++state_id)
  {
    // Convert to robot state
    mb_state_space->copyToRobotState(*shared_robot_state_, graph->getVertex(state_id).getState());
    ROS_WARN_STREAM_NAMED("temp", "updateStateWithFakeBase disabled");
    // shared_robot_state_->updateStateWithFakeBase();

    // Each tip in the robot state
    for (std::size_t tip_id = 0; tip_id < tips.size(); ++tip_id)
    {
      // Forward kinematics
      pose = shared_robot_state_->getGlobalLinkTransform(tips[tip_id]);

      vertex_tip_points[state_id].push_back(convertPose(pose).position);
    }
  }

  return true;
}

bool OmplVisualTools::convertPath(const og::PathGeometric& path,
                                  const robot_model::JointModelGroup* jmg,
                                  robot_trajectory::RobotTrajectoryPtr& traj, double speed)
{
  // Error check
  if (path.getStateCount() <= 0)
  {
    ROS_WARN_STREAM_NAMED(name_, "No states found in path");
    return false;
  }

  // New trajectory
  traj.reset(new robot_trajectory::RobotTrajectory(robot_model_, jmg));
  moveit::core::RobotState state(*shared_robot_state_); // TODO(davetcoleman):do i need to copy this?

  // Get correct type of space
  moveit_ompl::ModelBasedStateSpacePtr mb_state_space =
    boost::static_pointer_cast<moveit_ompl::ModelBasedStateSpace>(si_->getStateSpace());

  // Convert solution to MoveIt! format, reversing the solution
  //for (std::size_t i = path.getStateCount(); i > 0; --i)
  for (std::size_t i = 0; i < path.getStateCount(); ++i)
  {
    // Convert format
    mb_state_space->copyToRobotState(state, path.getState(i));

    // Add to trajectory
    traj->addSuffixWayPoint(state, speed);
  }
}

void OmplVisualTools::vizTriggerCallback()
{
  triggerBatchPublish();
}

// TODO: deprecate?
void OmplVisualTools::vizCallback(ompl::base::Planner* planner)
{
  // Error check
  if (!checkSpaceInformation())
    return;

  deleteAllMarkers();
  ros::spinOnce();

  ompl::base::PlannerDataPtr planner_data(new ompl::base::PlannerData(si_));
  planner->getPlannerData(*planner_data);

  std::cout << "vizCallback has states: " << planner_data->numVertices() << std::endl;

  // Show edges of graph
  publishGraph(planner_data, rviz_visual_tools::PURPLE);

  // Convert planner data to path
  og::PathGeometric path(si_);
  convertPlannerData(planner_data, path);

  // Show samples of graph
  OmplVisualTools::publishSpheres(path, rviz_visual_tools::ORANGE, rviz_visual_tools::SMALL);

  // Outline with circle
  // double nn_radius = 3.46482;
  // OmplVisualTools::publishSpheres( path, rviz_visual_tools::TRANSLUCENT, nn_radius*2 );

  ros::Duration(0.1).sleep();

  ros::spinOnce();
}

void OmplVisualTools::vizStateCallback(const ompl::base::State* state, std::size_t type, double neighborRadius)
{
  // Error check
  if (!checkSpaceInformation())
    return;

  if (si_->getStateSpace()->getDimension() <= 3)
  {
    vizState2DCallback(stateToPoint(state), type, neighborRadius);
  }
  else
  {
    // Make sure a robot state is available
    loadSharedRobotState();

    moveit_ompl::ModelBasedStateSpacePtr mb_state_space =
      boost::static_pointer_cast<moveit_ompl::ModelBasedStateSpace>(si_->getStateSpace());

    // Convert to robot state
    mb_state_space->copyToRobotState(*shared_robot_state_, state);

    // Show the joint limits in the console
    // MoveItVisualTools::showJointLimits(shared_robot_state_);

    switch (type)
    {
      case 1:  // Candidate COEVERAGE node to be added
        MoveItVisualTools::publishRobotState(shared_robot_state_, rviz_visual_tools::GREEN);
        break;
      case 2:  // Candidate CONNECTIVITY node to be added
        MoveItVisualTools::publishRobotState(shared_robot_state_, rviz_visual_tools::BLUE);
        break;
      case 3:  // sampled nearby node
        MoveItVisualTools::publishRobotState(shared_robot_state_, rviz_visual_tools::RED);
        break;
      case 4:  // Candidate node has already been added
        MoveItVisualTools::publishRobotState(shared_robot_state_, rviz_visual_tools::PURPLE);
        break;
      case 5:  // Large node
        MoveItVisualTools::publishRobotState(shared_robot_state_, rviz_visual_tools::BLACK);
        break;
      case 6:  // Only show red sphere, no robot state
        {
          // Publish sphere
          Eigen::Affine3d pose = shared_robot_state_->getGlobalLinkTransform("right_gripper_target");
          publishSphere(pose, rviz_visual_tools::RED, rviz_visual_tools::REGULAR);
        }
        break;
      default:
        ROS_ERROR_STREAM_NAMED(name_, "Invalid state type value");
    }
  }
}

void OmplVisualTools::vizState2DCallback(const Eigen::Vector3d& point, std::size_t type, double neighborRadius)
{
  batch_publishing_enabled_ = true;  // when using the callbacks, all pubs must be manually triggered

  switch (type)
  {
    case 0:  // This deletes all markers
      deleteAllMarkers();
      break;
    case 1:  // Candidate COEVERAGE node to be added
      publishSphere(point, rviz_visual_tools::GREEN, rviz_visual_tools::SMALL);
      break;
    case 2:  // Candidate CONNECTIVITY node to be added
      publishSphere(point, rviz_visual_tools::BLUE, rviz_visual_tools::SMALL);
      break;
    case 3:  // sampled nearby node
      publishSphere(point, rviz_visual_tools::RED, rviz_visual_tools::SMALL);
      break;
    case 4:  // Candidate node has already been added
      publishSphere(point, rviz_visual_tools::PURPLE, rviz_visual_tools::REGULAR);
      // Outline with circle
      publishSphere(point, rviz_visual_tools::TRANSLUCENT, neighborRadius * 2);
      break;
    case 5:  // Large node
      publishSphere(point, rviz_visual_tools::BLACK, rviz_visual_tools::LARGE);
      break;
    default:
      ROS_ERROR_STREAM_NAMED(name_, "Invalid state type value");
  }
}

void OmplVisualTools::vizEdgeCallback(const ompl::base::State* stateA, const ompl::base::State* stateB, double cost)
{
  batch_publishing_enabled_ = true;  // when using the callbacks, all pubs must be manually triggered

  // Convert input cost
  double percent = (cost - min_edge_cost_) / (max_edge_cost_ - min_edge_cost_);

  // Swap colors
  if (invert_edge_cost_)
  {
    percent = 1 - percent;
  }

  //const double radius = percent / 6.0 + 0.15;
  const double radius = (max_edge_radius_ - min_edge_radius_) * percent + min_edge_radius_;

  if (false)
    std::cout << "cost: " << cost << " min_edge_cost_: " << min_edge_cost_ << " max_edge_cost_: " << max_edge_cost_
              << " percent: " << percent << " radius: " << radius << std::endl;

  publishEdge(stateA, stateB, getColorScale(percent), radius);
}

}  // end namespace
