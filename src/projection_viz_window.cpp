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

#include <ompl_visual_tools/projection_viz_window.h>

// ROS
#include <ros/ros.h>
#include <graph_msgs/GeometryGraph.h>
#include <graph_msgs/Edges.h>

// Boost
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>

// OMPL
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/ScopedState.h>

// Custom validity checker that accounts for cost
#include <ompl_visual_tools/costs/cost_map_2d_optimization_objective.h>

// MoveIt
#include <moveit_ompl/model_based_state_space.h>

//#include <moveit/macros/deprecation.h>

namespace ot = ompl::tools;
namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace bnu = boost::numeric::ublas;

namespace ompl_visual_tools
{
ProjectionVizWindow::ProjectionVizWindow(rviz_visual_tools::RvizVisualToolsPtr visuals, ompl::base::SpaceInformationPtr si)
  : name_("projection_viz_window"), visuals_(visuals), si_(si)
{
  // with this OMPL interface to Rviz all pubs must be manually triggered
  visuals_->enableBatchPublishing(false);

  // Calculate ranges
  moveit_ompl::ModelBasedStateSpacePtr mb_state_space =
      std::static_pointer_cast<moveit_ompl::ModelBasedStateSpace>(si_->getStateSpace());
  ompl::base::RealVectorBounds bounds = mb_state_space->getBounds();

  // Only allow 2 or 3 dimensions
  BOOST_ASSERT_MSG(si_->getStateSpace()->getDimension() == 2 || si_->getStateSpace()->getDimension() == 3, "Invalid number of dimensions");

  range0_ = fabs(bounds.high[0] - bounds.low[0]);
  range1_ = fabs(bounds.high[1] - bounds.low[1]);
  if (si_->getStateSpace()->getDimension() == 3)
    range2_ = fabs(bounds.high[2] - bounds.low[2]);
  BOOST_ASSERT_MSG(range0_ > 0, "Range is zero");
  BOOST_ASSERT_MSG(range1_ > 0, "Range is zero");

  ROS_DEBUG_STREAM_NAMED(name_, "Initializing ProjectionVizWindow()");
}

void ProjectionVizWindow::state(const ompl::base::State* state, ot::VizSizes size, ot::VizColors color, double extra_data)
{
  Eigen::Vector3d point2 = stateToPoint(state);

  // Optional - show state above lines - looks good in 2D but not 3D TODO(davetcoleman): how to auto choose this
  if (false)
    point2.z() += 0.5;

  switch (size)
  {
    case ompl::tools::SMALL:
      visuals_->publishSphere(point2, visuals_->intToRvizColor(color), rvt::SMALL);
      break;
    case ompl::tools::MEDIUM:
      visuals_->publishSphere(point2, visuals_->intToRvizColor(color), rvt::REGULAR);
      break;
    case ompl::tools::LARGE:
      visuals_->publishSphere(point2, visuals_->intToRvizColor(color), rvt::LARGE);
      break;
    case ompl::tools::VARIABLE_SIZE:
      visuals_->publishSphere(point2, visuals_->intToRvizColor(color), extra_data * 2);
      break;
    case ompl::tools::SCALE:
    {
      const double percent = (extra_data - min_edge_cost_) / (max_edge_cost_ - min_edge_cost_);
      const double radius = ((max_state_radius_ - min_state_radius_) * percent + min_state_radius_);
      geometry_msgs::Vector3 scale;
      scale.x = radius;
      scale.y = radius;
      scale.z = radius;
      visuals_->publishSphere(visuals_->convertPointToPose(point2), visuals_->getColorScale(percent), scale);
    }
    break;
    case ompl::tools::SMALL_TRANSLUCENT:
      visuals_->publishSphere(point2, rvt::TRANSLUCENT_LIGHT, extra_data);
      break;
    default:
      ROS_ERROR_STREAM_NAMED(name_, "vizState2D: Invalid state size value");
  }
}


void ProjectionVizWindow::edge(const ompl::base::State* stateA, const ompl::base::State* stateB, double cost)
{
  // Error check
  if (si_->getStateSpace()->equalStates(stateA, stateB))
  {
    ROS_WARN_STREAM_NAMED(name_, "Unable to visualize edge because states are the same");
    visuals_->publishSphere(stateToPoint(stateA), rvt::RED, rvt::XLARGE);
    visuals_->triggerBatchPublish();
    ros::Duration(0.01).sleep();
    return;
  }

  // Convert input cost
  double percent = (cost - min_edge_cost_) / (max_edge_cost_ - min_edge_cost_);

  // Swap colors
  if (invert_edge_cost_)
  {
    percent = 1 - percent;
  }

  // const double radius = percent / 6.0 + 0.15;
  const double radius = (max_edge_radius_ - min_edge_radius_) * percent + min_edge_radius_;

  if (false)
    std::cout << "cost: " << cost << " min_edge_cost_: " << min_edge_cost_ << " max_edge_cost_: " << max_edge_cost_
              << " percent: " << percent << " radius: " << radius << std::endl;

  publishEdge(stateA, stateB, visuals_->getColorScale(percent), radius);
}

void ProjectionVizWindow::edge(const ompl::base::State* stateA, const ompl::base::State* stateB,
                        ot::VizSizes size, ot::VizColors color)
{
  Eigen::Vector3d pointA = stateToPoint(stateA);
  Eigen::Vector3d pointB = stateToPoint(stateB);

  double radius;
  switch (size)
  {
    case ompl::tools::SMALL:
      radius = 0.01;
      break;
    case ompl::tools::MEDIUM:
      radius = 0.1;
      break;
    case ompl::tools::LARGE:
      radius = 1;
      break;
    default:
      OMPL_ERROR("Unknown size");
      exit(-1);
  }

  visuals_->publishCylinder(pointA, pointB, visuals_->intToRvizColor(color), radius);
}

void ProjectionVizWindow::path(ompl::geometric::PathGeometric* path, ompl::tools::VizSizes type, ot::VizColors color)
{
  // Convert
  const og::PathGeometric& geometric_path = *path;  // static_cast<og::PathGeometric&>(*path);

  switch (type)
  {
    case ompl::tools::SMALL:  // Basic black line with vertiices
      publish2DPath(geometric_path, visuals_->intToRvizColor(color), min_edge_radius_);
      publishSpheres(geometric_path, visuals_->intToRvizColor(color), rvt::SMALL);
      break;
    case ompl::tools::ROBOT:
      // Playback motion for real robot, which is not applicable for this space
      break;
    default:
      ROS_ERROR_STREAM_NAMED(name_, "Invalid vizPath type value " << type);
  }
}

void ProjectionVizWindow::trigger()
{
  visuals_->triggerBatchPublish();
}

void ProjectionVizWindow::deleteAllMarkers()
{
  visuals_->deleteAllMarkers();
}

bool ProjectionVizWindow::shutdownRequested()
{
  if (!ros::ok())
  {
    std::cout << std::endl;
    std::cout << "-------------------------------------------------------" << std::endl;
    std::cout << "Shutting down process by request of ros::ok()" << std::endl;
    std::cout << "-------------------------------------------------------" << std::endl;
    return true;
  }
  return false;
}

// From ompl_visual_tools ------------------------------------------------------

bool ProjectionVizWindow::publishEdge(const ob::State* stateA, const ob::State* stateB, const std_msgs::ColorRGBA& color,
                                  const double radius)
{
  return visuals_->publishCylinder(stateToPoint(stateA), stateToPoint(stateB), color, radius / 2.0);
}

bool ProjectionVizWindow::publishSpheres(const og::PathGeometric& path, const rvt::colors& color, double scale,
                                     const std::string& ns)
{
  geometry_msgs::Vector3 scale_vector;
  scale_vector.x = scale;
  scale_vector.y = scale;
  scale_vector.z = scale;
  return publishSpheres(path, color, scale_vector, ns);
}

bool ProjectionVizWindow::publishSpheres(const og::PathGeometric& path, const rvt::colors& color, const rvt::scales scale,
                                     const std::string& ns)
{
  return publishSpheres(path, color, visuals_->getScale(scale, false, 0.25), ns);
}

bool ProjectionVizWindow::publishSpheres(const og::PathGeometric& path, const rvt::colors& color,
                                     const geometry_msgs::Vector3& scale, const std::string& ns)
{
  std::vector<geometry_msgs::Point> points;
  for (std::size_t i = 0; i < path.getStateCount(); ++i)
    points.push_back(visuals_->convertPoint(stateToPoint(path.getState(i))));

  return visuals_->publishSpheres(points, color, scale, ns);
}

bool ProjectionVizWindow::publishStates(std::vector<const ompl::base::State*> states)
{
  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.
  marker.header.frame_id = visuals_->getBaseFrame();
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

  marker.color = visuals_->getColor(rvt::RED);

  // Make line color
  std_msgs::ColorRGBA color = visuals_->getColor(rvt::RED);

  // Point
  geometry_msgs::Point point_a;

  // Loop through all verticies
  for (int vertex_id = 0; vertex_id < int(states.size()); ++vertex_id)
  {
    // First point
    point_a = visuals_->convertPoint(stateToPoint(states[vertex_id]));

    // Add the point pair to the line message
    marker.points.push_back(point_a);
    marker.colors.push_back(color);
  }

  // Send to Rviz
  return visuals_->publishMarker(marker);
}

// Deprecated
bool ProjectionVizWindow::publishPath(const og::PathGeometric& path, const rvt::colors& color, const double thickness,
                                  const std::string& ns)
{
  return publish2DPath(path, color, thickness, ns);
}

bool ProjectionVizWindow::publish2DPath(const og::PathGeometric& path, const rvt::colors& color, const double thickness,
                                    const std::string& ns)
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
    visuals_->publishCylinder(prev_vertex, this_vertex, color, thickness, ns);

    // Save these coordinates for next line
    prev_vertex = this_vertex;
  }

  return true;
}

int ProjectionVizWindow::natRound(double x)
{
  return static_cast<int>(floor(x + 0.5f));
}

bool ProjectionVizWindow::publishState(const ob::State* state, const rvt::colors& color, const rvt::scales scale,
                                   const std::string& ns)
{
  return visuals_->publishSphere(stateToPoint(state), color, scale, ns);
}

bool ProjectionVizWindow::publishState(const ob::State* state, const rvt::colors& color, const double scale,
                                   const std::string& ns)
{
  return visuals_->publishSphere(stateToPoint(state), color, scale, ns);
}

bool ProjectionVizWindow::publishState(const ob::ScopedState<> state, const rvt::colors& color, const rvt::scales scale,
                                   const std::string& ns)
{
  return visuals_->publishSphere(stateToPoint(state), color, scale, ns);
}

bool ProjectionVizWindow::publishState(const ob::ScopedState<> state, const rvt::colors& color, double scale,
                                   const std::string& ns)
{
  return visuals_->publishSphere(stateToPoint(state), color, scale, ns);
}

bool ProjectionVizWindow::publishState(const ob::ScopedState<> state, const rvt::colors& color,
                                   const geometry_msgs::Vector3& scale, const std::string& ns)
{
  return visuals_->publishSphere(stateToPoint(state), color, scale.x, ns);
}

bool ProjectionVizWindow::publishSampleRegion(const ob::ScopedState<>& state_area, const double& distance)
{
  temp_point_.x = state_area[0];
  temp_point_.y = state_area[1];
  temp_point_.z = state_area[2];

  visuals_->publishSphere(temp_point_, rvt::BLACK, rvt::REGULAR, "sample_region");  // mid point
  // outer sphere (x2 b/c its a radius, x0.1 to make it look nicer)
  return visuals_->publishSphere(temp_point_, rvt::TRANSLUCENT, rvt::REGULAR, "sample_region");
}

Eigen::Vector3d ProjectionVizWindow::stateToPoint(const ob::ScopedState<> state)
{
  return stateToPoint(state.get());
}

Eigen::Vector3d ProjectionVizWindow::stateToPoint(const ob::State* state)
{
  if (!state)
  {
    ROS_FATAL_NAMED(name_, "No state found for vertex");
    exit(1);
  }

  // Copy point
  temp_eigen_point_.x() = state->as<moveit_ompl::ModelBasedStateSpace::StateType>()->values[0];
  temp_eigen_point_.y() = state->as<moveit_ompl::ModelBasedStateSpace::StateType>()->values[1];
  if (si_->getStateSpace()->getDimension() == 3)
    temp_eigen_point_.z() = state->as<moveit_ompl::ModelBasedStateSpace::StateType>()->values[2];

  // Move point to useful location
  projectPoint(temp_eigen_point_);

  return temp_eigen_point_;
}

void ProjectionVizWindow::projectPoint(Eigen::Vector3d &point)
{
  // Visualize within a 1:1:1 3D space
  point.x() = point.x() / range0_;
  point.y() = point.y() / range1_;
  if (si_->getStateSpace()->getDimension() == 3) // avoid divide by zero
    point.z() = point.z() / range2_;

  // Move all points to a hard coded location
  point.x() += 1.5;
  point.y() += 0.5;

  // TODO(davetcoleman):remove
  // std::cout << "point.x(): " << point.x() << std::endl;
  // std::cout << "point.y(): " << point.y() << std::endl;
  // std::cout << "point.z(): " << point.z() << std::endl;
}

}  // namespace ompl_visual_tools
