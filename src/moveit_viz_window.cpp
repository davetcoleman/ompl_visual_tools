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

#include <ompl_visual_tools/moveit_viz_window.h>

// ROS
#include <ros/ros.h>
#include <graph_msgs/GeometryGraph.h>
#include <graph_msgs/Edges.h>

// OMPL
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/ScopedState.h>

// For converting OMPL state to a MoveIt robot state
#include <moveit_ompl/model_based_state_space.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/macros/deprecation.h>

namespace ot = ompl::tools;
namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ompl_visual_tools
{
MoveItVizWindow::MoveItVizWindow(moveit_visual_tools::MoveItVisualToolsPtr visuals, ompl::base::SpaceInformationPtr si)
  : name_("moveit_viz_window"), visuals_(visuals), si_(si)
{
  // with this OMPL interface to Rviz all pubs must be manually triggered
  visuals_->enableBatchPublishing(false);

  ROS_DEBUG_STREAM_NAMED(name_, "Initializing MoveItVizWindow()");
}

void MoveItVizWindow::state(const ompl::base::State* state, ot::VizSizes size, ot::VizColors color, double extra_data)
{
  // We do not use stateToPoint() because the publishRobotState() function might need the robot state in this function
  visuals_->loadSharedRobotState();
  moveit_ompl::ModelBasedStateSpacePtr mb_state_space =
      std::static_pointer_cast<moveit_ompl::ModelBasedStateSpace>(si_->getStateSpace());
  // We must use the root_robot_state here so that the virtual_joint isn't affected
  mb_state_space->copyToRobotState(*visuals_->getRootRobotState(), state);
  Eigen::Affine3d pose = visuals_->getRootRobotState()->getGlobalLinkTransform("right_gripper_target");

  switch (size)
  {
    case ompl::tools::SMALL:
      visuals_->publishSphere(pose, omplColorToRviz(color), rvt::SMALL);
      break;
    case ompl::tools::MEDIUM:
      visuals_->publishSphere(pose, omplColorToRviz(color), rvt::REGULAR);
      break;
    case ompl::tools::LARGE:
      visuals_->publishSphere(pose, omplColorToRviz(color), rvt::LARGE);
      break;
    case ompl::tools::VARIABLE_SIZE:  // Medium purple, translucent outline
      visuals_->publishSphere(pose, rvt::PURPLE, rvt::REGULAR);
      // visuals_->publishSphere(pose.translation(), rvt::TRANSLUCENT_LIGHT, extra_data * 2);
      break;
    case ompl::tools::SCALE:  // Display sphere based on value between 0-100
    {
      const double percent = (extra_data - min_edge_cost_) / (max_edge_cost_ - min_edge_cost_);
      const double radius = ((max_state_radius_ - min_state_radius_) * percent + min_state_radius_);
      geometry_msgs::Vector3 scale;
      scale.x = radius;
      scale.y = radius;
      scale.z = radius;
      visuals_->publishSphere(pose, visuals_->getColorScale(percent), scale);
    }
    break;
    case ompl::tools::ROBOT:  // Show actual robot in custom color
      mb_state_space->copyToRobotState(*visuals_->getSharedRobotState(), state);
      visuals_->publishRobotState(visuals_->getSharedRobotState(), omplColorToRviz(color));
      break;
    default:
      ROS_ERROR_STREAM_NAMED(name_, "vizStateRobot: Invalid state type value");
  }  // end switch
}

void MoveItVizWindow::edge(const ompl::base::State* stateA, const ompl::base::State* stateB, double cost)
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

  const double radius = (max_edge_radius_ - min_edge_radius_) * percent + min_edge_radius_;

  if (false)
  {
    std::cout << "cost: " << cost << " min_edge_cost_: " << min_edge_cost_ << " max_edge_cost_: " << max_edge_cost_
              << " percent: " << percent << " radius: " << radius << std::endl;
    std::cout << "max edge r: " << max_edge_radius_ << " min edge r: " << min_edge_radius_ << std::endl;
  }

  visuals_->publishLine(stateToPoint(stateA), stateToPoint(stateB), visuals_->getColorScale(percent), radius / 2.0);
}

void MoveItVizWindow::edge(const ompl::base::State* stateA, const ompl::base::State* stateB, ot::VizSizes size,
                           ot::VizColors color)
{
  Eigen::Vector3d pointA = stateToPoint(stateA);
  Eigen::Vector3d pointB = stateToPoint(stateB);

  double radius;
  switch (size)
  {
    case ompl::tools::SMALL:
      radius = 0.0005;
      break;
    case ompl::tools::MEDIUM:
      radius = 0.005;
      break;
    case ompl::tools::LARGE:
      radius = 0.05;
      break;
    default:
      OMPL_ERROR("Unknown size");
      exit(-1);
  }

  visuals_->publishLine(pointA, pointB, omplColorToRviz(color), radius);
}

void MoveItVizWindow::path(ompl::geometric::PathGeometric* path, ompl::tools::VizSizes type, ot::VizColors color)
{
  // Convert
  const og::PathGeometric& geometric_path = *path;  // static_cast<og::PathGeometric&>(*path);

  switch (type)
  {
    case ompl::tools::SMALL:  // Basic line with vertiices
      publish2DPath(geometric_path, omplColorToRviz(color), min_edge_radius_);
      publishSpheres(geometric_path, omplColorToRviz(color), rvt::SMALL);
      break;
    case ompl::tools::MEDIUM:  // Basic line with vertiices
      publish2DPath(geometric_path, omplColorToRviz(color), max_edge_radius_ / 2.0);
      publishSpheres(geometric_path, omplColorToRviz(color), rvt::SMALL);
      break;
    case ompl::tools::LARGE:  // Basic line with vertiices
      publish2DPath(geometric_path, omplColorToRviz(color), max_edge_radius_);
      publishSpheres(geometric_path, omplColorToRviz(color), rvt::SMALL);
      break;
    case ompl::tools::ROBOT:  // Playback motion for real robot
      // Check that jmg_ was set
      if (!jmg_)
        ROS_ERROR_STREAM_NAMED(name_, "Joint model group has not been set");

      publishTrajectoryPath(geometric_path, jmg_, true /*wait_for_trajectory*/);
      break;
    default:
      ROS_ERROR_STREAM_NAMED(name_, "Invalid vizPath type value " << type);
  }
}

void MoveItVizWindow::trigger()
{
  vizTrigger();
}

void MoveItVizWindow::deleteAllMarkers()
{
  visuals_->deleteAllMarkers();
}

bool MoveItVizWindow::shutdownRequested()
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

bool MoveItVizWindow::publishSpheres(const og::PathGeometric& path, const rvt::colors& color, double scale,
                                     const std::string& ns)
{
  geometry_msgs::Vector3 scale_vector;
  scale_vector.x = scale;
  scale_vector.y = scale;
  scale_vector.z = scale;
  return publishSpheres(path, color, scale_vector, ns);
}

bool MoveItVizWindow::publishSpheres(const og::PathGeometric& path, const rvt::colors& color, const rvt::scales scale,
                                     const std::string& ns)
{
  return publishSpheres(path, color, visuals_->getScale(scale, false, 0.25), ns);
}

bool MoveItVizWindow::publishSpheres(const og::PathGeometric& path, const rvt::colors& color,
                                     const geometry_msgs::Vector3& scale, const std::string& ns)
{
  std::vector<geometry_msgs::Point> points;
  for (std::size_t i = 0; i < path.getStateCount(); ++i)
    points.push_back(visuals_->convertPoint(stateToPoint(path.getState(i))));

  return visuals_->publishSpheres(points, color, scale, ns);
}

bool MoveItVizWindow::publishStates(std::vector<const ompl::base::State*> states)
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

bool MoveItVizWindow::publishRobotState(const ompl::base::State* state)
{
  // Make sure a robot state is available
  visuals_->loadSharedRobotState();

  moveit_ompl::ModelBasedStateSpacePtr mb_state_space =
      std::static_pointer_cast<moveit_ompl::ModelBasedStateSpace>(si_->getStateSpace());

  // Convert to robot state
  mb_state_space->copyToRobotState(*visuals_->getSharedRobotState(), state);

  // Show the robot visualized in Rviz
  return visuals_->publishRobotState(visuals_->getSharedRobotState());
}

bool MoveItVizWindow::publishTrajectoryPath(const ompl::base::PlannerDataPtr& path, robot_model::JointModelGroup* jmg,
                                            const std::vector<const robot_model::LinkModel*>& tips,
                                            bool show_trajectory_animated)
{
  // Make sure a robot state is available
  visuals_->loadSharedRobotState();

  // Vars
  Eigen::Affine3d pose;
  std::vector<std::vector<geometry_msgs::Point> > paths_msgs(tips.size());  // each tip has its own path of points
  robot_trajectory::RobotTrajectoryPtr robot_trajectory;

  moveit_ompl::ModelBasedStateSpacePtr mb_state_space =
      std::static_pointer_cast<moveit_ompl::ModelBasedStateSpace>(si_->getStateSpace());

  // Optionally save the trajectory
  if (show_trajectory_animated)
  {
    robot_trajectory.reset(new robot_trajectory::RobotTrajectory(visuals_->getRobotModel(), jmg->getName()));
  }

  // Each state in the path
  for (std::size_t state_id = 0; state_id < path->numVertices(); ++state_id)
  {
    // Check if program is shutting down
    if (!ros::ok())
      return false;

    // Convert to robot state
    mb_state_space->copyToRobotState(*visuals_->getSharedRobotState(), path->getVertex(state_id).getState());
    ROS_WARN_STREAM_NAMED("temp", "updateStateWithFakeBase disabled");
    // visuals_->getSharedRobotState()->updateStateWithFakeBase();

    visuals_->publishRobotState(visuals_->getSharedRobotState());

    // Each tip in the robot state
    for (std::size_t tip_id = 0; tip_id < tips.size(); ++tip_id)
    {
      // Forward kinematics
      pose = visuals_->getSharedRobotState()->getGlobalLinkTransform(tips[tip_id]);

      // Optionally save the trajectory
      if (show_trajectory_animated)
      {
        robot_state::RobotState robot_state_copy = *visuals_->getSharedRobotState();
        robot_trajectory->addSuffixWayPoint(robot_state_copy, 0.01);  // 1 second interval
      }

      // Debug pose
      // std::cout << "Pose: " << state_id << " of link " << tips[tip_id]->getName() << ": \n" << pose.translation() <<
      // std::endl;

      paths_msgs[tip_id].push_back(visuals_->convertPose(pose).position);

      // Show goal state arrow
      if (state_id == path->numVertices() - 1)
      {
        visuals_->publishArrow(pose, rvt::BLACK);
      }
    }
  }  // for each state

  for (std::size_t tip_id = 0; tip_id < tips.size(); ++tip_id)
  {
    visuals_->publishPath(paths_msgs[tip_id], rvt::RAND, rvt::SMALL);
    ros::Duration(0.05).sleep();
    visuals_->publishSpheres(paths_msgs[tip_id], rvt::ORANGE, rvt::SMALL);
    ros::Duration(0.05).sleep();
  }

  // Debugging - Convert to trajectory
  if (show_trajectory_animated)
  {
    visuals_->publishTrajectoryPath(*robot_trajectory, true);
  }

  return true;
}

bool MoveItVizWindow::publishTrajectoryPath(const og::PathGeometric& path, const robot_model::JointModelGroup* jmg,
                                            const bool blocking)
{
  // Convert to MoveIt! format
  robot_trajectory::RobotTrajectoryPtr traj;
  double speed = 0.01;
  if (!convertPath(path, jmg, traj, speed))
  {
    return false;
  }

  return visuals_->publishTrajectoryPath(*traj, blocking);
}

// Deprecated
bool MoveItVizWindow::publishPath(const og::PathGeometric& path, const rvt::colors& color, const double thickness,
                                  const std::string& ns)
{
  return publish2DPath(path, color, thickness, ns);
}

bool MoveItVizWindow::publish2DPath(const og::PathGeometric& path, const rvt::colors& color, const double thickness,
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

Eigen::Vector3d MoveItVizWindow::stateToPoint(const ob::ScopedState<> state)
{
  return stateToPoint(state.get());
}

Eigen::Vector3d MoveItVizWindow::stateToPoint(const ob::State* state)
{
  if (!state)
  {
    ROS_FATAL_NAMED(name_, "No state found for vertex");
    exit(1);
  }

  // Make sure a robot state is available
  visuals_->loadSharedRobotState();

  moveit_ompl::ModelBasedStateSpacePtr mb_state_space =
      std::static_pointer_cast<moveit_ompl::ModelBasedStateSpace>(si_->getStateSpace());

  // Convert to robot state
  mb_state_space->copyToRobotState(*visuals_->getRootRobotState(), state);

  // Get pose
  // TODO(davetcoleman): do not hard code
  Eigen::Affine3d pose = visuals_->getRootRobotState()->getGlobalLinkTransform("right_gripper_target");

  return pose.translation();
}

bool MoveItVizWindow::publishState(const ob::State* state, const rvt::colors& color, const rvt::scales scale,
                                   const std::string& ns)
{
  return visuals_->publishSphere(stateToPoint(state), color, scale, ns);
}

bool MoveItVizWindow::publishState(const ob::State* state, const rvt::colors& color, const double scale,
                                   const std::string& ns)
{
  return visuals_->publishSphere(stateToPoint(state), color, scale, ns);
}

bool MoveItVizWindow::publishState(const ob::ScopedState<> state, const rvt::colors& color, const rvt::scales scale,
                                   const std::string& ns)
{
  return visuals_->publishSphere(stateToPoint(state), color, scale, ns);
}

bool MoveItVizWindow::publishState(const ob::ScopedState<> state, const rvt::colors& color, double scale,
                                   const std::string& ns)
{
  return visuals_->publishSphere(stateToPoint(state), color, scale, ns);
}

bool MoveItVizWindow::publishState(const ob::ScopedState<> state, const rvt::colors& color,
                                   const geometry_msgs::Vector3& scale, const std::string& ns)
{
  return visuals_->publishSphere(stateToPoint(state), color, scale.x, ns);
}

bool MoveItVizWindow::publishSampleRegion(const ob::ScopedState<>& state_area, const double& distance)
{
  temp_point_.x = state_area[0];
  temp_point_.y = state_area[1];
  temp_point_.z = state_area[2];

  visuals_->publishSphere(temp_point_, rvt::BLACK, rvt::REGULAR, "sample_region");  // mid point
  // outer sphere (x2 b/c its a radius, x0.1 to make it look nicer)
  return visuals_->publishSphere(temp_point_, rvt::TRANSLUCENT, rvt::REGULAR, "sample_region");
}

bool MoveItVizWindow::convertPath(const og::PathGeometric& path, const robot_model::JointModelGroup* jmg,
                                  robot_trajectory::RobotTrajectoryPtr& traj, double speed)
{
  // Error check
  if (path.getStateCount() <= 0)
  {
    ROS_WARN_STREAM_NAMED(name_, "No states found in path");
    return false;
  }

  // New trajectory
  traj.reset(new robot_trajectory::RobotTrajectory(visuals_->getRobotModel(), jmg));
  moveit::core::RobotState state(*visuals_->getSharedRobotState());  // TODO(davetcoleman):do i need to copy this?

  // Get correct type of space
  moveit_ompl::ModelBasedStateSpacePtr mb_state_space =
      std::static_pointer_cast<moveit_ompl::ModelBasedStateSpace>(si_->getStateSpace());

  // Convert solution to MoveIt! format, reversing the solution
  // for (std::size_t i = path.getStateCount(); i > 0; --i)
  for (std::size_t i = 0; i < path.getStateCount(); ++i)
  {
    // Convert format
    mb_state_space->copyToRobotState(state, path.getState(i));

    // Add to trajectory
    traj->addSuffixWayPoint(state, speed);
  }
  return true;
}

void MoveItVizWindow::vizTrigger()
{
  visuals_->triggerBatchPublish();

  // Kill OMPL
  // if (!ros::ok())
  // {
  //   std::cout << std::endl;
  //   std::cout << "-------------------------------------------------------" << std::endl;
  //   std::cout << "Shutting down process by request of ros::ok()" << std::endl;
  //   std::cout << "-------------------------------------------------------" << std::endl;
  //   exit(0);
  // }
}

rvt::colors MoveItVizWindow::omplColorToRviz(std::size_t color)
{
  // clang-format off
  switch (color)
  {
    case 0: return rviz_visual_tools::BLACK; break;
    case 1: return rviz_visual_tools::BROWN; break;
    case 2: return rviz_visual_tools::BLUE; break;
    case 3: return rviz_visual_tools::CYAN; break;
    case 4: return rviz_visual_tools::GREY; break;
    case 5: return rviz_visual_tools::DARK_GREY; break;
    case 6: return rviz_visual_tools::GREEN; break;
    case 7: return rviz_visual_tools::LIME_GREEN; break;
    case 8: return rviz_visual_tools::MAGENTA; break;
    case 9: return rviz_visual_tools::ORANGE; break;
    case 10: return rviz_visual_tools::PURPLE; break;
    case 11: return rviz_visual_tools::RED; break;
    case 12: return rviz_visual_tools::PINK; break;
    case 13: return rviz_visual_tools::WHITE; break;
    case 14: return rviz_visual_tools::YELLOW; break;
    case 15: return rviz_visual_tools::TRANSLUCENT; break;
    case 16: return rviz_visual_tools::TRANSLUCENT_LIGHT; break;
    case 17: return rviz_visual_tools::TRANSLUCENT_DARK; break;
    case 18: return rviz_visual_tools::RAND; break;
    case 19: return rviz_visual_tools::CLEAR; break;
    case 20: return rviz_visual_tools::DEFAULT; break;
  }
  // clang-format on
  return rviz_visual_tools::DEFAULT;
}

}  // namespace ompl_visual_tools
