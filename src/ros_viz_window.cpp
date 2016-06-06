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

#include <ompl_visual_tools/ros_viz_window.h>

namespace ot = ompl::tools;
namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace bnu = boost::numeric::ublas;

namespace ompl_visual_tools
{
ROSVizWindow::ROSVizWindow(ompl_visual_tools::OmplVisualToolsPtr visual_tools)
  : name_("ros_viz_window"), visual_tools_(visual_tools), si_(visual_tools_->getSpaceInformation())
{
  ROS_DEBUG_STREAM_NAMED(name_, "Initializing ROSVizWindow()");
}

void ROSVizWindow::state(const ompl::base::State* state, ot::VizSizes size, ot::VizColors color, double extra_data)
{
  // Determine which StateSpace to work in
  if (si_->getStateSpace()->getDimension() <= 3)
    visual_tools_->vizState2D(visual_tools_->stateToPoint(state), size, color, extra_data);
  else
    visual_tools_->vizStateRobot(state, size, color, extra_data);
}

void ROSVizWindow::state(const ompl::base::State* state, std::size_t level, ot::VizSizes size, ot::VizColors color,
                         double extra_data)
{
  // Determine which StateSpace to work in
  if (si_->getStateSpace()->getDimension() == 2)
  {
    Eigen::Vector3d point = visual_tools_->stateToPoint(state);
    point.z() = level;
    visual_tools_->vizState2D(point, size, color, extra_data);
  }
  else
  {
    OMPL_WARN("visualizing state with level not supported in != 2D yet");
    visual_tools_->vizStateRobot(state, size, color, extra_data);
  }
}

void ROSVizWindow::edge(const ompl::base::State* stateA, const ompl::base::State* stateB, double cost)
{
  visual_tools_->vizEdge(stateA, stateB, cost);
}

void ROSVizWindow::edge(const ompl::base::State* stateA, std::size_t levelA, const ompl::base::State* stateB,
                        std::size_t levelB, ot::VizSizes size, ot::VizColors color)
{
  Eigen::Vector3d pointA = visual_tools_->stateToPoint(stateA);
  pointA.z() = levelA;

  Eigen::Vector3d pointB = visual_tools_->stateToPoint(stateB);
  pointB.z() = levelB;

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

  visual_tools_->publishCylinder(pointA, pointB, visual_tools_->intToColor(color), radius);
}

void ROSVizWindow::path(ompl::geometric::PathGeometric* path, std::size_t size, ot::VizColors color)
{
  visual_tools_->vizPath(path, size, color);
}

void ROSVizWindow::trigger()
{
  visual_tools_->vizTrigger();
}

void ROSVizWindow::deleteAllMarkers()
{
  visual_tools_->deleteAllMarkers();
}

}  // namespace ompl_visual_tools
