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

#ifndef OMPL_VISUAL_TOOLS__ROS_VIZ_WINDOW_
#define OMPL_VISUAL_TOOLS__ROS_VIZ_WINDOW_

#include <ros/ros.h>
#include <ompl_visual_tools/ompl_visual_tools.h>
#include <ompl/tools/debug/VizWindow.h>

// namespace ob = ompl::base;
// namespace og = ompl::geometric;
// namespace bnu = boost::numeric::ublas;

namespace ompl_visual_tools
{
class ROSVizWindow : public ompl::tools::VizWindow
{
public:
  ROSVizWindow(ompl_visual_tools::OmplVisualToolsPtr visual_tools);

  /** \brief Visualize a state during runtime, externally */
  void state(const ompl::base::State* state, ompl::tools::VizSizes size, ompl::tools::VizColors color,
             double extraData);

  /** \brief Visualize edge during runtime, externally */
  void edge(const ompl::base::State* stateA, const ompl::base::State* stateB, double cost);

  /** \brief Visualize edge with a level during runtime, externally */
  void edge(const ompl::base::State* stateA, const ompl::base::State* stateB,
            ompl::tools::VizSizes size, ompl::tools::VizColors color);

  /** \brief Visualize path during runtime, externally */
  void path(ompl::geometric::PathGeometric* path, std::size_t size, ompl::tools::VizColors color);

  /** \brief Trigger visualizer to refresh/repaint/display all graphics */
  void trigger();

  /** \brief Trigger visualizer to clear all graphics */
  void deleteAllMarkers();

  /** \brief Check if SIGINT has been called */
  bool shutdownRequested();

private:
  /** \brief Short name of class */
  std::string name_;

  /** \brief Rviz visualization tools */
  ompl_visual_tools::OmplVisualToolsPtr visual_tools_;

  /** \brief Remember what space we are working in */
  ompl::base::SpaceInformationPtr si_;
};

typedef std::shared_ptr<ROSVizWindow> ROSVizWindowPtr;
typedef std::shared_ptr<const ROSVizWindow> ROSVizWindowConstPtr;

}  // namespace ompl_visual_tools

#endif  // OMPL_VISUAL_TOOLS__ROS_VIZ_WINDOW_
