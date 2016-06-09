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
//#include <ompl_visual_tools/ompl_visual_tools.h>
#include <ompl/tools/debug/VizWindow.h>

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

// OMPL
#include <ompl/base/SpaceInformation.h>
#include <ompl/tools/debug/Visualizer.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/ScopedState.h>

// Custom validity checker that accounts for cost
#include <ompl_visual_tools/costs/cost_map_2d_optimization_objective.h>

// Visualization
#include <moveit_visual_tools/moveit_visual_tools.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace bnu = boost::numeric::ublas;
namespace rvt = rviz_visual_tools;

namespace ompl_interface
{
class ModelBasedPlanningContext;
typedef boost::shared_ptr<ModelBasedPlanningContext> ModelBasedPlanningContextPtr;
}

namespace ompl_visual_tools
{

// Default constants
static const std::string RVIZ_MARKER_TOPIC = "/ompl_rviz_markers";
static const double COST_HEIGHT_OFFSET = 0.5;

typedef std::map<std::string, std::list<std::size_t> > MarkerList;

class ROSVizWindow : public ompl::tools::VizWindow
{
public:
  ROSVizWindow(rviz_visual_tools::RvizVisualToolsPtr visuals, ompl::base::SpaceInformationPtr si);

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

  /** \brief Get the underlying visualizer */
  rviz_visual_tools::RvizVisualToolsPtr getVisualTools()
  {
    return visuals_;
  }

  // From ompl_visual_tools ------------------------------------------------------

  /**
   * \brief Optional cost map for 2D environments
   */
  void setCostMap(intMatrixPtr cost);

  /**
   * \brief Helper function for converting a point to the correct cost
   */
  double getCost(const geometry_msgs::Point& point);

  /**
   * \brief Visualize Results
   */
  bool publishCostMap(PPMImage* image, bool static_id = true);

  /**
   * \brief Helper Function to display triangles
   */
  bool publishTriangle(int x, int y, visualization_msgs::Marker* marker, PPMImage* image);

  /**
   * \brief Publish a marker of a series of spheres to rviz
   * \param spheres - where to publish them
   * \param color - an enum pre-defined name of a color
   * \param scale - an enum pre-defined name of a size
   * \param ns - namespace of marker
   * \return true on success
   */
  bool publishSpheres(const og::PathGeometric& path, const rviz_visual_tools::colors& color = rviz_visual_tools::RED,
                      double scale = 0.1, const std::string& ns = "path_spheres");
  bool publishSpheres(const og::PathGeometric& path, const rviz_visual_tools::colors& color = rviz_visual_tools::RED,
                      const rviz_visual_tools::scales scale = rviz_visual_tools::SMALL,
                      const std::string& ns = "path_spheres");
  bool publishSpheres(const og::PathGeometric& path, const rviz_visual_tools::colors& color,
                      const geometry_msgs::Vector3& scale, const std::string& ns = "path_spheres");

  /**
   * \brief Display a connection between two states as a straight line
   */
  bool publishEdge(const ob::State* stateA, const ob::State* stateB, const std_msgs::ColorRGBA& color,
                   const double radius = 0.05);

  /**
   * \brief Display States
   * \return true on success
   */
  bool publishStates(std::vector<const ompl::base::State*> states);

  /**
   * \brief Display result path from a solver
   * \return true on success
   */
  RVIZ_VISUAL_TOOLS_DEPRECATED
  bool publishPath(const og::PathGeometric& path, const rviz_visual_tools::colors& color, const double thickness = 0.4,
                   const std::string& ns = "result_path");

  bool publish2DPath(const og::PathGeometric& path, const rviz_visual_tools::colors& color,
                     const double thickness = 0.4, const std::string& ns = "result_path");

  /**
   * \brief Helper Function: gets the x,y coordinates for a given vertex id
   * \param id of a vertex
   * \param result from an OMPL planner
   * \return geometry point msg with no z value filled in
   */
  //Eigen::Vector3d stateToPoint(std::size_t vertex_id, ob::PlannerDataPtr planner_data);
  Eigen::Vector3d stateToPoint(const ob::ScopedState<> state);
  Eigen::Vector3d stateToPoint(const ob::State* state);

  /**
   * \brief Nat_Rounding helper function to make readings from cost map more accurate
   * \param double
   * \return rounded down number
   */
  static int natRound(double x);

  /**
   * \brief Display the start and goal states on the image map
   * \param start state
   * \param color
   */
  bool publishState(const ob::State* state, const rviz_visual_tools::colors& color,
                    const rviz_visual_tools::scales scale = rviz_visual_tools::REGULAR,
                    const std::string& ns = "state_sphere");
  bool publishState(const ob::State* state, const rviz_visual_tools::colors& color, const double scale = 0.1,
                    const std::string& ns = "state_sphere");
  bool publishState(const ob::ScopedState<> state, const rviz_visual_tools::colors& color,
                    const rviz_visual_tools::scales scale = rviz_visual_tools::REGULAR,
                    const std::string& ns = "state_sphere");
  bool publishState(const ob::ScopedState<> state, const rviz_visual_tools::colors& color, double scale = 0.1,
                    const std::string& ns = "state_sphere");
  bool publishState(const ob::ScopedState<> state, const rviz_visual_tools::colors& color,
                    const geometry_msgs::Vector3& scale, const std::string& ns = "state_sphere");

  /**
   * \brief Visualize the sampling area in Rviz
   * \param state_area - the center point of the uniform sampler
   * \param distance - the radius around the center for sampling
   */
  bool publishSampleRegion(const ob::ScopedState<>& state_area, const double& distance);

  /**
   * \brief Set the range to visualize the edge costs
   * \param invert - if true, red is largest values and green is lowest
   */
  void setMinMaxEdgeCost(const double& min_edge_cost, const double& max_edge_cost, bool invert = false)
  {
    min_edge_cost_ = min_edge_cost;
    max_edge_cost_ = max_edge_cost;
    invert_edge_cost_ = invert;
  }

  void setMinMaxEdgeRadius(const double& min_edge_radius, const double& max_edge_radius)
  {
    min_edge_radius_ = min_edge_radius;
    max_edge_radius_ = max_edge_radius;
  }

  void setMinMaxStateRadius(const double& min_state_radius, const double& max_state_radius)
  {
    min_state_radius_ = min_state_radius;
    max_state_radius_ = max_state_radius;
  }

  /**
   * \brief An OMPL planner calls this function directly through boost::bind to display its graph's progress during
   * search
   * \param pointer to the planner, to be used for getPlannerData()
   */
  void vizTrigger();
  void vizState(const ompl::base::State* state, ompl::tools::VizSizes type, ompl::tools::VizColors color, double extra_data = 0);
  void vizState2D(const Eigen::Vector3d& point, ompl::tools::VizSizes type, ompl::tools::VizColors color, double extra_data = 0);

  /**
   * \brief Publish a line from state A to state B
   * \param value how red->green the line should be, where [0,1] 0 is red
   * \return true on success
   */
  void vizEdge(const ompl::base::State* stateA, const ompl::base::State* stateB, double value);

  /**
   * \brief Publish a full path of multiple points and edges
   * \param path
   * \param type - the style to display the line as
   * \return true on success
   */
  void vizPath(const ompl::geometric::PathGeometric* path, std::size_t type, ompl::tools::VizColors color);

  /** \brief Convert a number to an rviz_visual_tools color enum */
  rviz_visual_tools::colors omplColorToRviz(std::size_t color);

  /** \brief Getter for SpaceInformation */
  const ompl::base::SpaceInformationPtr& getSpaceInformation() const
  {
    return si_;
  }

private:
  /** \brief Short name of class */
  std::string name_;

  /** \brief Rviz visualization tools */
  rviz_visual_tools::RvizVisualToolsPtr visuals_;

  /** \brief Remember what space we are working in */
  ompl::base::SpaceInformationPtr si_;

  // From ompl_visual_tools ------------------------------------------------------

  // Keep a pointer to an optional cost map
  intMatrixPtr cost_;

  // Cached Point object to reduce memory loading
  geometry_msgs::Point temp_point_;
  Eigen::Vector3d temp_eigen_point_;

  // Set bounds on an edge's cost/weight/value for visualization purposes
  double max_edge_cost_ = 100.0;
  double min_edge_cost_ = 0.0;
  bool invert_edge_cost_ = false;
  double max_edge_radius_ = 0.5;
  double min_edge_radius_ = 0.1;

  // Set bounds on an state's cost/weight/value for visualization purposes
  double max_state_radius_ = 0.1;
  double min_state_radius_ = 0.5;

  double level_scale_ = 20.0;

};

typedef std::shared_ptr<ROSVizWindow> ROSVizWindowPtr;
typedef std::shared_ptr<const ROSVizWindow> ROSVizWindowConstPtr;

}  // namespace ompl_visual_tools

#endif  // OMPL_VISUAL_TOOLS__ROS_VIZ_WINDOW_
