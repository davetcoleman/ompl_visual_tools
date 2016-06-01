/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
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

#ifndef OMPL_VISUAL_TOOLS__OMPL_VISUAL_TOOLS_
#define OMPL_VISUAL_TOOLS__OMPL_VISUAL_TOOLS_

// ROS
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

// OMPL
#include <ompl/base/SpaceInformation.h>
#include <ompl/tools/debug/Visualizer.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/ScopedState.h>

// Custom validity checker that accounts for cost
#include <ompl_visual_tools/costs/cost_map_2d_optimization_objective.h>

// MoveIt
#include <moveit/robot_model/link_model.h>

// Visualization
#include <moveit_visual_tools/moveit_visual_tools.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace bnu = boost::numeric::ublas;

// Forward Declartions
// namespace ompl
// {
// namespace base
// {
// class PlannerData;
// typedef boost::shared_ptr<PlannerData> PlannerDataPtr;
// }
// }

namespace ompl_interface
{
class ModelBasedPlanningContext;
typedef boost::shared_ptr<ModelBasedPlanningContext> ModelBasedPlanningContextPtr;
}

// namespace moveit
//{
// namespace core
//{
// class LinkModel;
//{
//}

namespace ompl_visual_tools
{
// Default constants
static const std::string RVIZ_MARKER_TOPIC = "/ompl_rviz_markers";
static const double COST_HEIGHT_OFFSET = 0.5;

typedef std::map<std::string, std::list<std::size_t> > MarkerList;

class OmplVisualTools : public moveit_visual_tools::MoveItVisualTools
{
public:
  /**
   * \brief Constructor
   */
  OmplVisualTools(const std::string& base_link, const std::string& marker_topic = ompl_visual_tools::RVIZ_MARKER_TOPIC,
                  robot_model::RobotModelConstPtr robot_model = robot_model::RobotModelConstPtr());

  /**
   * \brief Load the OMPL state space or space information pointer
   */
  void setStateSpace(ompl::base::StateSpacePtr space);
  void setSpaceInformation(ompl::base::SpaceInformationPtr si);

  /**
   * \brief Make sure user has specified a space information
   * \return true if space information has been set
   */
  bool checkSpaceInformation();

  /**
   * \brief Optional cost map for 2D environments
   */
  void setCostMap(intMatrixPtr cost);

  /**
   * \brief Getter for disable 3d option in Rviz
   */
  bool getDisable3D();

  /**
   * \brief Setter for disable 3d option in Rviz
   */
  void setDisable3D(bool disable_3d);

  /**
   * \brief Helper function for converting a point to the correct cost
   */
  double getCost(const geometry_msgs::Point& point);

  /**
   * \brief Use bilinear interpolation, if necessary, to find the cost of a point between whole numbers
   *        From http://supercomputingblog.com/graphics/coding-bilinear-interpolation/
   */
  double getCostHeight(const geometry_msgs::Point& point);

  /**
   * \brief Visualize Results
   */
  bool publishCostMap(PPMImage* image, bool static_id = true);

  /**
   * \brief Helper Function to display triangles
   */
  bool publishTriangle(int x, int y, visualization_msgs::Marker* marker, PPMImage* image);

  /**
   * \brief Helper Function for Display Graph that makes the exploration lines follow the curvature of the map
   */
  bool interpolateLine(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2,
                       visualization_msgs::Marker* marker, const std_msgs::ColorRGBA color);

  /**
   * \brief Display Start Goal
   */
  bool publishStartGoalSpheres(ob::PlannerDataPtr planner_data, const std::string& ns);

  /**
   * \brief Display Explored Space
   */
  bool publishGraph(ob::PlannerDataPtr planner_data, const rviz_visual_tools::colors& color = rviz_visual_tools::BLUE,
                    const double thickness = 0.2, const std::string& ns = "space_exploration");

  /**
   * \brief Publish a marker of a series of spheres to rviz
   * \param spheres - where to publish them
   * \param color - an enum pre-defined name of a color
   * \param scale - an enum pre-defined name of a size
   * \param ns - namespace of marker
   * \return true on success
   */
  bool publishSpheres(const ob::PlannerDataPtr& planner_data,
                      const rviz_visual_tools::colors& color = rviz_visual_tools::RED,
                      const rviz_visual_tools::scales scale = rviz_visual_tools::SMALL,
                      const std::string& ns = "planner_data_spheres");
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
   * \brief Display labels on samples
   */
  bool publishSampleIDs(const og::PathGeometric& path, const rviz_visual_tools::colors& color = rviz_visual_tools::RED,
                        const rviz_visual_tools::scales scale = rviz_visual_tools::SMALL,
                        const std::string& ns = "sample_labels");

  /**
   * \brief Convert PlannerData to PathGeometric. Assume ordering of verticies is order of path
   * \param PlannerData
   * \param PathGeometric
   */
  void convertPlannerData(const ob::PlannerDataPtr planner_data, og::PathGeometric& path);

  /**
   * \brief Display States
   * \return true on success
   */
  bool publishStates(std::vector<const ompl::base::State*> states);

  /**
   * \brief Convert an OMPL state to a MoveIt! robot state and publish it
   * \param OMPL format of a robot state
   * \return true on success
   */
  bool publishRobotState(const ompl::base::State* state);

  /**
   * \brief Display resulting path from a solver, in the form of a planner_data
   *        where the list of states is also the order of the path. This uses MoveIt's robot state for inverse
   * kinematics
   * \return true on success
   */
  RVIZ_VISUAL_TOOLS_DEPRECATED
  bool publishRobotPath(const ompl::base::PlannerDataPtr& path, robot_model::JointModelGroup* jmg,
                        const std::vector<const robot_model::LinkModel*>& tips, bool show_trajectory_animated)
  {
    return publishTrajectoryPath(path, jmg, tips, show_trajectory_animated);
  }

  RVIZ_VISUAL_TOOLS_DEPRECATED
  bool publishRobotPath(const og::PathGeometric& path, const robot_model::JointModelGroup* jmg,
                        const bool blocking)
  {
    return publishTrajectoryPath(path, jmg, blocking);
  }

  bool publishTrajectoryPath(const ompl::base::PlannerDataPtr& path, robot_model::JointModelGroup* jmg,
                             const std::vector<const robot_model::LinkModel*>& tips, bool show_trajectory_animated);

  bool publishTrajectoryPath(const og::PathGeometric& path, const robot_model::JointModelGroup* jmg,
                             const bool blocking);

  /**
   * \brief Display resulting graph from a planner, in the form of a planner_data object
   *        This uses MoveIt's robot state for inverse kinematics
   * \return true on success
   */
  bool publishRobotGraph(const ompl::base::PlannerDataPtr& graph,
                         const std::vector<const robot_model::LinkModel*>& tips);
  /**
   * \brief Display result path from a solver, in the form of a planner_data
   * where the list of states is also the order of the path
   * \return true on success
   */
  RVIZ_VISUAL_TOOLS_DEPRECATED
  bool publishPath(const ob::PlannerDataPtr& planner_data, const rviz_visual_tools::colors& color,
                   const double thickness = 0.4, const std::string& ns = "result_path");

  /**
   * \brief Display result path from a solver
   * \return true on success
   */
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
  Eigen::Vector3d stateToPoint(std::size_t vertex_id, ob::PlannerDataPtr planner_data);
  Eigen::Vector3d stateToPoint(const ob::ScopedState<> state);
  Eigen::Vector3d stateToPoint(const ob::State* state);
  Eigen::Vector3d stateToPoint2D(const ob::State* state);
  Eigen::Vector3d stateToPointRobot(const ob::State* state);

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
   * \brief Publish text to rviz at a given location
   */
  // bool publishText(const geometry_msgs::Point& point, const std::string& text,
  //                  const rviz_visual_tools::colors& color = rviz_visual_tools::BLACK, bool static_id = true);

  // bool publishText(const geometry_msgs::Pose& pose, const std::string& text,
  //                  const rviz_visual_tools::colors& color = rviz_visual_tools::BLACK, bool static_id = true);

  /**
   * \brief Convet each vertex in a graph into a list of tip locations, as desired
   * \param input - description
   * \param input - description
   * \return
   */
  bool convertRobotStatesToTipPoints(const ompl::base::PlannerDataPtr& graph,
                                     const std::vector<const robot_model::LinkModel*>& tips,
                                     std::vector<std::vector<geometry_msgs::Point> >& vertex_tip_points);

  /** \brief Convert path formats */
  bool convertPath(const og::PathGeometric& path, const robot_model::JointModelGroup* jmg,
                   robot_trajectory::RobotTrajectoryPtr& traj, double speed = 0.1);

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

  /** \brief Print to console a state */
  void printState(ompl::base::State* state);

  /**
   * \brief An OMPL planner calls this function directly through boost::bind to display its graph's progress during
   * search
   * \param pointer to the planner, to be used for getPlannerData()
   */
  void vizTrigger();
  void vizState(const ompl::base::State* state, ompl::tools::sizes type, ompl::tools::colors color, double extra_data = 0);
  void vizStateRobot(const ompl::base::State* state, ompl::tools::sizes type, ompl::tools::colors color, double extra_data);
  void vizState2D(const Eigen::Vector3d& point, ompl::tools::sizes type, ompl::tools::colors color, double extra_data = 0);
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
  void vizPath(const ompl::base::PathPtr path, std::size_t type, ompl::tools::colors color);

  /**
   * \brief Helper to set an OMPL's planner to use the visualizer callback
   * \return a callback function
   */
  ompl::tools::VizTrigger getVizTriggerCallback()
  {
    return boost::bind(&OmplVisualTools::vizTrigger, this);
  }
  ompl::tools::VizState getVizStateCallback()
  {
    return boost::bind(&OmplVisualTools::vizState, this, _1, _2, _3, _4);
  }
  ompl::tools::VizEdge getVizEdgeCallback()
  {
    return boost::bind(&OmplVisualTools::vizEdge, this, _1, _2, _3);
  }
  ompl::tools::VizPath getVizPathCallback()
  {
    return boost::bind(&OmplVisualTools::vizPath, this, _1, _2, _3);
  }

  /** \brief Getter for JointModelGroup */
  const robot_model::JointModelGroup* getJointModelGroup() const
  {
    return jmg_;
  }

  /** \brief Setter for JointModelGroup */
  void setJointModelGroup(const robot_model::JointModelGroup* jmg)
  {
    jmg_ = jmg;
  }

  /** \brief Convert a number to an rviz_visual_tools color enum */
  rviz_visual_tools::colors intToColor(std::size_t color);

private:
  // Keep a pointer to an optional cost map
  intMatrixPtr cost_;

  // Remember what space we are working in
  ompl::base::SpaceInformationPtr si_;

  // Remember what joint model group we care about so that calls from OMPL don't have to
  const robot_model::JointModelGroup* jmg_;

  // Cached Point object to reduce memory loading
  geometry_msgs::Point temp_point_;
  Eigen::Vector3d temp_eigen_point_;

  // Mode that disables showing 3D in Rviz
  bool disable_3d_ = false;

  // Set bounds on an edge's cost/weight/value for visualization purposes
  double max_edge_cost_ = 100.0;
  double min_edge_cost_ = 0.0;
  bool invert_edge_cost_ = false;
  double max_edge_radius_ = 0.5;
  double min_edge_radius_ = 0.1;

  // Set bounds on an state's cost/weight/value for visualization purposes
  double max_state_radius_ = 0.1;
  double min_state_radius_ = 0.5;

};  // end class

typedef boost::shared_ptr<OmplVisualTools> OmplVisualToolsPtr;
typedef boost::shared_ptr<const OmplVisualTools> OmplVisualToolsConstPtr;

}  // end namespace

#endif
