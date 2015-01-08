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
#include <ompl/base/Planner.h>
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
namespace ompl
{
namespace base
{
class PlannerData;
typedef boost::shared_ptr<PlannerData> PlannerDataPtr;
}}

namespace ompl_interface
{
class ModelBasedPlanningContext;
typedef boost::shared_ptr<ModelBasedPlanningContext> ModelBasedPlanningContextPtr;
}

//namespace moveit
//{
//namespace core
//{
//class LinkModel;
//{
//}

namespace ompl_visual_tools
{

// Default constants
static const std::string RVIZ_MARKER_TOPIC = "/ompl_rviz_markers";
static const double COST_HEIGHT_OFFSET = 0.5;

typedef std::map< std::string, std::list<std::size_t> > MarkerList;

class OmplVisualTools : public moveit_visual_tools::MoveItVisualTools
{
private:

  // Keep a pointer to an optional cost map
  intMatrixPtr cost_;

  // Remember what space we are working in
  ompl::base::SpaceInformationPtr si_;

  // Cached Point object to reduce memory loading
  geometry_msgs::Point temp_point_;

  // Mode that disables showing 3D in Rviz
  bool disable_3d_;

public:

  /**
   * \brief Constructor
   */
  OmplVisualTools(const std::string& base_link,
                  const std::string& marker_topic = ompl_visual_tools::RVIZ_MARKER_TOPIC,
                  robot_model::RobotModelConstPtr robot_model = robot_model::RobotModelConstPtr());

  /**
   * \brief Load the OMPL state space or space information pointer
   */
  void setStateSpace(ompl::base::StateSpacePtr space);
  void setSpaceInformation(ompl::base::SpaceInformationPtr si);

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
  double getCost(const geometry_msgs::Point &point);

  /**
   * \brief Use bilinear interpolation, if necessary, to find the cost of a point between whole numbers
   *        From http://supercomputingblog.com/graphics/coding-bilinear-interpolation/
   */
  double getCostHeight(const geometry_msgs::Point &point);

  /**
   * \brief Visualize Results
   */
  bool publishCostMap(PPMImage *image, bool static_id = true);

  /**
   * \brief Helper Function to display triangles
   */
  bool publishTriangle( int x, int y, visualization_msgs::Marker* marker, PPMImage *image );

  /**
   * \brief Helper Function for Display Graph that makes the exploration lines follow the curvature of the map
   */
  bool interpolateLine( const geometry_msgs::Point &p1, const geometry_msgs::Point &p2, visualization_msgs::Marker* marker,
                        const std_msgs::ColorRGBA color );

  /**
   * \brief Display Start Goal
   */
  bool publishStartGoalSpheres(ob::PlannerDataPtr planner_data, const std::string& ns);

  /**
   * \brief Display Explored Space
   */
  bool publishGraph(ob::PlannerDataPtr planner_data, const rviz_visual_tools::colors& color = rviz_visual_tools::BLUE, const double thickness = 0.2,
                    const std::string& ns = "space_exploration");

  /**
   * \brief Display Sample Points
   */
  MOVEIT_DEPRECATED bool publishSamples( const ob::PlannerDataPtr& planner_data, const rviz_visual_tools::colors color = rviz_visual_tools::RED,
                                         const rviz_visual_tools::scales scale = rviz_visual_tools::SMALL, const std::string& ns = "sample_locations" );


  MOVEIT_DEPRECATED bool publishSamples( const og::PathGeometric& path, const rviz_visual_tools::colors color = rviz_visual_tools::RED,
                                         const rviz_visual_tools::scales scale = rviz_visual_tools::SMALL, const std::string& ns = "sample_locations" );

  /**
   * \brief Publish a marker of a series of spheres to rviz
   * \param spheres - where to publish them
   * \param color - an enum pre-defined name of a color
   * \param scale - an enum pre-defined name of a size
   * \param ns - namespace of marker
   * \return true on success
   */
  bool publishSpheres( const ob::PlannerDataPtr& planner_data, const rviz_visual_tools::colors color = rviz_visual_tools::RED, 
                       const rviz_visual_tools::scales scale = rviz_visual_tools::SMALL, const std::string& ns = "planner_data_spheres" );
  bool publishSpheres( const og::PathGeometric& path, const rviz_visual_tools::colors color = rviz_visual_tools::RED, 
                       double scale = 0.1, const std::string& ns = "path_spheres" );
  bool publishSpheres( const og::PathGeometric& path, const rviz_visual_tools::colors color = rviz_visual_tools::RED, 
                       const rviz_visual_tools::scales scale = rviz_visual_tools::SMALL, const std::string& ns = "path_spheres" );
  bool publishSpheres( const og::PathGeometric& path, const rviz_visual_tools::colors color, 
                       const geometry_msgs::Vector3 &scale, const std::string& ns = "path_spheres");

  /**
   * \brief 
   * \param input - description
   * \param input - description
   * \return 
   */
  bool publishEdge(const ob::State* stateA, const ob::State* stateB, const rviz_visual_tools::colors color = rviz_visual_tools::BLUE, 
                   const rviz_visual_tools::scales scale = rviz_visual_tools::REGULAR);

  /**
   * \brief Display labels on samples
   */
  bool publishSampleIDs( const og::PathGeometric& path, const rviz_visual_tools::colors color = rviz_visual_tools::RED,
                         const rviz_visual_tools::scales scale = rviz_visual_tools::SMALL, const std::string& ns = "sample_labels" );
  
  /**
   * \brief Convert PlannerData to PathGeometric. Assume ordering of verticies is order of path
   * \param PlannerData
   * \param PathGeometric
   */
  void convertPlannerData(const ob::PlannerDataPtr planner_data, og::PathGeometric &path);

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
  bool publishRobotState( const ompl::base::State *state );

  /**
   * \brief Display resulting path from a solver, in the form of a planner_data
   *        where the list of states is also the order of the path. This uses MoveIt's robot state for inverse kinematics
   * \return true on success
   */
  bool publishRobotPath( const ompl::base::PlannerDataPtr &path, robot_model::JointModelGroup* joint_model_group,
                         const std::vector<const robot_model::LinkModel*> &tips, bool show_trajectory_animated);

  /**
   * \brief Display resulting graph from a planner, in the form of a planner_data object
   *        This uses MoveIt's robot state for inverse kinematics
   * \return true on success
   */
  bool publishRobotGraph( const ompl::base::PlannerDataPtr &graph, 
                          const std::vector<const robot_model::LinkModel*> &tips);
  /**
   * \brief Display result path from a solver, in the form of a planner_data
   * where the list of states is also the order of the path
   * \return true on success
   */
  bool publishPath( const ob::PlannerDataPtr& planner_data, const rviz_visual_tools::colors color,
                    const double thickness = 0.4, const std::string& ns = "result_path" );

  /**
   * \brief Display result path from a solver
   * \return true on success
   */
  bool publishPath( const og::PathGeometric& path, const rviz_visual_tools::colors color, const double thickness = 0.4, const std::string& ns = "result_path" );

  /**
   * \brief Helper Function: gets the x,y coordinates for a given vertex id
   * \param id of a vertex
   * \param result from an OMPL planner
   * \return geometry point msg with no z value filled in
   */
  geometry_msgs::Point stateToPointMsg( int vertex_id, ob::PlannerDataPtr planner_data );

  geometry_msgs::Point stateToPointMsg( const ob::State *state );

  geometry_msgs::Point stateToPointMsg( const ob::ScopedState<> state );

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
  bool publishState(const ob::ScopedState<> state, const rviz_visual_tools::colors &color,
                    const rviz_visual_tools::scales scale = rviz_visual_tools::REGULAR,
                    const std::string& ns = "state_sphere");
  bool publishState(const ob::ScopedState<> state, const rviz_visual_tools::colors &color,
                    double scale = 0.1, const std::string& ns = "state_sphere");
  bool publishState(const ob::ScopedState<> state, const rviz_visual_tools::colors &color,
                    const geometry_msgs::Vector3 &scale, const std::string& ns = "state_sphere");
                   
  /**
   * \brief Visualize the sampling area in Rviz
   * \param state_area - the center point of the uniform sampler
   * \param distance - the radius around the center for sampling
   */
  bool publishSampleRegion(const ob::ScopedState<>& state_area, const double& distance);

  /**
   * \brief Publish text to rviz at a given location
   */
  bool publishText(const geometry_msgs::Point &point, const std::string &text,
                   const rviz_visual_tools::colors &color = rviz_visual_tools::BLACK,
                   bool static_id = true);

  bool publishText(const geometry_msgs::Pose &pose, const std::string &text, 
                   const rviz_visual_tools::colors &color = rviz_visual_tools::BLACK,
                   bool static_id = true);

  /**
   * \brief Convet each vertex in a graph into a list of tip locations, as desired
   * \param input - description
   * \param input - description
   * \return 
   */
  bool convertRobotStatesToTipPoints(const ompl::base::PlannerDataPtr &graph, 
                                     const std::vector<const robot_model::LinkModel*> &tips, 
                                     std::vector< std::vector<geometry_msgs::Point> > & vertex_tip_points);

  /**
   * \brief An OMPL planner calls this function directly through boost::bind to display its graph's progress during search
   * \param pointer to the planner, to be used for getPlannerData()
   *
  void visualizationCallback(ompl::base::Planner *planner);
  void visualizationStateCallback(ompl::base::State *state, std::size_t type, double neighborRadius);
  void visualizationEdgeCallback(ompl::base::State *stateA, ompl::base::State *stateB);

  **
   * \brief Helper to set an OMPL's planner to use the visualizer callback
   * \return a callback function
   *
  ompl::base::VisualizationCallback getVisualizationCallback();
  ompl::base::VisualizationStateCallback getVisualizationStateCallback();
  ompl::base::VisualizationEdgeCallback getVisualizationEdgeCallback();
  */

}; // end class

typedef boost::shared_ptr<OmplVisualTools> OmplVisualToolsPtr;
typedef boost::shared_ptr<const OmplVisualTools> OmplVisualToolsConstPtr;

} // end namespace

#endif
