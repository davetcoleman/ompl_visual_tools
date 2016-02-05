/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
 *  Copyright (c) 2012, Willow Garage, Inc.
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
 *   * Neither the name of the Willow Garage nor the names of its
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

/*
  Author: Dave Coleman <dave@dav.ee>
  Desc:   Visualize planning with OMPL in Rviz
*/

// ROS
#include <ros/ros.h>
#include <ros/package.h> // for getting file path for loading images

// Display in Rviz tool
#include <ompl_visual_tools/ompl_visual_tools.h>
#include <ompl_visual_tools/costs/two_dimensional_validity_checker.h>

// OMPL
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

// Boost
#include <boost/pointer_cast.hpp>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ompl_visual_tools
{

static const std::string BASE_FRAME = "/world";

/**
 * \brief SimpleSetup demo class
 */
class RRTDemo
{

private:

  og::SimpleSetupPtr simple_setup_;

  // Cost in 2D
  ompl::base::CostMap2DOptimizationObjectivePtr cost_map_;

  // Cost is just path length
  boost::shared_ptr<ompl::base::PathLengthOptimizationObjective> path_length_objective_;

  // The visual tools for interfacing with Rviz
  ompl_visual_tools::OmplVisualToolsPtr visual_tools_;

  // The number of dimensions - always 2 for images
  static const unsigned int DIMENSIONS = 2;

  // The state space we plan in
  ob::StateSpacePtr space_;

  // Remember what space we are working in
  ompl::base::SpaceInformationPtr si_;

  // Flag for determining amount of debug output to show
  bool verbose_;

  // Display graphics in Rviz
  bool use_visuals_;

public:

  /**
   * \brief Constructor
   */
  RRTDemo(bool verbose, bool use_visuals)
    : verbose_(verbose),
      use_visuals_(use_visuals)
  {
    // Construct the state space we are planning in
    space_.reset( new ob::RealVectorStateSpace( DIMENSIONS ));

    // Define an experience setup class
    simple_setup_ = og::SimpleSetupPtr( new og::SimpleSetup(space_) );
    si_ = simple_setup_->getSpaceInformation();

    // Load the tool for displaying in Rviz
    visual_tools_.reset(new ompl_visual_tools::OmplVisualTools(BASE_FRAME));
    visual_tools_->setSpaceInformation(si_);
    visual_tools_->setGlobalScale(100);

    // Set the planner
    //simple_setup_->setPlanner(ob::PlannerPtr(new og::RRTstar( si_ )));
    simple_setup_->setPlanner(ob::PlannerPtr(new og::RRT( si_ )));

    // Load the cost map
    cost_map_.reset(new ompl::base::CostMap2DOptimizationObjective( si_ ));

    // Load an alternitive optimization objective
    path_length_objective_.reset(new ompl::base::PathLengthOptimizationObjective( si_ ));
  }

  /**
   * \brief Deconstructor
   */
  ~RRTDemo()
  {
  }

  /**
   * \brief Clear all markers displayed in Rviz
   */
  void deleteAllMakers()
  {
    // Clear current rviz makers
    visual_tools_->deleteAllMarkers();
  }

  /**
   * \brief Load cost map from file
   * \param file path
   * \param how much of the peaks of the mountains are considered obstacles
   */
  void loadCostMapImage( std::string image_path, double max_cost_threshold_percent = 0.4 )
  {
    cost_map_->max_cost_threshold_percent_ = max_cost_threshold_percent;
    cost_map_->loadImage(image_path);

    // Set the bounds for the R^2
    ob::RealVectorBounds bounds( DIMENSIONS );
    bounds.setLow( 0 ); // both dimensions start at 0
    bounds.setHigh( 0, cost_map_->image_->x - 1 ); // allow for non-square images
    bounds.setHigh( 1, cost_map_->image_->y - 1 ); // allow for non-square images
    space_->as<ob::RealVectorStateSpace>()->setBounds( bounds );
    space_->setup();

    // Pass cost to visualizer
    visual_tools_->setCostMap(cost_map_->cost_);

    // Set state validity checking for this space
    simple_setup_->setStateValidityChecker( ob::StateValidityCheckerPtr(
      new ob::TwoDimensionalValidityChecker( si_, cost_map_->cost_, cost_map_->max_cost_threshold_ ) ) );

    // The interval in which obstacles are checked for between states
    // seems that it default to 0.01 but doesn't do a good job at that level
    si_->setStateValidityCheckingResolution(0.001);

    // Setup the optimization objective to use the 2d cost map
    // NEWER VERSION OF OMPL
    //simple_setup_->setOptimizationObjective(cost_map_);
    //simple_setup_->setOptimizationObjective(path_length_objective_);

    // Setup -----------------------------------------------------------

    // Auto setup parameters
    simple_setup_->setup(); // optional

    //ROS_ERROR_STREAM_NAMED("temp","out of curiosity: coll check resolution: "
    //   << si_->getStateValidityCheckingResolution());

    // Debug - this call is optional, but we put it in to get more output information
    //simple_setup_->print();
  }

  void publishCostMapImage()
  {
    if (use_visuals_)
      visual_tools_->publishCostMap(cost_map_->image_);
  }

  /**
   * \brief Solve a planning proble that we randomly make up
   * \param run_id - which run this is
   * \param runs - how many total runs we will do
   * \return true on success
   */
  bool plan(const int& run_id, const int& runs)
  {
    // Start and Goal State ---------------------------------------------
    ob::PlannerStatus solved;

    // Clear all planning data. This only includes data generated by motion plan computation.
    // Planner settings, start & goal states are not affected.
    if (run_id) // skip first run
      simple_setup_->clear();

    // Clear the previous solutions
    //simple_setup_->getProblemDefinition()->clearSolutionPaths();

    // Create the termination condition
    double seconds = 2;
    ob::PlannerTerminationCondition ptc = ob::timedPlannerTerminationCondition( seconds, 0.1 );

    // Create start and goal space
    ob::ScopedState<> start(space_);
    ob::ScopedState<> goal(space_);
    chooseStartGoal(start, goal);

    // Show start and goal
    if (use_visuals_)
    {
      visual_tools_->publishState(start, rviz_visual_tools::GREEN,  rviz_visual_tools::XLARGE, "plan_start_goal");
      visual_tools_->publishState(goal,  rviz_visual_tools::ORANGE, rviz_visual_tools::XLARGE, "plan_start_goal");
    }

    // set the start and goal states
    simple_setup_->setStartAndGoalStates(start, goal);

    // Solve -----------------------------------------------------------

    // attempt to solve the problem within x seconds of planning time
    solved = simple_setup_->solve( ptc );

    geometry_msgs::Pose text_pose;
    text_pose.position.x = cost_map_->cost_->size1()/2.0;
    text_pose.position.y = cost_map_->cost_->size1()/-20.0;
    text_pose.position.z = cost_map_->cost_->size1()/10.0;

    if (solved)
    {
      if (!simple_setup_->haveExactSolutionPath())
      {
        ROS_WARN_STREAM_NAMED("plan","APPROXIMATE solution found");
        if (use_visuals_)
          visual_tools_->publishText(text_pose, "APPROXIMATE solution found");
      }
      else
      {
        ROS_DEBUG_STREAM_NAMED("plan","Exact solution found");
        if (use_visuals_)
          visual_tools_->publishText(text_pose, "Exact solution found");

      }

      if (use_visuals_)
      {
        if (runs == 1)
        {
          // display all the aspects of the solution
          publishPlannerData(false);
        }
        else
        {
          // only display the paths
          publishPlannerData(false);
        }
      }
    }
    else
    {
      ROS_ERROR("No Solution Found");
      if (use_visuals_)
        visual_tools_->publishText(text_pose, "No Solution Found");
    }

    return solved;
  }

  void chooseStartGoal(ob::ScopedState<>& start, ob::ScopedState<>& goal)
  {
    if( true ) // choose completely random state
    {
      findValidState(start);
      findValidState(goal);
    }
    else if (false) // Manually set the start location
    {
      // Plan from scrach location
      start[0] = 5;  start[1] = 5;
      goal[0] = 5;  goal[1] = 45;
      // Recall location
      start[0] = 45;  start[1] = 5;
      goal[0] = 45;  goal[1] = 45;
    }
    else // Randomly sample around two states
    {
      ROS_INFO_STREAM_NAMED("temp","Sampling start and goal around two center points");

      ob::ScopedState<> start_area(space_);
      start_area[0] = 100;
      start_area[1] = 80;

      ob::ScopedState<> goal_area(space_);
      goal_area[0] = 330;
      goal_area[1] = 350;

      // Check these hard coded values against varying image sizes
      if (!space_->satisfiesBounds(start_area.get()) || !space_->satisfiesBounds(goal_area.get()))
      {
        ROS_ERROR_STREAM_NAMED("chooseStartGoal:","State does not satisfy bounds");
        exit(-1);
        return;
      }

      // Choose the distance to sample around
      double maxExtent = si_->getMaximumExtent();
      double distance = maxExtent * 0.1;
      ROS_INFO_STREAM_NAMED("temp","Distance is " << distance << " from max extent " << maxExtent);

      // Publish the same points
      if (true)
      {
        findValidState(start.get(), start_area.get(), distance);
        findValidState(goal.get(), goal_area.get(), distance);
      }
      else
      {
        //                start[0] = 277.33;  start[1] = 168.491;
        //                 goal[0] = 843.296;  goal[1] = 854.184;
      }

      // Show the sample regions
      if (use_visuals_ && false)
      {
        visual_tools_->publishSampleRegion(start_area, distance);
        visual_tools_->publishSampleRegion(goal_area, distance);
      }
    }

    // Print the start and goal
    //ROS_DEBUG_STREAM_NAMED("chooseStartGoal","Start: " << start);
    //ROS_DEBUG_STREAM_NAMED("chooseStartGoal","Goal: " << goal);
  }

  void findValidState(ob::ScopedState<>& state)
  {
    std::size_t rounds = 0;
    while (rounds < 100)
    {
      state.random();

      // Check if the sampled points are valid
      if( si_->isValid(state.get()) )
      {
        return;
      }
      ++rounds;
    }
    ROS_ERROR_STREAM_NAMED("findValidState","Unable to find valid start/goal state after " << rounds << " rounds");
  }

  void findValidState(ob::State *state, const ob::State *near, const double distance)
  {
    // Create sampler
    ob::StateSamplerPtr sampler = si_->allocStateSampler();

    while (true)
    {
      sampler->sampleUniformNear(state, near, distance); // samples (near + distance, near - distance)

      // Check if the sampled points are valid
      if( si_->isValid(state) )
      {
        return;
      }
      else
        ROS_INFO_STREAM_NAMED("temp","Searching for valid start/goal state");
    }
  }

  /**
   * \brief Show the planner data in Rviz
   * \param just_path - if true, do not display the search tree/graph or the samples
   */
  void publishPlannerData(bool just_path)
  {
    // Final Solution  ----------------------------------------------------------------

    if (true)
    {
      // Show basic solution
      //visual_tools_->publishPath( simple_setup_->getSolutionPath(), RED);

      // Simplify solution
      //simple_setup_->simplifySolution();

      // Interpolate solution
      simple_setup_->getSolutionPath().interpolate();

      // Show path
      visual_tools_->publishPath( simple_setup_->getSolutionPath(), rviz_visual_tools::GREEN, 1.0, "final_solution");
    }

    // Print the states to screen -----------------------------------------------------
    if (false)
    {
      ROS_DEBUG_STREAM_NAMED("temp","showing path");
      simple_setup_->getSolutionPath().print(std::cout);
    }

    // Get information about the exploration data structure the motion planner used in planning
    const ob::PlannerDataPtr planner_data( new ob::PlannerData( si_ ) );
    simple_setup_->getPlannerData( *planner_data );

    // Optionally display the search tree/graph or the samples
    if (!just_path)
    {
      // Visualize the explored space
      visual_tools_->publishGraph(planner_data, rviz_visual_tools::ORANGE, 0.2, "tree");

      // Visualize the sample locations
      //visual_tools_->publishSamples(planner_data);
    }

  }

  /** \brief Allow access to simple_setup framework */
  og::SimpleSetupPtr getSimpleSetup()
  {
    return simple_setup_;
  }


}; // end of class

} // namespace

// *********************************************************************************************************
// Main
// *********************************************************************************************************
int main( int argc, char** argv )
{
  ros::init(argc, argv, "ompl_visual_tools");
  ROS_INFO( "OMPL Visual Tools Demo ----------------------------------------- " );

  // Seed random
  srand ( time(NULL) );

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Default argument values
  bool verbose = false;
  bool use_visuals = true;
  std::string image_path;
  int runs = 1;

  if (argc > 1)
  {
    for (std::size_t i = 0; i < argc; ++i)
    {
      // Help mode
      if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0)
      {
        ROS_INFO_STREAM_NAMED("main","Usage: ompl_rviz_demos"
                              << " --verbose --noVisuals --image [image_file] --runs [num plans] "
                              << " -h --help");
        return 0;
      }

      // Check for verbose flag
      if (strcmp(argv[i], "--verbose") == 0)
      {
        ROS_INFO_STREAM_NAMED("main","Running in VERBOSE mode (slower)");
        verbose = true;
      }

      // Check if we should publish markers
      if (strcmp(argv[i], "--noVisuals") == 0)
      {
        ROS_INFO_STREAM_NAMED("main","NOT displaying graphics");
        use_visuals = false;
      }

      // Check if user has passed in an image to read
      if (strcmp(argv[i], "--image") == 0)
      {
        i++; // get next arg
        image_path = argv[i];
      }

      // Check if user has passed in the number of runs to perform
      if (strcmp(argv[i], "--runs") == 0)
      {
        i++; // get next arg
        runs = atoi( argv[i] );
      }
    }
  }

  // Provide image if necessary
  if (image_path.empty()) // use default image
  {
    // Get image path based on package name
    image_path = ros::package::getPath("ompl_visual_tools");
    if( image_path.empty() )
    {
      ROS_ERROR( "Unable to get OMPL Visual Tools package path " );
      return false;
    }

    // Choose random image
    int rand_num = ompl_visual_tools::OmplVisualTools::dRand(0,2);
    switch( rand_num )
    {
      case 0:
        image_path.append( "/resources/wilbur_medium/wilbur_medium1.ppm" );
        break;
      case 1:
        image_path.append( "/resources/wilbur_medium/wilbur_medium2.ppm" );
        break;
      default:
        ROS_ERROR_STREAM_NAMED("main","Random has no case " << rand_num);
        break;
    }
  }

  // Create the planner
  ompl_visual_tools::RRTDemo demo(verbose, use_visuals);

  // Clear Rviz
  demo.deleteAllMakers();

  // Load an image
  ROS_INFO_STREAM_NAMED("main","Loading image " << image_path);
  demo.loadCostMapImage( image_path, 0.4 );
  demo.publishCostMapImage();

  // Run the demo the desired number of times
  for (std::size_t i = 0; i < runs; ++i)
  {
    // Check if user wants to shutdown
    if (!ros::ok())
    {
      ROS_WARN_STREAM_NAMED("plan","Terminating early");
      break;
    }
    ROS_INFO_STREAM_NAMED("plan","Planning " << i+1 << " out of " << runs << " ------------------------------------");

    // Refresh visuals
    if (use_visuals && i > 0)
    {
      demo.publishCostMapImage();
      ros::spinOnce();
    }

    // Run the planner
    demo.plan( i, runs );

    // Create a pause if we are doing more runs and this is not the last run
    if (runs > 1 && i < runs - 1 && use_visuals)
    {
      // Let publisher publish
      ros::spinOnce();
      ros::Duration(5.0).sleep();
    }

    // Clear markers if this is not our last run
    if (i < runs - 1 && use_visuals)
      demo.deleteAllMakers();
  }

  // Wait to let anything still being published finish
  ros::Duration(0.1).sleep();

  ROS_INFO_STREAM("Shutting down.");

  return 0;
}
