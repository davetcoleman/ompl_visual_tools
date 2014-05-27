/*********************************************************************
 * Software License Agreement (BSD License)
 *
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

/* Author: Dave Coleman */

// ROS
#include <ros/ros.h>
#include <ros/package.h> // for getting file path for loading images

// For reading image files
#include <ompl_rviz_viewer/utilities/ppm.h>

// Display in Rviz tool
#include <ompl_rviz_viewer/ompl_rviz_viewer.h>

// OMPL planner
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/TRRT.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace bnu = boost::numeric::ublas;

namespace ompl_rviz_viewer
{

/**
 * \brief Custom State Validity Checker with cost function
 */
class TwoDimensionalValidityChecker : public ob::StateValidityChecker
{
private:
  bnu::matrix<int> cost_;
  double max_threshold_;

public:

  /** \brief Constructor */
  TwoDimensionalValidityChecker( const ob::SpaceInformationPtr& si, const bnu::matrix<int>& cost,
    double max_threshold ) :
    StateValidityChecker(si)
  {
    cost_ = cost;
    max_threshold_ = max_threshold;
  }

  /** \brief Obstacle checker */
  virtual bool isValid(const ob::State * state ) const
  {
    return cost(state) < max_threshold_;
  }

  virtual double cost(const ob::State *state) const
  {
    const double *coords = state->as<ob::RealVectorStateSpace::StateType>()->values;

    // Return the cost from the matrix at the current dimensions
    double cost = cost_( nat_round(coords[1]), nat_round(coords[0]) );

    return cost;
  }

};

/**
 * \brief SimpleSetup Planning Class
 */
class OmplRvizPlanner
{

 private:
  // The RGB image data
  PPMImage *image_;

  // The cost for each x,y - which is derived from the RGB data
  bnu::matrix<int> cost_;

  // The resulting graph that was searched
  ob::PlannerDataPtr planner_data_;

  // Save the simple setup until the program ends so that the planner data is not lost
  og::SimpleSetupPtr simple_setup_;

  // The visual tools for interfacing with Rviz
  ompl_rviz_viewer::OmplRvizViewerPtr viewer_;

  // Remember the min and max cost from the cost_ matrix for color visualization
  int max_cost_;
  int min_cost_;

  // The cost at which it becomes an obstacle
  double max_threshold_;

  // The percentage of the top min/max cost value that is considered an obstacle, e.g. 10 is top 10% of peaks
  //  static const double MAX_THRESHOLD_PERCENTAGE_ = 40;
  static const double MAX_THRESHOLD_PERCENTAGE_ = 1;

  // The number of dimensions - always 2 for images
  static const unsigned int DIMENSIONS = 2;

  // Use random goal and start locations
  static const bool USE_RANDOM_STATES = true;

 public:

  /**
   * \brief Constructor
   */
  OmplRvizPlanner()
  {
    image_ = NULL;

    bool verbose = true;
    viewer_.reset(new ompl_rviz_viewer::OmplRvizViewer(verbose));
  }

  /**
   * \brief Deconstructor
   */
  ~OmplRvizPlanner()
  {
    delete image_;
  }

  /**
   * \brief Main Function
   */
  void runImage( std::string image_path )
  {
    // Load cost map from image file
    image_ = readPPM( image_path.c_str() );

    // Error check
    if( !image_ )
    {
      ROS_ERROR( "No image data loaded " );
      return;
    }

    // Disallow non-square
    if( image_->x != image_->y )
    {
      ROS_ERROR( "Does not currently support non-square images because of some weird bug. Feel free to fork and fix!" );
      return;
    }

    ROS_INFO_STREAM( "Map Height: " << image_->y << " Map Width: " << image_->x );

    // Create an array of ints that represent the cost of every pixel
    cost_.resize( image_->x, image_->y );

    // gets the min and max values of the cost map
    getMinMaxCost();

    // Generate the cost map
    createCostMap();

    ROS_INFO_STREAM( "OMPL version: " << OMPL_VERSION );

    // OMPL Processing -------------------------------------------------------------------------------------------------
    // Run OMPL and display
    if( planWithSimpleSetup() )
    {
      // Make line color
      std_msgs::ColorRGBA color;
      color.a = 1.0;

      // Render the map ---------------------------------------------------
      viewer_->displayTriangles(image_, cost_);
      ros::Duration(0.1).sleep();

      // Visualize the explored space ---------------------------------------
      viewer_->displayGraph(cost_, planner_data_);
      ros::Duration(0.1).sleep();

      // Visualize the sample locations -----------------------------------
      viewer_->displaySamples(cost_, planner_data_);
      ros::Duration(0.1).sleep();

      // Show basic solution ----------------------------------------
      if( false )
      {
        // Visualize the chosen path
        color.r = 1.0;
        color.g = 0.0;
        color.b = 0.0;
        viewer_->displayResult( simple_setup_->getSolutionPath(), &color, cost_ );
        ros::Duration(0.25).sleep();
      }

      // Interpolate -------------------------------------------------------
      if( true )
      {
        simple_setup_->getSolutionPath().interpolate();

        // Visualize the chosen path
        color.r = 0.0;
        color.g = 1.0;
        color.b = 0.0;
        viewer_->displayResult( simple_setup_->getSolutionPath(), &color, cost_ );
        //      ros::Duration(0.25).sleep();
      }

      // Simplify solution ------------------------------------------------------
      if( false )
      {
        simple_setup_->simplifySolution();

        // Visualize the chosen path
        color.r = 0.0;
        color.g = 0.5;
        color.b = 0.5;
        viewer_->displayResult( simple_setup_->getSolutionPath(), &color, cost_ );
        ros::Duration(0.25).sleep();
      }
    }
    else // just show the map
    {
      // Render the map
      viewer_->displayTriangles(image_, cost_);
      ros::Duration(0.25).sleep();
    }

  }

 private:

  /**
   * \brief Helper Function: calculate cost map
   */
  void createCostMap()
  {
    // This factor is the author's visual preference for scaling a cost map in Rviz
    const double artistic_scale = 2.0;

    // This scale adapts that factor depending on the cost map min max
    const double scale = (max_cost_ - min_cost_ ) / ( image_->x / artistic_scale );

    // Dynamically calculate the obstacle threshold
    max_threshold_ = max_cost_ - ( MAX_THRESHOLD_PERCENTAGE_ / 100 * (max_cost_ - min_cost_) );

    // Preprocess the pixel data for cost and give it a nice colored tint
    for( size_t i = 0; i < image_->getSize(); ++i )
    {
      // Calculate cost
      cost_.data()[i]  = ( image_->data[ i ].red ) / scale;

      // Prevent cost from being zero
      if( !cost_.data()[i] )
        cost_.data()[i] = 1;

      // Color different if it is an obstacle
      if( cost_.data()[i] > max_threshold_ )
      {
        image_->data[ i ].red = 255; //image_->data[ i ].red;
        image_->data[ i ].green = image_->data[ i ].green;
        image_->data[ i ].blue = image_->data[ i ].blue;
      }

    }

  }

  /**
   * \brief Helper Function: gets the min and max values of the cost map
   */
  void getMinMaxCost()
  {
    // Find the min and max cost from the image
    min_cost_ = image_->data[ 0 ].red;
    max_cost_ = image_->data[ 0 ].red;

    for( size_t i = 0; i < image_->getSize(); ++i )
    {
      // Max
      if( image_->data[ i ].red > max_cost_ )
        max_cost_ = image_->data[ i ].red;
      // Min
      else if( image_->data[ i ].red < min_cost_ )
        min_cost_ = image_->data[ i ].red;
    }
  }

  /**
   * \brief Display the start and goal states on the image map
   * \param start state
   * \param goal state
   */
  void showStartGoal(ob::ScopedState<> start, ob::ScopedState<> goal)
  {
    // Modify the map to show start and end locations
    image_->data[ image_->getID( start[0], start[1] ) ].red = 50;
    image_->data[ image_->getID( start[0], start[1] ) ].blue = 50;
    image_->data[ image_->getID( start[0], start[1] ) ].green = 255;

    image_->data[ image_->getID( goal[0], goal[1] ) ].red = 255;
    image_->data[ image_->getID( goal[0], goal[1] ) ].blue = 255;
    image_->data[ image_->getID( goal[0], goal[1] ) ].green = 10;
  }

  /**
   * \brief Plan
   */
  bool planWithSimpleSetup()
  {
    // Construct the state space we are planning in
    ob::StateSpacePtr space( new ob::RealVectorStateSpace( DIMENSIONS ));

    // Set the bounds for the R^2
    ob::RealVectorBounds bounds( DIMENSIONS );
    bounds.setLow( 0 ); // both dimensions start at 0
    bounds.setHigh( 0, image_->x - 1 ); // allow for non-square images
    bounds.setHigh( 1, image_->y - 1 ); // allow for non-square images
    space->as<ob::RealVectorStateSpace>()->setBounds( bounds );

    // Define a simple setup class ---------------------------------------
    simple_setup_ = og::SimpleSetupPtr( new og::SimpleSetup(space) );

    // Set the setup planner (TRRT)
    og::TRRT *trrt = new og::TRRT( simple_setup_->getSpaceInformation() );
    //og::RRT *trrt = new og::RRT( simple_setup_->getSpaceInformation() );

    simple_setup_->setPlanner(ob::PlannerPtr(trrt));

    // Set state validity checking for this space
    simple_setup_->setStateValidityChecker( ob::StateValidityCheckerPtr( new TwoDimensionalValidityChecker( simple_setup_->getSpaceInformation(), cost_, max_threshold_ ) ) );

    // Start and Goal State ---------------------------------------------

    // Create start space
    ob::ScopedState<> start(space);
    if( USE_RANDOM_STATES )
    {
      start.random();
    }
    else // Manually set the start location
    {
      start[0] = 95;
      start[1] = 10;
    }

    // Create a goal state
    ob::ScopedState<> goal(space);
    if( USE_RANDOM_STATES )
    {
      goal.random();
    }
    else // Manually set the start location
    {
      goal[0] = 40;
      goal[1] = 300;
    }

    // Visualize on map
    showStartGoal(start, goal);

    // set the start and goal states
    simple_setup_->setStartAndGoalStates(start, goal);

    // Setup -----------------------------------------------------------

    // Auto setup parameters (optional actually)
    simple_setup_->setup();

    // The interval in which obstacles are checked for between states
    // simple_setup_->getSpaceInformation()->setStateValidityCheckingResolution(0.005);

    // Debug - this call is optional, but we put it in to get more output information
    simple_setup_->print();

    // Solve -----------------------------------------------------------
    ROS_INFO( "Starting OMPL motion planner..." );

    // attempt to solve the problem within x seconds of planning time
    ob::PlannerStatus solved = simple_setup_->solve( 10.0 );

    if (solved)
    {
      ROS_INFO("Solution Found");

      // Get information about the exploration data structure the motion planner used. Used later in visualizing
      planner_data_.reset( new ob::PlannerData( simple_setup_->getSpaceInformation() ) );
      simple_setup_->getPlannerData( *planner_data_ );
    }
    else
    {
      ROS_INFO("No Solution Found");
    }

    return solved;
  }

}; // end of class

} // namespace


// *********************************************************************************************************
// Main
// *********************************************************************************************************
int main( int argc, char** argv )
{
  ros::init(argc, argv, "ompl_rviz_viewer");
  ROS_INFO( "OMPL RViz Viewer ----------------------------------------- " );

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(1);
  spinner.start();

  bool verbose = true;

  std::string image_path;

  // Check if user has passed in an image to read
  if( argc > 1 )
  {
    image_path = argv[1];
  }
  else // use default image
  {
    // Get image path based on package name
    image_path = ros::package::getPath("ompl_rviz_viewer");
    if( image_path.empty() )
    {
      ROS_ERROR( "Unable to get OMPL RViz Viewer package path " );
      return false;
    }

    // Seed random
    srand ( time(NULL) );

    // Choose random image
    switch( 0 ) //rand() % 4 )
    {
      case 0:
        image_path.append( "/resources/grand_canyon.ppm" );
        break;
      case 1:
        image_path.append( "/resources/height_map0.ppm" );
        break;
      case 2:
        image_path.append( "/resources/height_map1.ppm" );
        break;
      case 3:
        image_path.append( "/resources/height_map2.ppm" );
        break;
    }
  }

  // Create the planner
  ompl_rviz_viewer::OmplRvizPlanner planner;

  // Load an image and run the planner
  ROS_INFO_STREAM_NAMED("main","Loading image " << image_path);
  planner.runImage( image_path );

  // Wait to let anything still being published finish
  ros::Duration(1).sleep();

  ROS_INFO_STREAM("Shutting down. TESTING HERE");

  return 0;
}


