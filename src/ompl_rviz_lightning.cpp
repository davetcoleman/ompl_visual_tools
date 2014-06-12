/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, University of Colorado, Boulder
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
#include <ompl_rviz_viewer/ompl_rviz_viewer.h>

// Custom validity checker that accounts for cost
#include <ompl_rviz_viewer/cost_map_optimization_objective.h>
#include <ompl_rviz_viewer/two_dimensional_validity_checker.h>

// OMPL planner
#include <ompl/tools/lightning/Lightning.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <ompl/base/PlannerTerminationCondition.h>

namespace ob = ompl::base;
namespace ot = ompl::tools;
namespace og = ompl::geometric;

namespace ompl_rviz_viewer
{

/**
 * \brief Lightning Planning Class
 */
class OmplRvizLightning
{

private:
    // The RGB image data
    PPMImage *image_;

    // The cost for each x,y - which is derived from the RGB data
    intMatrix cost_;

    // Save the experience setup until the program ends so that the planner data is not lost
    ot::LightningPtr lightning_setup_;

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

    // Flag for determining amount of debug output to show
    bool verbose_;

public:

    /**
     * \brief Constructor
     */
    OmplRvizLightning(bool verbose)
        : verbose_(verbose),
          image_(NULL)
    {
        ROS_INFO_STREAM( "OMPL version: " << OMPL_VERSION );

        // Load the tool for displaying in Rviz
        viewer_.reset(new ompl_rviz_viewer::OmplRvizViewer(verbose_));
    }

    /**
     * \brief Deconstructor
     */
    ~OmplRvizLightning()
    {
        delete image_;
    }

    /**
     * \brief Main Function
     */
    void loadImage( std::string image_path )
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

        if (verbose_)
            ROS_INFO_STREAM( "Map Height: " << image_->y << " Map Width: " << image_->x );

        // Create an array of ints that represent the cost of every pixel
        cost_.resize( image_->x, image_->y );

        // gets the min and max values of the cost map
        getMinMaxCost();

        // Generate the cost map
        createCostMap();

        // Render the map ---------------------------------------------------
        viewer_->displayTriangles(image_, cost_);
        ros::Duration(0.1).sleep();
    }

    bool plan(int runs)
    {
        // OMPL Processing -------------------------------------------------------------------------------------------------
        // Run OMPL and display

        // Construct the state space we are planning in
        ob::StateSpacePtr space( new ob::RealVectorStateSpace( DIMENSIONS ));

        // Set the bounds for the R^2
        ob::RealVectorBounds bounds( DIMENSIONS );
        bounds.setLow( 0 ); // both dimensions start at 0
        bounds.setHigh( 0, image_->x - 1 ); // allow for non-square images
        bounds.setHigh( 1, image_->y - 1 ); // allow for non-square images
        space->as<ob::RealVectorStateSpace>()->setBounds( bounds );

        // Define a experience setup class ---------------------------------------
        //lightning_ = ompl::


        lightning_setup_ = ot::LightningPtr( new ot::Lightning(space) );

        // Set the setup planner (TRRT)
        og::TRRT *trrt = new og::TRRT( lightning_setup_->getSpaceInformation() );
        //og::RRT *trrt = new og::RRT( lightning_setup_->getSpaceInformation() );

        lightning_setup_->setPlanner(ob::PlannerPtr(trrt));

        // Start and Goal State ---------------------------------------------
        ob::PlannerStatus solved;

        for (std::size_t i = 0; i < runs; ++i)
        {
            // Check if user wants to shutdown
            if (!ros::ok())
            {
                ROS_WARN_STREAM_NAMED("planWithLightning","Terminating early");
                break;
            }

            ROS_INFO_STREAM_NAMED("planWithLightning","Planning #" << i << " out of " << runs << " ------------------------------------");

            // Clear all planning data. This only includes data generated by motion plan computation.
            // Planner settings, start & goal states are not affected.
            if (i) // skip first run
                lightning_setup_->clear();

            // Set state validity checking for this space
            lightning_setup_->setStateValidityChecker( ob::StateValidityCheckerPtr( new ob::TwoDimensionalValidityChecker(
                        lightning_setup_->getSpaceInformation(), cost_, max_threshold_ ) ) );

            // Setup the optimization objective to use the 2d cost map
            ompl::base::OptimizationObjectivePtr opt;
            opt.reset(new ompl::base::CostMapOptimizationObjective( lightning_setup_->getSpaceInformation(), cost_ ));
            lightning_setup_->setOptimizationObjective(opt);

            // Create the termination condition
            ob::PlannerTerminationCondition ptc = ob::timedPlannerTerminationCondition( 10.0, 0.1 );

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
            lightning_setup_->setStartAndGoalStates(start, goal);

            // Setup -----------------------------------------------------------

            // Auto setup parameters (optional actually)
            lightning_setup_->setup();

            // The interval in which obstacles are checked for between states
            // lightning_setup_->getSpaceInformation()->setStateValidityCheckingResolution(0.005);

            // Debug - this call is optional, but we put it in to get more output information
            //lightning_setup_->print();

            // Solve -----------------------------------------------------------
            ROS_INFO( "Starting OMPL motion planner..." );

            // attempt to solve the problem within x seconds of planning time
            solved = lightning_setup_->solve( ptc );

            if (solved)
            {                
                ROS_INFO("Solution Found");
                if (runs == 1)
                {
                    // display all the aspects of the solution
                    displayPlannerData(false);
                }
                else
                {
                    // only display the paths
                    displayPlannerData(true);
                }
            }
            else
            {
                ROS_ERROR("No Solution Found");
            }
        }

        if (!lightning_setup_->save())
            ROS_ERROR("Unable to save experience database");

        return solved;
    }

    /**
     * \brief Show the planner data in Rviz
     * \param planner_data
     * \param just_path - if true, do not display the search tree/graph or the samples
     */
    void displayPlannerData(bool just_path)
    {
        // Get information about the exploration data structure the motion planner used. Used later in visualizing
        const ob::PlannerDataPtr planner_data( new ob::PlannerData( lightning_setup_->getSpaceInformation() ) );
        lightning_setup_->getPlannerData( *planner_data );

        std_msgs::ColorRGBA color;
        color.a = 1.0;
        
        // Optionally display the search tree/graph or the samples
        if (!just_path)
        {
            // Visualize the explored space ---------------------------------------
            viewer_->displayGraph(cost_, planner_data);
            ros::Duration(0.1).sleep();

            // Visualize the sample locations -----------------------------------
            viewer_->displaySamples(cost_, planner_data);
            ros::Duration(0.1).sleep();
        }

        // Show basic solution ----------------------------------------
        if( false )
        {
            // Visualize the chosen path
            color.r = 1.0;
            color.g = 0.0;
            color.b = 0.0;
            viewer_->displayResult( lightning_setup_->getSolutionPath(), color, cost_ );
            ros::Duration(0.25).sleep();
        }

        // Interpolate -------------------------------------------------------
        if( true )
        {
            lightning_setup_->getSolutionPath().interpolate();

            // Visualize the chosen path
            color.r = 0.0;
            color.g = 1.0;
            color.b = 0.0;
            viewer_->displayResult( lightning_setup_->getSolutionPath(), color, cost_ );
            //      ros::Duration(0.25).sleep();
        }

        // Simplify solution ------------------------------------------------------
        if( false )
        {
            lightning_setup_->simplifySolution();

            // Visualize the chosen path
            color.r = 0.0;
            color.g = 0.5;
            color.b = 0.5;
            viewer_->displayResult( lightning_setup_->getSolutionPath(), color, cost_ );
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
        geometry_msgs::Point start_pt;
        start_pt.x = start[0];
        start_pt.y = start[1];
        start_pt.z = viewer_->getCostHeight(start_pt, cost_);
        viewer_->publishSphere(start_pt, viewer_->green_, 1.5);

        geometry_msgs::Point goal_pt;
        goal_pt.x = goal[0];
        goal_pt.y = goal[1];
        goal_pt.z = viewer_->getCostHeight(goal_pt, cost_);
        viewer_->publishSphere(goal_pt, viewer_->red_, 1.5);

        // Modify the map to show start and end locations
        /*image_->data[ image_->getID( start[0], start[1] ) ].red = 50;
          image_->data[ image_->getID( start[0], start[1] ) ].blue = 50;
          image_->data[ image_->getID( start[0], start[1] ) ].green = 255;

          image_->data[ image_->getID( goal[0], goal[1] ) ].red = 255;
          image_->data[ image_->getID( goal[0], goal[1] ) ].blue = 255;
          image_->data[ image_->getID( goal[0], goal[1] ) ].green = 10;
        */
    }

}; // end of class

} // namespace


// *********************************************************************************************************
// Main
// *********************************************************************************************************
int main( int argc, char** argv )
{
    ros::init(argc, argv, "ompl_rviz_viewer");
    ROS_INFO( "OMPL RViz Viewer with Lightning Framework ----------------------------------------- " );

    // Allow the action server to recieve and send ros messages
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Default argument values
    bool verbose = false;
    std::string image_path;
    int runs = 1;

    if (argc > 1)
    {
        for (std::size_t i = 0; i < argc; ++i)
        {
            // Check for verbose flag
            if (strcmp(argv[i], "--verbose") == 0)
            {
                ROS_INFO_STREAM_NAMED("main","Running in VERBOSE mode (slower)");
                verbose = true;
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
        image_path = ros::package::getPath("ompl_rviz_viewer");
        if( image_path.empty() )
        {
            ROS_ERROR( "Unable to get OMPL RViz Viewer package path " );
            return false;
        }

        // Seed random
        srand ( time(NULL) );

        // Choose random image
        switch( 1 ) //rand() % 4 )
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
    ompl_rviz_viewer::OmplRvizLightning planner(verbose);

    // Load an image and run the planner
    ROS_INFO_STREAM_NAMED("main","Loading image " << image_path);
    planner.loadImage( image_path );

    // Wait to let anything still being published finish
    ros::Duration(0.1).sleep();

    ROS_INFO_STREAM("Shutting down.");

    return 0;
}


