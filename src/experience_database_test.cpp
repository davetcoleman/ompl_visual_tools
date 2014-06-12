/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, University of Colorado, Boulder
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
   Desc:   Various tests for testing the OMPL Experience database functionality
*/

// ROS
#include <ros/ros.h>

// Display in Rviz tool
#include <ompl_rviz_viewer/ompl_rviz_viewer.h>

// OMPL
#include <ompl/tools/lightning/ExperienceDB.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/PathGeometric.h>

namespace ompl_rviz_viewer
{

static const std::string OMPL_STORAGE_PATH = "/home/dave/ros/ompl_storage/file1";

class ExperienceDatabaseTest
{
private:

  // A shared node handle
  ros::NodeHandle nh_;

  // Show more visual and console output, with general slower run time.
  bool verbose_;

  // The created space information
  ompl::base::SpaceInformationPtr si_;

  // The visual tools for interfacing with Rviz
  ompl_rviz_viewer::OmplRvizViewerPtr viewer_;

  // The database of motions to search through
  ompl::tools::ExperienceDBPtr experienceDB_;

  // The number of dimensions - always 2 for images
  static const unsigned int DIMENSIONS = 2;

  // The cost for each x,y - which is derived from the RGB data
  intMatrix cost_;

public:

  /**
   * \brief Constructor
   * \param verbose - run in debug mode
   */
  ExperienceDatabaseTest(bool verbose)
    : verbose_(verbose)
  {
    // Load the tool for displaying in Rviz
    viewer_.reset(new ompl_rviz_viewer::OmplRvizViewer(verbose_));

    // Construct the state space we are planning in
    ob::StateSpacePtr space( new ob::RealVectorStateSpace( DIMENSIONS ));
    si_.reset(new ompl::base::SpaceInformation(space));

    // Load the experience database
    experienceDB_.reset(new ompl::tools::ExperienceDB(space));
    experienceDB_->load(OMPL_STORAGE_PATH); // load from file

    // Display all of the saved paths
    std::vector<ompl::geometric::PathGeometric> paths;
    experienceDB_->getAllPaths(paths);

    ROS_INFO_STREAM_NAMED("experience_database_test","Number of paths: " << paths.size());

    // Show all paths
    for (std::size_t i = 0; i < paths.size(); ++i)
    {        
        viewer_->displayResult( paths[i], viewer_->randColor(), cost_ );
        ROS_INFO_STREAM_NAMED("temp","Sleeping after path " << i);
        ros::Duration(0.1).sleep();
    }

    ROS_INFO_STREAM_NAMED("experience_database_test","ExperienceDatabaseTest Ready.");

  }

  /**
   * \brief Destructor
   */
  ~ExperienceDatabaseTest()
  {

  }

}; // end class

} // end namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "experience_database_test");
  ROS_INFO_STREAM_NAMED("main", "Starting ExperienceDatabaseTest...");

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // Check for verbose flag
  bool verbose = false;
  if (argc > 1)
  {
    for (std::size_t i = 0; i < argc; ++i)
    {
      if (strcmp(argv[i], "--verbose") == 0)
      {
        ROS_INFO_STREAM_NAMED("main","Running in VERBOSE mode (slower)");
        verbose = true;
      }
    }
  }

  ompl_rviz_viewer::ExperienceDatabaseTest server(verbose);

  ROS_INFO_STREAM_NAMED("main", "Shutting down.");
  ros::shutdown();

  return 0;
}

