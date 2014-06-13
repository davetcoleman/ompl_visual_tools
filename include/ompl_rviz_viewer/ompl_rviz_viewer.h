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
   Desc:   Tools for displaying OMPL components in Rviz
*/

#ifndef OMPL_RVIZ_VIEWER__OMPL_RVIZ_VIEWER_
#define OMPL_RVIZ_VIEWER__OMPL_RVIZ_VIEWER_

// ROS
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

// Boost
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>

// OMPL
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/PlannerData.h>
#include <ompl/base/ScopedState.h>

#include <ompl/config.h>

// Custom validity checker that accounts for cost
#include <ompl_rviz_viewer/cost_map_2d_optimization_objective.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace bnu = boost::numeric::ublas;

namespace ompl_rviz_viewer
{

//typedef boost::numeric::ublas::matrix<int> intMatrix;
//typedef boost::shared_ptr<intMatrix> intMatrixPtr;

static const std::string BASE_FRAME = "/world";
static const double COST_HEIGHT_OFFSET = 0.5;

enum rviz_colors { RED, GREEN, BLUE, GREY, WHITE, ORANGE, BLACK, YELLOW, TRANSLUCENT, RAND };

/**
 * \brief Nat_Rounding helper function to make readings from cost map more accurate
 * \param double
 * \return rounded down number
 */
int nat_round(double x)
{
    return static_cast<int>(floor(x + 0.5f));
}

typedef std::map< std::string, std::list<std::size_t> > MarkerList;

class OmplRvizViewer
{
private:

    // A shared node handle
    ros::NodeHandle nh_;

    // Show more visual and console output, with general slower run time.
    bool verbose_;

    // A shared ROS publisher
    ros::Publisher marker_pub_;

    // Track what markers have been published so that we can delete them
    //MarkerList marker_tracker_;


public:

    /**
     * \brief Constructor
     * \param verbose - run in debug mode
     */
    OmplRvizViewer(bool verbose)
        : verbose_(verbose)
    {
        // ROS Publishing stuff
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("ompl_rviz_markers", 1);
        ros::Duration(1).sleep();

    }

    /**
     * \brief Destructor
     */
    ~OmplRvizViewer()
    {

    }

    void markerPublisher(const visualization_msgs::Marker& marker)
    {
        // Save this marker id and namespacee
        //marker_tracker_[marker.ns].push_back( marker.id );

        // Publish normal
        marker_pub_.publish( marker );

        // Process
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }

    void deleteAllMarkers()
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = BASE_FRAME;
        marker.header.stamp = ros::Time();
        marker.action = visualization_msgs::Marker::DELETE;

        // Set the namespace and id for this marker.  This serves to create a unique ID
        marker.ns = "";
        marker.id = 0;

        // Publish normal
        marker_pub_.publish( marker );

        // Process
        ros::spinOnce();
        ros::Duration(0.1).sleep();

        /*
          for(MarkerList::iterator iterator = marker_tracker_.begin(); iterator != marker_tracker_.end(); iterator++)
          {
          for (std::list<std::size_t>::iterator id = iterator->second.begin(); id != iterator->second.end(); id++)
          {
          // Set the namespace and id for this marker.  This serves to create a unique ID
          marker.ns = iterator->first; // key
          marker.id = *id; // value

          //ROS_INFO_STREAM_NAMED("temp","Deleting marker: \n" << marker);

          // Publish normal
          marker_pub_.publish( marker );

          // Process
          ros::spinOnce();
          ros::Duration(0.01).sleep();
          }
          }
        */
    }

    /**
     * \brief Helper function for converting a point to the correct cost
     */
    double getCost(const geometry_msgs::Point &point, const intMatrix &cost)
    {
        return double(cost( nat_round(point.y), nat_round(point.x) )) / 2.0;
    }

    /**
     * \brief Use bilinear interpolation, if necessary, to find the cost of a point between whole numbers
     *        From http://supercomputingblog.com/graphics/coding-bilinear-interpolation/
     */
    double getCostHeight(const geometry_msgs::Point &point, const intMatrix &cost)
    {
        // TODO make faster

        //std::cout << "------------------------" << std::endl;

        // check if whole number
        if (floor(point.x) == point.x && floor(point.y) == point.y)
            return getCost(point, cost) + COST_HEIGHT_OFFSET;

        // else do Bilinear Interpolation

        // top left
        geometry_msgs::Point a;
        a.x = floor(point.x);
        a.y = ceil(point.y);
        a.z = getCost(a, cost);

        // bottom left
        geometry_msgs::Point b;
        b.x = floor(point.x);
        b.y = floor(point.y);
        b.z = getCost(b, cost);

        // bottom right
        geometry_msgs::Point c;
        c.x = ceil(point.x);
        c.y = floor(point.y);
        c.z = getCost(c, cost);

        // top right
        geometry_msgs::Point d;
        d.x = ceil(point.x);
        d.y = ceil(point.y);
        d.z = getCost(d, cost);

        //std::cout << "point: \n" << point << std::endl;
        //std::cout << "a: \n" << a << std::endl;
        //std::cout << "b: \n" << b << std::endl;
        //std::cout << "c: \n" << c << std::endl;
        //std::cout << "d: \n" << d << std::endl;

        double R1;
        double R2;

        // check if our x axis is the same
        if (c.x == b.x)
        {
            // just choose either
            R1 = b.z;
            R2 = a.z;
        }
        else
        {
            R1 = ((c.x - point.x)/(c.x - b.x))*b.z + ((point.x - b.x)/(c.x - b.x))*c.z;
            R2 = ((c.x - point.x)/(c.x - b.x))*a.z + ((point.x - b.x)/(c.x - b.x))*d.z;
        }

        //std::cout << "R1: " << R1 << std::endl;
        //std::cout << "R2: " << R2 << std::endl;

        // After the two R values are calculated, the value of P can finally be calculated by a weighted average of R1 and R2.
        double val;

        // check if y axis is the same
        if ( a.y - b.y == 0) // division by zero
            val = R1;
        else
            val = ((a.y - point.y)/(a.y - b.y))*R1 + ((point.y - b.y)/(a.y - b.y))*R2;

        //std::cout << "val: " << val << std::endl;
        return val + COST_HEIGHT_OFFSET;
    }

    /**
     * \brief After running the algorithm, this converts the results to a vector of coordinates
     */
    std::vector<std::pair<double, double> > convertSolutionToVector(og::PathGeometric& path)
    {
        // The returned result
        std::vector<std::pair<double, double> > coordinates;

        // Get data
        const std::vector<ob::State*>& states = path.getStates();

        // Convert solution to coordinate vector
        //for( std::vector<ob::State*>::const_iterator state_it = states.begin();
        //         state_it != states.end(); ++state_it )
        for( size_t state_id = 0; state_id < states.size(); ++state_id )
        {
            const ob::State *state = states[ state_id ];

            if (!state)
                continue; // no data?

            // Convert to RealVectorStateSpace
            const ob::RealVectorStateSpace::StateType *real_state =
                static_cast<const ob::RealVectorStateSpace::StateType*>(state);

            // Add to vector of results
            coordinates.push_back( std::pair<double,double>( real_state->values[0], real_state->values[1] ) );
        }

        return coordinates;
    }

    /**
     * \brief Visualize Results
     */
    void displayTriangles(PPMImage *image, intMatrix &cost)
    {
        visualization_msgs::Marker marker;
        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        marker.header.frame_id = BASE_FRAME;
        marker.header.stamp = ros::Time::now();

        // Set the namespace and id for this marker.  This serves to create a unique ID
        marker.ns = "cost_map";

        // Set the marker type.
        marker.type = visualization_msgs::Marker::TRIANGLE_LIST;

        // Set the marker action.  Options are ADD and DELETE
        marker.action = visualization_msgs::Marker::ADD;
        marker.id = 1;

        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;
        marker.color = getColor( RED );

        // Visualize Results -------------------------------------------------------------------------------------------------
        for( size_t marker_id = 0; marker_id < image->getSize(); ++marker_id )
        {

            unsigned int x = marker_id % image->x;    // Map index back to coordinates
            unsigned int y = marker_id / image->x;    // Map index back to coordinates

            // Make right and down triangle
            // Check that we are not on the far right or bottom
            if( ! (x + 1 >= image->x ||  y + 1 >= image->y ) )
            {
                addPoint( x,   y, &marker, image, cost );
                addPoint( x+1, y, &marker, image, cost );
                addPoint( x,   y+1, &marker, image, cost );
            }

            // Make back and down triangle
            // Check that we are not on the far left or bottom
            if( ! ( int(x) - 1 < 0 ||  y + 1 >= image->y ) )
            {
                addPoint( x,   y, &marker, image, cost );
                addPoint( x,   y+1, &marker, image, cost );
                addPoint( x-1, y+1, &marker, image, cost );
            }

        }

        markerPublisher(marker);
    }

    // *********************************************************************************************************
    // Helper Function to display triangles
    // *********************************************************************************************************
    void addPoint( int x, int y, visualization_msgs::Marker* marker, PPMImage *image, intMatrix &cost  )
    {
        // Point
        geometry_msgs::Point point;
        point.x = x;
        point.y = y;
        point.z = getCost(point, cost); // to speed things up, we know is always whole number
        marker->points.push_back( point );

        std_msgs::ColorRGBA color;
        color.r = image->data[ image->getID( x, y ) ].red / 255.0;
        color.g = image->data[ image->getID( x, y ) ].green / 255.0;
        color.b = image->data[ image->getID( x, y ) ].blue / 255.0;
        color.a = 1.0;

        marker->colors.push_back( color );
    }

    // *********************************************************************************************************
    // Helper Function for Display Graph that makes the exploration lines follow the curvature of the map
    // *********************************************************************************************************
    void interpolateLine( const geometry_msgs::Point &p1, const geometry_msgs::Point &p2, visualization_msgs::Marker* marker,
        const rviz_colors& color, intMatrix &cost )
    {
        // Copy to non-const
        geometry_msgs::Point point_a = p1;
        geometry_msgs::Point point_b = p2;

        // Get the heights
        point_a.z = getCostHeight(point_a, cost);
        point_b.z = getCostHeight(point_b, cost);

        //ROS_INFO_STREAM("a is: " << point_a);
        //ROS_INFO_STREAM("b is: " << point_b);

        // Switch the coordinates such that x1 < x2
        if( point_a.x > point_b.x )
        {
            // Swap the coordinates
            geometry_msgs::Point point_temp = point_a;
            point_a = point_b;
            point_b = point_temp;
        }

        // Show the straight line --------------------------------------------------------------------
        if( false )
        {
            // Add the point pair to the line message
            marker->points.push_back( point_a );
            marker->points.push_back( point_b );
            marker->colors.push_back( getColor( color ) );
            marker->colors.push_back( getColor( color ) );

            // Show start and end point
            publishSphere(point_a, GREEN);
            publishSphere(point_b, RED);
        }

        // Interpolate the line ----------------------------------------------------------------------

        // Calculate slope between the lines
        double m = (point_b.y - point_a.y)/(point_b.x - point_a.x);

        // Calculate the y-intercep
        double b = point_a.y - m * point_a.x;

        // Define the interpolation interval
        double interval = 0.1; //0.5;

        // Make new locations
        geometry_msgs::Point temp_a = point_a; // remember the last point
        geometry_msgs::Point temp_b = point_a; // move along this point

        // Loop through the line adding segements along the cost map
        for( temp_b.x = point_a.x + interval; temp_b.x <= point_b.x; temp_b.x += interval )
        {
            //publishSphere(temp_b, color2);

            // Find the y coordinate
            temp_b.y = m*temp_b.x + b;

            // Add the new heights
            temp_a.z = getCostHeight(temp_a, cost);
            temp_b.z = getCostHeight(temp_b, cost);

            // Add the point pair to the line message
            marker->points.push_back( temp_a );
            marker->points.push_back( temp_b );
            // Add colors
            marker->colors.push_back( getColor( color ) );
            marker->colors.push_back( getColor( color ) );

            // Remember the last coordiante for next iteration
            temp_a = temp_b;
        }

        // Finish the line for non-even interval lengths
        marker->points.push_back( temp_a );
        marker->points.push_back( point_b );
        // Add colors
        marker->colors.push_back( getColor( color )  );
        marker->colors.push_back( getColor( color ) );

    }

    // *********************************************************************************************************
    // Display Explored Space
    // *********************************************************************************************************
    void displayGraph(intMatrix &cost, ob::PlannerDataPtr planner_data)
    {
        visualization_msgs::Marker marker;
        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        marker.header.frame_id = BASE_FRAME;
        marker.header.stamp = ros::Time();

        // Set the namespace and id for this marker.  This serves to create a unique ID
        marker.ns = "space_exploration";

        // Set the marker type.
        marker.type = visualization_msgs::Marker::LINE_LIST;

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

        marker.scale.x = 0.2;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;

        marker.color = getColor( BLACK );

        geometry_msgs::Point this_vertex;
        geometry_msgs::Point next_vertex;

        // Make line color
        std_msgs::ColorRGBA color = getColor( BLUE );

        if (verbose_)
            ROS_INFO("Publishing Graph");

        // Loop through all verticies
        for( int vertex_id = 0; vertex_id < int( planner_data->numVertices() ); ++vertex_id )
        {

            this_vertex = getCoordinates( vertex_id, planner_data );

            // Get the out edges from the current vertex
            std::vector<unsigned int> edge_list;
            planner_data->getEdges( vertex_id, edge_list );

            // Now loop through each edge
            for( std::vector<unsigned int>::const_iterator edge_it = edge_list.begin();
                 edge_it != edge_list.end(); ++edge_it)
            {
                // Convert vertex id to next coordinates
                next_vertex = getCoordinates( *edge_it, planner_data );

                interpolateLine( this_vertex, next_vertex, &marker, BLUE, cost );
            }

        }

        // Publish the marker
        markerPublisher(marker);
    }

    // *********************************************************************************************************
    // Display Sample Points
    // *********************************************************************************************************
    void displaySamples(intMatrix &cost, ob::PlannerDataPtr planner_data)
    {
        visualization_msgs::Marker marker;
        // Set the frame ID and timestamp.
        marker.header.frame_id = BASE_FRAME;
        marker.header.stamp = ros::Time();

        // Set the namespace and id for this marker.  This serves to create a unique ID
        marker.ns = "sample_locations";

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

        marker.color = getColor( RED );

        // Make line color
        std_msgs::ColorRGBA color = getColor( RED );

        // Point
        geometry_msgs::Point point_a;

        if (verbose_)
            ROS_INFO("Publishing Spheres");

        // Loop through all verticies
        for( int vertex_id = 0; vertex_id < int( planner_data->numVertices() ); ++vertex_id )
        {
            // First point
            point_a = getCoordinates( vertex_id, planner_data );
            point_a.z = getCostHeight(point_a, cost);

            // Add the point pair to the line message
            marker.points.push_back( point_a );
            marker.colors.push_back( color );
        }

        // Publish the marker
        markerPublisher(marker);
    }

    // *********************************************************************************************************
    // Display States
    // *********************************************************************************************************
    void displayStates(std::vector<const ompl::base::State*> states)
    {
        visualization_msgs::Marker marker;
        // Set the frame ID and timestamp.
        marker.header.frame_id = BASE_FRAME;
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

        marker.color = getColor( RED );

        // Make line color
        std_msgs::ColorRGBA color = getColor( RED );

        // Point
        geometry_msgs::Point point_a;

        if (verbose_)
            ROS_INFO("Publishing Spheres");

        // Loop through all verticies
        for( int vertex_id = 0; vertex_id < int( states.size() ); ++vertex_id )
        {
            // First point
            point_a = getCoordinates( states[vertex_id] );

            // Add the point pair to the line message
            marker.points.push_back( point_a );
            marker.colors.push_back( color );
        }

        // Publish the marker
        markerPublisher(marker);
    }

    void publishSphere(const geometry_msgs::Point &point, const rviz_colors color, double scale = 0.3)
    {
        visualization_msgs::Marker sphere_marker;

        sphere_marker.header.frame_id = BASE_FRAME;

        // Set the namespace and id for this marker.  This serves to create a unique ID
        sphere_marker.ns = "sphere";
        // Set the marker type.
        sphere_marker.type = visualization_msgs::Marker::SPHERE_LIST;
        // Set the marker action.  Options are ADD and DELETE
        sphere_marker.action = visualization_msgs::Marker::ADD;
        // Marker group position and orientation
        sphere_marker.pose.position.x = 0;
        sphere_marker.pose.position.y = 0;
        sphere_marker.pose.position.z = 0;
        sphere_marker.pose.orientation.x = 0.0;
        sphere_marker.pose.orientation.y = 0.0;
        sphere_marker.pose.orientation.z = 0.0;
        sphere_marker.pose.orientation.w = 1.0;

        // Add the point pair to the line message
        sphere_marker.points.push_back( point );
        sphere_marker.colors.push_back( getColor(color) );
        // Lifetime
        //sphere_marker.lifetime = ros

        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        sphere_marker.header.stamp = ros::Time::now();

        static int sphere_id_ = 0;
        sphere_marker.id = ++sphere_id_;
        sphere_marker.color = getColor(color);
        sphere_marker.scale.x = scale;
        sphere_marker.scale.y = scale;
        sphere_marker.scale.z = scale;

        // Update the single point with new pose
        sphere_marker.points[0] = point;
        sphere_marker.colors[0] = getColor(color);

        // Publish
        markerPublisher(sphere_marker);
    }


    /**
     * \brief Display Result Path
     */
    void displayResult( og::PathGeometric& path, const rviz_colors color, const intMatrix& cost )
    {
        const std::vector<std::pair<double, double> > coordinates = convertSolutionToVector(path);

        visualization_msgs::Marker marker;
        // Set the frame ID and timestamp.
        marker.header.frame_id = BASE_FRAME;
        marker.header.stamp = ros::Time();

        // Set the namespace and id for this marker.  This serves to create a unique ID
        marker.ns = "result_path";

        std_msgs::ColorRGBA this_color = getColor( color );

        // Set the marker type.
        marker.type = visualization_msgs::Marker::LINE_LIST;

        // Set the marker action.  Options are ADD and DELETE
        marker.action = visualization_msgs::Marker::ADD;

        // Provide a new id every call to this function
        static int result_id = 0;
        marker.id = result_id++;

        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.0;

        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.4;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;

        marker.color = this_color;

        if (verbose_)
            ROS_INFO("Publishing result");

        // Get the initial points
        double x1 = coordinates[0].first;
        double y1 = coordinates[0].second;
        // Declare the second points
        double x2;
        double y2;

        // Convert path coordinates to red line
        for( unsigned int i = 1; i < coordinates.size(); ++i )
        {
            x2 = coordinates[i].first;
            y2 = coordinates[i].second;

            // Points
            geometry_msgs::Point point_a;
            geometry_msgs::Point point_b;

            // First point
            point_a.x = x1;
            point_a.y = y1;

            // Create a second point
            point_b.x = x2;
            point_b.y = y2;

            // Add cost if available
            if ( cost.size1() > 0 && cost.size2() > 0 )
            {
                point_a.z = getCostHeight(point_a, cost);
                point_b.z = getCostHeight(point_b, cost);
            }

            // Add the point pair to the line message
            marker.points.push_back( point_a );
            marker.points.push_back( point_b );
            marker.colors.push_back( this_color );
            marker.colors.push_back( this_color );

            // Save these coordinates for next line
            x1 = x2;
            y1 = y2;
        }

        // Publish the marker
        markerPublisher(marker);
    }

    /**
     * \brief Helper Function: gets the x,y coordinates for a given vertex id
     * \param id of a vertex
     * \param result from an OMPL planner
     * \return geometry point msg with no z value filled in
     */
    geometry_msgs::Point getCoordinates( int vertex_id, ob::PlannerDataPtr planner_data )
    {
        ob::PlannerDataVertex vertex = planner_data->getVertex( vertex_id );

        // Get this vertex's coordinates
        const ob::State *state = vertex.getState();

        return getCoordinates(state);
    }

    geometry_msgs::Point getCoordinates( const ob::State *state )
    {

        if (!state)
        {
            ROS_ERROR("No state found for a vertex");
            exit(1);
        }

        // Convert to RealVectorStateSpace
        const ob::RealVectorStateSpace::StateType *real_state =
            static_cast<const ob::RealVectorStateSpace::StateType*>(state);

        // Create point
        geometry_msgs::Point point;
        point.x = real_state->values[0];
        point.y = real_state->values[1];
        point.z = 0; // dummy value
        return point;
    }

    double fRand(double fMin, double fMax)
    {
        double f = (double)rand() / RAND_MAX;
        return fMin + f * (fMax - fMin);
    }

    /**
     * \brief Display the start and goal states on the image map
     * \param start state
     * \param goal state
     * \param cost map
     */
    void showState(ob::ScopedState<> state, const intMatrix &cost, const rviz_colors &color)
    {
        geometry_msgs::Point state_pt;
        state_pt.x = state[0];
        state_pt.y = state[1];
        state_pt.z = getCostHeight(state_pt, cost);

        publishSphere(state_pt, color, 1.5);
    }

    /**
     * \brief Visualize the sampling area in Rviz
     * \param state_area - the center point of the uniform sampler
     * \param distance - the radius around the center for sampling
     */
    void displaySampleRegion(const ob::ScopedState<>& state_area, const double& distance, const intMatrix &cost)
    {
        geometry_msgs::Point state_pt;
        state_pt.x = state_area[0];
        state_pt.y = state_area[1];
        state_pt.z = getCostHeight(state_pt, cost);

        publishSphere(state_pt, BLACK, 1.5); // mid point
        publishSphere(state_pt, TRANSLUCENT, 2.1*distance); // outer sphere (x2 b/c its a radius, x0.1 to make it look nicer)
    }

    std_msgs::ColorRGBA getColor(const rviz_colors &color)
    {
        std_msgs::ColorRGBA result;
        result.a = 1.0;
        switch(color)
        {
            case RED:
                result.r = 0.8;
                result.g = 0.1;
                result.b = 0.1;
                break;
            case GREEN:
                result.r = 0.1;
                result.g = 0.8;
                result.b = 0.1;
                break;
            case GREY:
                result.r = 0.9;
                result.g = 0.9;
                result.b = 0.9;
                break;
            case WHITE:
                result.r = 1.0;
                result.g = 1.0;
                result.b = 1.0;
                break;
            case ORANGE:
                result.r = 1.0;
                result.g = 0.5;
                result.b = 0.0;
                break;
            case TRANSLUCENT:
                result.r = 0.1;
                result.g = 0.1;
                result.b = 0.8;
                result.a = 0.3;
                break;
            case BLACK:
                result.r = 0.0;
                result.g = 0.0;
                result.b = 0.0;
                break;
            case YELLOW:
                result.r = 1.0;
                result.g = 1.0;
                result.b = 0.0;
                break;
            case RAND:
                result.r = fRand(0.0,1.0);
                result.g = fRand(0.0,1.0);
                result.b = fRand(0.0,1.0);
                break;
            case BLUE:
            default:
                result.r = 0.1;
                result.g = 0.1;
                result.b = 0.8;
        }

        return result;
    }

}; // end class

typedef boost::shared_ptr<OmplRvizViewer> OmplRvizViewerPtr;
typedef boost::shared_ptr<const OmplRvizViewer> OmplRvizViewerConstPtr;

} // end namespace

#endif
