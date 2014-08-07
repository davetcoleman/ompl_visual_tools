/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, University of Colorado, Boulder
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

/* Author: Dave Coleman
 * Desc:   Optimization objective that simply reads a value from a 2D cost map
 */


#ifndef OMPL_VISUAL_TOOLS__COST_MAP_OPTIMIZATION_OBJECTIVE_
#define OMPL_VISUAL_TOOLS__COST_MAP_OPTIMIZATION_OBJECTIVE_

// OMPL
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/Cost.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

// Boost
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>

// For reading image files
#include <ompl_visual_tools/utilities/ppm.h>

namespace ompl_visual_tools
{
typedef boost::numeric::ublas::matrix<int> intMatrix;
typedef boost::shared_ptr<intMatrix> intMatrixPtr;
}

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ompl
{
namespace base
{
/** \brief An optimization objective which defines path cost using the idea of mechanical work. To be used in conjunction with TRRT. */
class CostMap2DOptimizationObjective : public OptimizationObjective
{
public:
    /** \brief Constructor */
    CostMap2DOptimizationObjective(const SpaceInformationPtr &si, double pathLengthWeight = 0.0001)
        : OptimizationObjective(si),
          max_cost_threshold_percent_(0.4),
          image_(NULL),
          pathLengthWeight_(pathLengthWeight)
    {
        description_ = "Cost Map";

        cost_.reset(new ompl_visual_tools::intMatrix());
    };

    /** \brief Deconstructor */
    ~CostMap2DOptimizationObjective()
    {
        delete image_;
    };

    double getPathLengthWeight() const
    {
        return pathLengthWeight_;
    }

    /** \brief Defines motion cost */
    virtual ompl::base::Cost motionCost(const State *s1, const State *s2) const
    {
        // Only accrue positive changes in cost
        double positiveCostAccrued = std::max(stateCost(s2).v - stateCost(s1).v, 0.0);
        return Cost(positiveCostAccrued + pathLengthWeight_*si_->distance(s1,s2));
    };

    ompl::base::Cost stateCost(const State *state) const
    {
        const double *coords = state->as<ob::RealVectorStateSpace::StateType>()->values;

        // Return the cost from the matrix at the current dimensions
        double cost = (*cost_)( natRound(coords[1]), natRound(coords[0]) );

        return Cost(cost);
    }

    /** \brief Passed in a cost matrix loaded from an image file, etc */
    void setCostMatrix(ompl_visual_tools::intMatrixPtr cost)
    {
        cost_ = cost;
    };

    /**
     * \brief NatRounding helper function to make readings from cost map more accurate
     * \param double
     * \return rounded down number
     */
    int natRound(double x) const
    {
        return static_cast<int>(floor(x + 0.5f));
    };

    void loadImage( std::string image_path )
    {
        // Load cost map from image file
        image_ = ompl_visual_tools::readPPM( image_path.c_str() );

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
        cost_->resize( image_->x, image_->y );

        // Generate the cost map
        createCostMap();
    };

    /**
     * \brief Helper Function: calculate cost map
     */
    void createCostMap()
    {
        // gets the min and max values of the cost map
        getMinMaxPixels();

        // This factor is the author's visual preference for scaling a cost map in Rviz
        const double artistic_scale = 6.0; // smaller is taller

        const double pixel_diff = max_pixel_ - min_pixel_;

        // This scale adapts that factor depending on the cost map min max
        const double scale = pixel_diff / ( image_->x / artistic_scale ); //image->x is width

        // Dynamically calculate the obstacle threshold
        max_cost_threshold_ = (max_pixel_ - ( max_cost_threshold_percent_ * pixel_diff )) / scale;

        // Preprocess the pixel data for cost and give it a nice colored tint
        for( size_t i = 0; i < image_->getSize(); ++i )
        {
            // Calculate cost
            cost_->data()[i]  = ( image_->data[ i ].red - min_pixel_ ) / scale;

            // Prevent cost from being zero
            if( !cost_->data()[i] )
                cost_->data()[i] = 1;

            // Color different if it is an obstacle
            if( cost_->data()[i] > max_cost_threshold_ || cost_->data()[i] <= 1)
            {
                //std::cout << "cost is " <<  cost_->data()[i] << " threshold is " <<  max_cost_threshold_ << std::endl;

                image_->data[ i ].red = 255; //image_->data[ i ].red;
                image_->data[ i ].green = image_->data[ i ].green;
                image_->data[ i ].blue = image_->data[ i ].blue;
            }

        }

    }

    /**
     * \brief Helper Function: gets the min and max values of the cost map
     */
    void getMinMaxPixels()
    {
        // Find the min and max cost from the image
        min_pixel_ = image_->data[ 0 ].red;
        max_pixel_ = image_->data[ 0 ].red;

        for( size_t i = 0; i < image_->getSize(); ++i )
        {
            // Max
            if( image_->data[ i ].red > max_pixel_ )
                max_pixel_ = image_->data[ i ].red;
            // Min
            else if( image_->data[ i ].red < min_pixel_ )
                min_pixel_ = image_->data[ i ].red;
        }
    }

    // The RGB image data
    ompl_visual_tools::PPMImage *image_;

    // The cost for each x,y - which is derived from the RGB data
    ompl_visual_tools::intMatrixPtr cost_;

    // The cost at which it becomes an obstacle
    double max_cost_threshold_;

    // The percentage of the top min/max cost value that is considered an obstacle, e.g. 0.1 is top 10% of peaks
    double max_cost_threshold_percent_;

protected:

    /** \brief The weighing factor for the path length in the mechanical work objective formulation. */
    double pathLengthWeight_;

    // Remember the min and max cost from the image
    int max_pixel_;
    int min_pixel_;

};

typedef boost::shared_ptr< CostMap2DOptimizationObjective > CostMap2DOptimizationObjectivePtr;
}
}

#endif


