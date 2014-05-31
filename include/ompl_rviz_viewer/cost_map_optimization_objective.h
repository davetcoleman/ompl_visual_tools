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


#ifndef OMPL_BASE_OBJECTIVES_COST_MAP_OPTIMIZATION_OBJECTIVE_
#define OMPL_BASE_OBJECTIVES_COST_MAP_OPTIMIZATION_OBJECTIVE_

#include <ompl/base/OptimizationObjective.h>

// Boost
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>


namespace ompl_rviz_viewer
{
  typedef boost::numeric::ublas::matrix<int> intMatrix;
  typedef boost::shared_ptr<intMatrix> intMatrixPtr;
}

namespace ompl
{
  namespace base
  {
    /** \brief An optimization objective which defines path cost using the idea of mechanical work. To be used in conjunction with TRRT. */
    class CostMapOptimizationObjective : public OptimizationObjective
    {
    public:
      /** \brief The mechanical work formulation requires a weighing factor to use for the length of a path in order to disambiguate optimal paths.
          This weighing factor should be small. The default value for this weight is 0.00001. */
      CostMapOptimizationObjective(const SpaceInformationPtr &si, ompl_rviz_viewer::intMatrix cost)
        : OptimizationObjective(si),
          cost_(cost)
      {
        description_ = "Cost Map";
      };

      /** \brief Defines motion cost */
      virtual Cost motionCost(const State *s1, const State *s2) const
      {
        // Only accrue positive changes in cost
        double positiveCostAccrued = std::max(stateCost(s2).v - stateCost(s1).v, 0.0);
        return Cost(positiveCostAccrued); // + pathLengthWeight_*si_->distance(s1,s2));
      };

      ompl::base::Cost stateCost(const State *state) const
      {
        const double *coords = state->as<ob::RealVectorStateSpace::StateType>()->values;

        // Return the cost from the matrix at the current dimensions
        double cost = cost_( nat_round(coords[1]), nat_round(coords[0]) );

        return Cost(cost);
      }

      /** \brief Passed in a cost matrix loaded from an image file, etc */
      void setCostMatrix(ompl_rviz_viewer::intMatrix cost)
      {
        cost_ = cost;
      };

      /**
       * \brief Nat_Rounding helper function to make readings from cost map more accurate
       * \param double
       * \return rounded down number
       */
      int nat_round(double x) const
      {
        return static_cast<int>(floor(x + 0.5f));
      };

    protected:

      // The cost for each x,y - which is derived from the RGB data
      ompl_rviz_viewer::intMatrix cost_;
    };

    typedef boost::shared_ptr< CostMapOptimizationObjective > CostMapOptimizationObjectivePtr;
  }
}

#endif
