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
   Desc:   Validity checker that supports cost
*/

// Boost
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>

// OMPL
#include <ompl/base/SpaceInformation.h>

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

} // namespace
