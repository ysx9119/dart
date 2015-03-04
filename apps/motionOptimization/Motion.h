/*
 * Copyright (c) 2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef APPS_MOTIONOPTIMIZATION_MOTION_H_
#define APPS_MOTIONOPTIMIZATION_MOTION_H_

#include <Eigen/Dense>

#include "dart/dart.h"

/// Motion
class Motion
{
public:
  /// Constructor
  Motion(dart::dynamics::Skeleton* _skel,
         size_t _numCPs = 4,
         double _time = 1.0);

  /// Destructor
  virtual ~Motion();

  /// Set final time
  void setFinalTime(double _time);

  /// Get final time
  double getFinalTime() const;

  /// Get control point
  double getControlPoint(int _i, int _j) const;

  /// Set control point
  void setControlPoint(int _i, int _j, double _val);

  /// Get position
  double getPosition(int i, double _t) const;

  /// Get velocity
  double getVelocity(int i, double _t) const;

  /// Get acceleration
  double getAcceleration(int i, double _t) const;

  /// Print control points
  void printCPs() const;

  /// Print plot data
  void printPlotData() const;

protected:
  /// Number of degree of freedom
  static const size_t mNumJoint = 6;

  /// B-spline
  dart::math::BSpline<double, mNumJoint> mSplines;
  // TODO: The dimension of BSpline cannot be changed in runtime due to the
  //       limitaion of its implementation.

  /// Final time
  double mFinalTime;
};

#endif  // APPS_MOTIONOPTIMIZATION_MOTION_H_
