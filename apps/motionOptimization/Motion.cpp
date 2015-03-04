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

#include "Motion.h"

#include <iostream>
#include <fstream>

using namespace std;
using namespace Eigen;
using namespace dart;

//==============================================================================
Motion::Motion(dynamics::Skeleton* _skel, size_t _numCPs, double _time)
  : mSplines(3, _numCPs, 0.0, _time)
{
//  setControlPoint(0, 1, 1);
//  setControlPoint(0, 2, 2);
//  setControlPoint(0, 3, -2);
//  setControlPoint(0, 4, -1);
//  std::cout << "CPs: " << mSplines.getControlPoints() << std::endl;
//  std::cout << "Knots: " << mSplines.getKnots().transpose() << std::endl;

  setFinalTime(_time);
}

//==============================================================================
Motion::~Motion()
{
}

//==============================================================================
void Motion::setFinalTime(double _time)
{
  mFinalTime = _time;
  mSplines.setUniformKnots(0.0, _time, true);
}

//==============================================================================
double Motion::getFinalTime() const
{
  return mFinalTime;
}

//==============================================================================
double Motion::getControlPoint(int _i, int _j) const
{
  return mSplines.getControlPoint(_i, _j);
}

//==============================================================================
void Motion::setControlPoint(int _i, int _j, double _val)
{
  mSplines.setControlPoint(_i, _j, _val);
}

//==============================================================================
double Motion::getPosition(int i, double _t) const
{
  return mSplines.getPosition(_t)[i];
}

//==============================================================================
double Motion::getVelocity(int i, double _t) const
{
  return mSplines.getVelocity(_t)(i, 1);
}

//==============================================================================
double Motion::getAcceleration(int i, double _t) const
{
  return mSplines.getAcceleration(_t)(i, 2);
}

//==============================================================================
void Motion::printCPs() const
{
  std::cout << "=================================================" << std::endl;
  std::cout << mSplines.getControlPoints() << std::endl;
  std::cout << "=================================================" << std::endl;
}

//==============================================================================
void Motion::printPlotData() const
{
  std::string _fileName = "motion.txt";

  ofstream file[mNumJoint];

  std::string fullName = std::string(DART_DATA_PATH) + _fileName;

  for (int i = 0; i < mNumJoint; ++i)
    file[i].open(fullName.c_str() + std::to_string(i));

  const double dt = 0.001;
  const double t0 = mSplines.getKnot(0);
  const double t1 = mSplines.getKnot(mSplines.getNumKnots() - 1);

  const size_t numFrames = (t1 - t0) / dt;

  for (size_t i = 0; i < mNumJoint; ++i)
  {
    for (size_t j = 0; j < numFrames; ++j)
    {
      const double t = t0 + dt * j;
      const double pos = mSplines.getPosition(t)[i];
      const double vel = mSplines.getVelocity(t)(i, 1);
      const double acc = mSplines.getAcceleration(t)(i, 2);

      file[i] << t << " " << pos << " " << vel << " " << acc << std::endl;
    }
  }

  std::cout << "Printed plot data." << std::endl;

  for (int i = 0; i < mNumJoint; ++i)
    file[i].close();
}
