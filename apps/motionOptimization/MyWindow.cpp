/*
 * Copyright (c) 2015, Georgia Tech Research Corporation
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

#include "MyWindow.h"

#include <iostream>

using namespace std;
using namespace Eigen;
using namespace dart;

//==============================================================================
MyWindow::MyWindow(dynamics::Skeleton* _robot)
    : SimWindow()
{
  assert(nullptr != _robot);

  _robot->getJoint("shoulder_yaw")->setPosition(0, DART_RADIAN * 180.0);

  mMotionOptimizer.reset(new MotionOptimizer(_robot, 3));
  mMotionOptimizer->setInitialPose(_robot->getPositions());
  mMotionOptimizer->setTargetPositionOfEndEffector(Vector3d(0.5, -0.1, 0.1));
  mMotionOptimizer->setFinalTime(1.0);
}

//==============================================================================
MyWindow::~MyWindow()
{
}

//==============================================================================
void MyWindow::timeStepping()
{
  switch (mMode)
  {
    case NONE:
    {
      // do nothing
      break;
    }
    case PLAY_OPTIMIZED_MOTION:
    {
      if (mWorld->getTime() < mMotionOptimizer->getFinalTime())
      {
        mMotionOptimizer->setCommandsToRobot(mWorld->getTime());
        mWorld->step(false);
      }
      else
      {
        mMode = NONE;
        dtmsg << "MyWindow: Finished the optimized motion." << endl;
      }
      break;
    }
  }
}

//==============================================================================
void MyWindow::drawSkels()
{
  // Draw the target position
  if (mRI)
  {
    mRI->setPenColor(Vector3d(0.8, 0.2, 0.2));
    mRI->pushMatrix();
    mRI->translate(mMotionOptimizer->getTargetPositionOfEndEffector());
    mRI->drawEllipsoid(Vector3d(0.05, 0.05, 0.05));
    mRI->popMatrix();
  }

  // Draw skeletons
  SimWindow::drawSkels();
}

//==============================================================================
void MyWindow::keyboard(unsigned char _key, int _x, int _y)
{
  switch (_key)
  {
    case 'o':
    {
      dtmsg << "MyWindow: Motion optimizing..." << endl;
      mMotionOptimizer->optimize();
      mMotionOptimizer->setToInitialPose();
      dtmsg << "MyWindow: Motion optimized." << endl;
      break;
    }
    case 'r':
    {
      mMode = PLAY_OPTIMIZED_MOTION;
      mMotionOptimizer->setToInitialPose();
      mWorld->reset();
      dtmsg << "MyWindow: Playing the optimized motion..." << endl;
      break;
    }
    default:
    {
      // Default keyboard control
      SimWindow::keyboard(_key, _x, _y);
      break;
    }
  }

  glutPostRedisplay();
}

