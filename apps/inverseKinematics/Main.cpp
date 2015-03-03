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

#include <iostream>
#include <memory>

#include "dart/dart.h"

#include "MyWindow.h"
#include "IKSolver.h"

using namespace std;
using namespace dart;

int main(int argc, char* argv[])
{
  // Create and initialize the world
  simulation::World world;

  // Load skeletons
  utils::DartLoader dl;
  dynamics::Skeleton* ground
          = dl.parseSkeleton(DART_DATA_PATH"urdf/KR5/ground.urdf");
  dynamics::Skeleton* robot
          = dl.parseSkeleton(DART_DATA_PATH"urdf/KR5/KR5 sixx R650.urdf");
  world.addSkeleton(ground);
  world.addSkeleton(robot);

  // Create a window and link it to the world
  IKSolver ikSolver(robot, robot->getBodyNode("palm"));
  MyWindow window(&ikSolver);
  window.setWorld(&world);

  cout << "space bar: simulation on/off" << endl;
  cout << "'p': playback/stop" << endl;
  cout << "'[' and ']': play one frame backward and forward" << endl;
  cout << "'v': visualization on/off" << endl;
  cout << "'1'--'6': programmed interaction" << endl;

  glutInit(&argc, argv);
  window.initWindow(640, 480, "Inverse Kinematics");
  glutMainLoop();

  return 0;
}
