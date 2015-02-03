/*
 * Copyright (c) 2011-2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Karen Liu <karenliu@cc.gatech.edu>,
 *            Jeongseok Lee <jslee02@gmail.com>
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

#include "apps/rigidCubes/MyWindow.h"

#include "dart/math/Helpers.h"
#include "dart/simulation/World.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/FreeJoint.h"
#include "dart/dynamics/BoxShape.h"
#include "dart/dynamics/MeshShape.h"
#include "dart/utils/Paths.h"
#include "dart/utils/SkelParser.h"

MyWindow::MyWindow()
  : SimWindow() {
  mForce = Eigen::Vector3d::Zero();
}

MyWindow::~MyWindow() {
}

void MyWindow::timeStepping() {
  //mWorld->getSkeleton(1)->getBodyNode(0)->addExtForce(mForce);


  // remove skeleton
  if (mWorld->getNumSkeletons() > 1) {
    mWorld->removeSkeleton(mWorld->getSkeleton(0));
    mWorld->removeSkeleton(mWorld->getSkeleton(1));
  }
  // add new skeleton
  std::string filename1 = DART_DATA_PATH"/skel/rigidclothpatch1.skel";
  dart::dynamics::Skeleton* patch1 = dart::utils::SkelParser::readSkeleton(filename1);
  mWorld->addSkeleton(patch1);
  std::string filename2 = DART_DATA_PATH"/skel/rigidclothpatch2.skel";
  dart::dynamics::Skeleton* patch2 = dart::utils::SkelParser::readSkeleton(filename2);
  mWorld->addSkeleton(patch2);

  // set dimension
  Eigen::Vector3d dim1(0.0160333, 0.01, 0.00802094);
  dart::dynamics::BoxShape* colShape1 = (dart::dynamics::BoxShape*)mWorld->getSkeleton(0)->getBodyNode(0)->getCollisionShape(0);
  colShape1->setSize(dim1);
  dart::dynamics::BoxShape* visShape1 = (dart::dynamics::BoxShape*)mWorld->getSkeleton(0)->getBodyNode(0)->getVisualizationShape(0);
  visShape1->setSize(dim1);

  Eigen::Vector3d dim2(0.0147092, 0.01, 0.0094610);
  dart::dynamics::BoxShape* colShape2 = (dart::dynamics::BoxShape*)mWorld->getSkeleton(1)->getBodyNode(0)->getCollisionShape(0);
  colShape2->setSize(dim2);
  dart::dynamics::BoxShape* visShape2 = (dart::dynamics::BoxShape*)mWorld->getSkeleton(1)->getBodyNode(0)->getVisualizationShape(0);
  visShape2->setSize(dim2);

  Eigen::VectorXd state1(12);
  state1 << -1.43395,  -1.24326,  0.607935,  0.722328, 0.0256898,  0.154474, 0, 0, 0, 0, 0, 0;
  mWorld->getSkeleton(0)->setState(state1);
  mWorld->getSkeleton(0)->computeForwardKinematics(true,false,false);
  Eigen::VectorXd state2(12);
  state2 << -1.17932, -0.666113,  0.568555,  0.639145, 0.0779996,  0.312032, 0, 0, 0, 0, 0, 0;
  mWorld->getSkeleton(1)->setState(state2);
  mWorld->getSkeleton(1)->computeForwardKinematics(true,false,false);


  mWorld->step();
//  mForce /= 2.0;
}

void MyWindow::drawSkels() {
  glEnable(GL_LIGHTING);
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

  SimWindow::drawSkels();
}

void MyWindow::keyboard(unsigned char _key, int _x, int _y) {
  switch (_key) {
    case ' ':  // use space key to play or stop the motion
      mSimulating = !mSimulating;
      if (mSimulating) {
        mPlay = false;
        glutTimerFunc(mDisplayTimeout, refreshTimer, 0);
      }
      break;
    case 'p':  // playBack
      mPlay = !mPlay;
      if (mPlay) {
        mSimulating = false;
        glutTimerFunc(mDisplayTimeout, refreshTimer, 0);
      }
      break;
    case '[':  // step backward
      if (!mSimulating) {
        mPlayFrame--;
        if (mPlayFrame < 0)
          mPlayFrame = 0;
        glutPostRedisplay();
      }
      break;
    case ']':  // step forwardward
      if (!mSimulating) {
        mPlayFrame++;
        if (mPlayFrame >= mWorld->getRecording()->getNumFrames())
          mPlayFrame = 0;
        glutPostRedisplay();
      }
      break;
    case 'v':  // show or hide markers
      mShowMarkers = !mShowMarkers;
      break;
    case '1':  // upper right force
      mForce[0] = -500;
      break;
    case '2':  // upper right force
      mForce[0] = 500;
      break;
    case '3':  // upper right force
      mForce[2] = -500;
      break;
    case '4':  // upper right force
      mForce[2] = 500;
      break;
    default:
      Win3D::keyboard(_key, _x, _y);
  }
  glutPostRedisplay();
}
