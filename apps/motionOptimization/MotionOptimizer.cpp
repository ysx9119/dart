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

#include "MotionOptimizer.h"

#include <iostream>

using namespace std;
using namespace Eigen;
using namespace dart;
using namespace dynamics;
using namespace optimizer;

//==============================================================================
MotionOptimizer::MotionOptimizer(dynamics::Skeleton* _robot, size_t _numCPs)
  : mRobot(_robot),
    mNumCPs(_numCPs),
    mMotion(new Motion(_robot, _numCPs + 1)) // the first control point is fixed
{
  assert(nullptr != _robot);

  for (size_t i = 0; i < mRobot->getNumBodyNodes(); ++i)
  {
    Joint* joint = mRobot->getJoint(i);
    joint->setActuatorType(Joint::VELOCITY);
  }

  mInitialPose = VectorXd::Zero(mRobot->getNumDofs());

  mMotion->setFinalTime(3.0);
}

//==============================================================================
MotionOptimizer::~MotionOptimizer()
{
  delete mMotion;
}

//==============================================================================
Skeleton* MotionOptimizer::getSkeleton()
{
  return mRobot;
}

//==============================================================================
const Skeleton* MotionOptimizer::getSkeleton() const
{
  return mRobot;
}

//==============================================================================
void MotionOptimizer::optimize()
{
  size_t numDofs = mRobot->getNumDofs();
  size_t dim = mNumCPs * numDofs;

  // Bounds and initial guess
  VectorXd lb = VectorXd::Zero(dim);
  VectorXd ub = VectorXd::Zero(dim);
  VectorXd x0 = VectorXd::Zero(dim);

  size_t index = 0;

  for (size_t i = 0; i < numDofs; ++i)
  {
    dynamics::DegreeOfFreedom* dof = mRobot->getDof(i);

    double qMin = dof->getPositionLowerLimit();
    double qMax = dof->getPositionUpperLimit();

    for (size_t j = 0; j < mNumCPs; ++j)
    {
      lb[index] = qMin;
      ub[index] = qMax;
      x0[index] = 0.5 * (lb[index] + ub[index]);
      index++;
    }
  }
  assert(index == dim);

  // Objective -- minimum joint torque
  ObjJointTorque obj(this);

  // Constraints1 -- target position of the end-effector
  ConstTargetPosition constTargetTangent(this, mTargetPositionOfEndEffector);

  // Constraints2 -- joint torque limit
  ConstJointTorqueLimit consJointLimit(this);

  // Problem
  dart::optimizer::Problem prob(dim);
  prob.setInitialGuess(x0);
  prob.setLowerBounds(lb);
  prob.setUpperBounds(ub);
  prob.setObjective(&obj);
  prob.addEqConstraint(&constTargetTangent);
//  prob.addIneqConstraint(&consJointLimit);

  // Solver
  NloptSolver solver(&prob, NLOPT_LN_COBYLA);
  solver.setNumMaxEvaluations(1e+3);
  solver.solve();

  // Result
  double minFunc = prob.getOptimumValue();
  Eigen::VectorXd optX = prob.getOptimalSolution();

  // Set optimized variables
  /*mResult = */evaluate(optX, false);

  std::cout << "cost       : " << minFunc << std::endl;
//  std::cout << "axis const : " << (mResult.rotation - getTargetAxis()).norm()
//            << std::endl;
//  std::cout << "optimized x: " << optX.transpose() << std::endl;
//  mResult.print(mTargetAxis);
}

//==============================================================================
void MotionOptimizer::setInitialPose(const VectorXd& _pose)
{
  mInitialPose = _pose;
}

//==============================================================================
const VectorXd& MotionOptimizer::getInitialPose() const
{
  return mInitialPose;
}

//==============================================================================
size_t MotionOptimizer::getNumCPs() const
{
  return mNumCPs;
}

//==============================================================================
void MotionOptimizer::setToInitialPose()
{
  mRobot->setPositions(getInitialPose());
}

//==============================================================================
void MotionOptimizer::setCommandsToRobot(double _time)
{
  mRobot->setCommands(getCommands(_time));
}

//==============================================================================
VectorXd MotionOptimizer::getCommands(double _time) const
{
  size_t numDofs = mRobot->getNumDofs();
  Eigen::VectorXd cmds = Eigen::VectorXd::Zero(numDofs);

  for (size_t i = 0; i < numDofs; ++i)
  {
    const dynamics::DegreeOfFreedom* dof = mRobot->getDof(i);
    size_t index = dof->getIndexInSkeleton();
    cmds[index] = mMotion->getVelocity(i, _time);
  }

  return cmds;
}

//==============================================================================
void MotionOptimizer::setVariables(const VectorXd& _x)
{
  for (size_t i = 0; i < mRobot->getNumDofs(); ++i)
  {
    for (size_t j = 0; j < mNumCPs; ++j)
    {
      // The first control points are fixed, which means they are not
      // free (optimization) variables.
      mMotion->setControlPoint(i, j + 1, _x[i * mNumCPs + j]);
    }
  }

//  mMotion->printCPs();
//  std::cout << "evaluating CPs : " << _x.head(_x.size() - 1).transpose()
//            << std::endl;
  //  std::cout << "evaluating time: " << _x.tail(1) << std::endl;
}

//==============================================================================
void MotionOptimizer::doFowardSimulation(const VectorXd& _x)
{
  // Set motion and final time
  setVariables(_x);

  // Time
  double time = 0.0;
  double tf = getFinalTime();

  // Init robot
  dynamics::Skeleton* skel = getSkeleton();
  int dof = skel->getNumDofs();
  Eigen::VectorXd  q0 = getInitialPose();
  Eigen::VectorXd dq0 = Eigen::VectorXd::Zero(dof);
  skel->setPositions(q0);
  skel->setVelocities(dq0);

  // Forward dynamics simulation
  while (time < tf)
  {
    Eigen::VectorXd cmds = getCommands(time);

    //    std::cout << "cmd: " << cmds.transpose() << std::endl;

    skel->setCommands(cmds);

    skel->computeForwardDynamicsRecursionPartB();
    skel->integrateVelocities(skel->getTimeStep());
    skel->integratePositions(skel->getTimeStep());
    skel->computeForwardDynamicsRecursionPartA();

    time += skel->getTimeStep();
  }

//  std::cout << "qf: " << skel->getPositions().transpose() << std::endl;
}

//==============================================================================
std::vector<VectorXd> MotionOptimizer::doFowardSimulationWithToqueRecord(
    const VectorXd& _x)
{
  std::vector<Eigen::VectorXd> torque;

  // Set motion and final time
  setVariables(_x);

  // Time
  double time = 0.0;
  double tf = getFinalTime();

  // Init robot
  dynamics::Skeleton* skel = getSkeleton();
  int dof = skel->getNumDofs();
  Eigen::VectorXd  q0 = getInitialPose();
  Eigen::VectorXd dq0 = Eigen::VectorXd::Zero(dof);
  skel->setPositions(q0);
  skel->setVelocities(dq0);

  // Forward dynamics simulation
  while (time < tf)
  {
    Eigen::VectorXd cmds = getCommands(time);

    //    std::cout << "cmd: " << cmds.transpose() << std::endl;

    skel->setCommands(cmds);

    skel->computeForwardDynamicsRecursionPartB();
    torque.push_back(skel->getForces());
    skel->integrateVelocities(skel->getTimeStep());
    skel->integratePositions(skel->getTimeStep());
    skel->computeForwardDynamicsRecursionPartA();

    time += skel->getTimeStep();
  }

  return torque;
}

//==============================================================================
MotionOptimizer::Result MotionOptimizer::evaluate(const VectorXd& _x,
                                                  bool _recordTorque)
{
  assert(_x.size() > 0);

  Result res;

  if (_recordTorque)
    res.torques = doFowardSimulationWithToqueRecord(_x);
  else
    doFowardSimulation(_x);

  // Evaluate cost with the total effort
  res.x = _x;
  res.positionOfEndEffector
      = mRobot->getBodyNode("palm")->getTransform().translation();

  return res;
}

//==============================================================================
void MotionOptimizer::setTargetPositionOfEndEffector(const Vector3d& _pos)
{
  mTargetPositionOfEndEffector = _pos;
}

//==============================================================================
const Vector3d& MotionOptimizer::getTargetPositionOfEndEffector() const
{
  return mTargetPositionOfEndEffector;
}

//==============================================================================
void MotionOptimizer::setFinalTime(double _time)
{
  mMotion->setFinalTime(_time);
}

//==============================================================================
double MotionOptimizer::getFinalTime() const
{
  return mMotion->getFinalTime();
}

//==============================================================================
MotionOptimizer::ObjJointTorque::ObjJointTorque(MotionOptimizer* _motionOptimizer)
  : Function(),
    mMotionOptimizer(_motionOptimizer)
{
  assert(mMotionOptimizer != nullptr);
}

//==============================================================================
MotionOptimizer::ObjJointTorque::~ObjJointTorque()
{}

//==============================================================================
double MotionOptimizer::ObjJointTorque::eval(Map<const VectorXd>& _x)
{
  static int count = 0;

//  std::cout << "(" << count << "): " << _x.transpose() << std::endl;

  // Evaluate with x
  Result res = mMotionOptimizer->evaluate(_x, true);

  // Minimizing torque
  double eval = 0.0;

  dynamics::Skeleton* skel = mMotionOptimizer->getSkeleton();
  const double timeStep = skel->getTimeStep();
  const size_t numDofs  = skel->getNumDofs();

  for (const auto& torque : res.torques)
  {
    for (size_t i = 0; i < numDofs; ++i)
      eval += timeStep * fabs(torque[i]);
  }

//  if (count % 100 == 0)
//  {
//    std::cout << "eval num: " << count << std::endl;
//    std::cout << "cost    : " << eval << std::endl;
//    std::cout << std::endl;
//  }

  count++;

  return eval;
}

//==============================================================================
MotionOptimizer::ConstTargetPosition::ConstTargetPosition(MotionOptimizer* _motionOptimizer,
    const Vector3d& _pos)
  : Function(),
    mMotionOptimizer(_motionOptimizer),
    mTargetPositionOfEndEffector(_pos)
{
  assert(mMotionOptimizer != nullptr);
}

//==============================================================================
MotionOptimizer::ConstTargetPosition::~ConstTargetPosition()
{}

//==============================================================================
double MotionOptimizer::ConstTargetPosition::eval(
    Map<const VectorXd>& _x)
{
  static int count = 0;

//  std::cout << "(" << count << "): " << _x.transpose() << std::endl;

  // Evaluate with x
  Result res = mMotionOptimizer->evaluate(_x, false);

  double w = 1.0;

  // Maximizing angle and minimizing time
  double eval = w * (res.positionOfEndEffector - mTargetPositionOfEndEffector).norm();

//  std::cout << "target: " << mTargetPositionOfEndEffector.transpose() << std::endl;
//  std::cout << "current: " << res.positionOfEndEffector.transpose() << std::endl;

//  if (count%100 == 0)
//  {
//    std::cout << "const1 eval num: " << count << std::endl;
//    std::cout << "const1 norm    : " << eval << std::endl;
//    std::cout << std::endl;
//  }

  count++;

  return eval;
}

//==============================================================================
MotionOptimizer::ConstJointTorqueLimit::ConstJointTorqueLimit(
    MotionOptimizer* _motionOptimizer)
  : Function(),
    mMotionOptimizer(_motionOptimizer)
{
  assert(mMotionOptimizer != nullptr);
}

//==============================================================================
MotionOptimizer::ConstJointTorqueLimit::~ConstJointTorqueLimit()
{

}

//==============================================================================
double MotionOptimizer::ConstJointTorqueLimit::eval(
    Eigen::Map<const VectorXd>& _x)
{
  static int count = 0;

//  std::cout << "(" << count << "): " << _x.transpose() << std::endl;

  // Evaluate with x
  Result res = mMotionOptimizer->evaluate(_x, true);

  // Minimizing torque
  double eval = 0.0;

  dynamics::Skeleton* skel = mMotionOptimizer->getSkeleton();
  const double timeStep = skel->getTimeStep();
  const size_t numDofs  = skel->getNumDofs();

  for (const auto& torque : res.torques)
  {
    for (size_t i = 0; i < numDofs; ++i)
    {
      if (torque[i] > skel->getForceUpperLimit(i))
        eval += timeStep * (torque[i] - skel->getForceUpperLimit(i));

      if (torque[i] < skel->getForceLowerLimit(i))
        eval += timeStep * (skel->getForceLowerLimit(i) - torque[i]);
    }
  }

//  if (count % 100 == 0)
//  {
//    std::cout << "const2 eval num: " << count << std::endl;
//    std::cout << "const2 norm    : " << eval << std::endl;
//    std::cout << std::endl;
//  }

  count++;

  return eval;
}
