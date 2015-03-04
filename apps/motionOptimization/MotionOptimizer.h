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

#ifndef APPS_MOTIONOPTIMIZATION_MOTIONOPTIMIZER_H_
#define APPS_MOTIONOPTIMIZATION_MOTIONOPTIMIZER_H_

#include <Eigen/Dense>

#include "dart/dart.h"

#include "Motion.h"

/// MotionOptimizer
class MotionOptimizer
{
public:
  /// Constructor
  MotionOptimizer(dart::dynamics::Skeleton* _robot, size_t _numCPs = 4);

  /// Destructor
  virtual ~MotionOptimizer();

  /// Get target robot
  dart::dynamics::Skeleton* getSkeleton();

  /// Get target robot
  const dart::dynamics::Skeleton* getSkeleton() const;

  /// Optimize
  void optimize();

  /// Get motion
  Motion* getMotion() { return mMotion; }

  /// Set initial pose of the robot
  void setInitialPose(const Eigen::VectorXd& _pose);

  /// Get initial pose of the robot
  const Eigen::VectorXd& getInitialPose() const;

  /// Set target position of end-effector
  void setTargetPositionOfEndEffector(const Eigen::Vector3d& _pos);

  /// Get target position of end-effector
  const Eigen::Vector3d& getTargetPositionOfEndEffector() const;

  /// Set time for the motion
  void setFinalTime(double _time);

  /// Get time for the motion
  double getFinalTime() const;

  /// Get number of control points for each dof of the robot
  size_t getNumCPs() const;

  /// Set the robot to its initial pose
  void setToInitialPose();

  /// Set the robot to optimized motion given time
  void setCommandsToRobot(double _time);

  /// Return command for the skeleton
  Eigen::VectorXd getCommands(double _time) const;

protected:

  friend class ObjJointTorque;
  friend class ConstTargetPosition;
  friend class ConstJointTorqueLimit;

  class ObjJointTorque : public dart::optimizer::Function
  {
  public:
    /// Constructor
    ObjJointTorque(MotionOptimizer* _motionOptimizer);

    /// Destructor
    virtual ~ObjJointTorque();

    // Documentation inherited
    virtual double eval(Eigen::Map<const Eigen::VectorXd>& _x) override;

  protected:
    /// Motion optimizer
    MotionOptimizer* mMotionOptimizer;
  };

  class ConstTargetPosition : public dart::optimizer::Function
  {
  public:
    /// Constructor
    ConstTargetPosition(MotionOptimizer* _motionOptimizer,
                        const Eigen::Vector3d& _pos);

    /// Destructor
    virtual ~ConstTargetPosition();

    // Documentation inherited
    virtual double eval(Eigen::Map<const Eigen::VectorXd>& _x) override;

  protected:
    /// Motion optimizer
    MotionOptimizer* mMotionOptimizer;

    /// Final position of the end-effector
    Eigen::Vector3d mTargetPositionOfEndEffector;
  };

  class ConstJointTorqueLimit : public dart::optimizer::Function
  {
  public:
    /// Constructor
    ConstJointTorqueLimit(MotionOptimizer* _motionOptimizer);

    /// Destructor
    virtual ~ConstJointTorqueLimit();

    // Documentation inherited
    virtual double eval(Eigen::Map<const Eigen::VectorXd>& _x) override;

  private:
    MotionOptimizer* mMotionOptimizer;
  };

  struct Result
  {
    ///
    Eigen::VectorXd x;

    ///
    Eigen::Vector3d positionOfEndEffector;

    ///
    std::vector<Eigen::VectorXd> torques;

    ///
//    void print(const Eigen::Vector3d& _targetAxis);
  };

  /// Set optimization variables
  void setVariables(const Eigen::VectorXd& _x);

  /// Do forward simulation
  void doFowardSimulation(const Eigen::VectorXd& _x);

  /// Do forward simulation and returns sequence of joint torques which is a
  /// result of the simulation
  std::vector<Eigen::VectorXd> doFowardSimulationWithToqueRecord(
      const Eigen::VectorXd& _x);

  /// Evaulate simulation results given optimization variables
  Result evaluate(const Eigen::VectorXd& _x, bool _recordTorque);

  /// Robot whose motion will be optimized
  dart::dynamics::Skeleton* mRobot;

  /// Initial pose of the robot
  Eigen::VectorXd mInitialPose;

  /// Final position of the end-effector
  Eigen::Vector3d mTargetPositionOfEndEffector;

  /// Motion
  Motion* mMotion;

  /// Number of control points for each dof of the robot
  size_t mNumCPs;
};

#endif  // APPS_MOTIONOPTIMIZATION_MOTIONOPTIMIZER_H_
