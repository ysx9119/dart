#include "IKSolver.h"

using namespace std;
using namespace Eigen;
using namespace dart;

//==============================================================================
IKSolver::IKSolver(dynamics::Skeleton* _robot, dynamics::BodyNode* _endEffector)
  : mRobot(_robot),
    mEndEffector(_endEffector),
    mNumIteration(200)
{
  assert(nullptr != _robot);
  assert(nullptr != _endEffector);
}

//==============================================================================
IKSolver::~IKSolver()
{

}

//==============================================================================
dynamics::Skeleton* IKSolver::getRobot() const
{
  return mRobot;
}

//==============================================================================
dynamics::BodyNode* IKSolver::getEndEffector() const
{
  return mEndEffector;
}

//==============================================================================
void IKSolver::solve(const Vector3d& _target)
{
  VectorXd newPose;

  size_t numIter = 0;
  while (numIter <= mNumIteration)
  {
    Vector3d diff = mEndEffector->getCOM() - _target;

    if (diff.norm() < 1e-6)
      break;

    MatrixXd J = mEndEffector->getLinearJacobian(dynamics::Frame::World());
    newPose = mRobot->getPositions() - 0.1 * 2 * J.transpose() * diff;
    mRobot->setPositions(newPose);

    numIter++;
  }
}

//==============================================================================
void IKSolver::keyboard(unsigned char _key, int _x, int _y)
{

}
