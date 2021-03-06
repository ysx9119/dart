/*
 * Copyright (c) 2014-2015, Georgia Tech Research Corporation
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

#ifndef DART_OPTIMIZER_IPOPT_IPOPTSOLVER_H_
#define DART_OPTIMIZER_IPOPT_IPOPTSOLVER_H_

//------------------------------------------------------------------------------
// Workaround for bug:
// (see https://bugs.debian.org/cgi-bin/bugreport.cgi?bug=684062)
// This is fixed at IpOpt 3.11.4-1
#define HAVE_CSTDDEF
#include <coin/IpTNLP.hpp>
#include <coin/IpIpoptApplication.hpp>
#undef HAVE_CSTDDEF
//------------------------------------------------------------------------------

#include "dart/optimizer/Solver.h"

namespace dart {
namespace optimizer {

class Problem;
class DartTNLP;

/// \brief class IpoptSolver
class IpoptSolver : public Solver
{
public:
  /// \brief Constructor
  explicit IpoptSolver(Problem* _problem);

  /// \brief Destructor
  virtual ~IpoptSolver();

  /// \copydoc Solver::solve
  virtual bool solve();

private:
  /// \brief IPOPT nonlinear programming problem
  Ipopt::SmartPtr<Ipopt::TNLP> mNlp;

  /// \brief Main application class for making calls to Ipopt
  Ipopt::SmartPtr<Ipopt::IpoptApplication> mIpoptApp;
};

/// \brief class DartTNLP
class DartTNLP : public Ipopt::TNLP
{
public:
  /// \brief
  explicit DartTNLP(Problem* _problem);

  /// \brief
  virtual ~DartTNLP();

  //------------------------- Ipopt::TNLP --------------------------------------
  /// \brief Method to return some info about the nlp
  virtual bool get_nlp_info(Ipopt::Index& n,
                            Ipopt::Index& m,
                            Ipopt::Index& nnz_jac_g,
                            Ipopt::Index& nnz_h_lag,
                            Ipopt::TNLP::IndexStyleEnum& index_style);

  /// \brief Method to return the bounds for my problem
  virtual bool get_bounds_info(Ipopt::Index n,
                               Ipopt::Number* x_l,
                               Ipopt::Number* x_u,
                               Ipopt::Index m,
                               Ipopt::Number* g_l,
                               Ipopt::Number* g_u);

  /// \brief Method to return the starting point for the algorithm
  virtual bool get_starting_point(Ipopt::Index n,
                                  bool init_x,
                                  Ipopt::Number* x,
                                  bool init_z,
                                  Ipopt::Number* z_L,
                                  Ipopt::Number* z_U,
                                  Ipopt::Index m,
                                  bool init_lambda,
                                  Ipopt::Number* lambda);

  /// \brief Method to return the objective value
  virtual bool eval_f(Ipopt::Index _n,
                      const Ipopt::Number* _x,
                      bool _new_x,
                      Ipopt::Number&
                      _obj_value);

  /// \brief Method to return the gradient of the objective
  virtual bool eval_grad_f(Ipopt::Index _n,
                           const Ipopt::Number* _x,
                           bool _new_x,
                           Ipopt::Number* _grad_f);

  /// \brief Method to return the constraint residuals
  virtual bool eval_g(Ipopt::Index _n,
                      const Ipopt::Number* _x,
                      bool _new_x,
                      Ipopt::Index _m,
                      Ipopt::Number* _g);

  /// \brief Method to return:
  ///        1) The structure of the jacobian (if "values" is NULL)
  ///        2) The values of the jacobian (if "values" is not NULL)
  virtual bool eval_jac_g(Ipopt::Index _n,
                          const Ipopt::Number* _x,
                          bool _new_x,
                          Ipopt::Index _m,
                          Ipopt::Index _nele_jac,
                          Ipopt::Index* _iRow,
                          Ipopt::Index* _jCol,
                          Ipopt::Number* _values);

  /// \brief Method to return:
  ///        1) The structure of the hessian of the lagrangian (if "values" is
  ///           NULL)
  ///        2) The values of the hessian of the lagrangian (if "values" is not
  ///           NULL)
  virtual bool eval_h(Ipopt::Index _n,
                      const Ipopt::Number* _x,
                      bool _new_x,
                      Ipopt::Number _obj_factor,
                      Ipopt::Index _m,
                      const Ipopt::Number* _lambda,
                      bool _new_lambda,
                      Ipopt::Index _nele_hess,
                      Ipopt::Index* _iRow,
                      Ipopt::Index* _jCol,
                      Ipopt::Number* _values);

  /// \brief This method is called when the algorithm is complete so the TNLP
  ///        can store/write the solution
  virtual void finalize_solution(Ipopt::SolverReturn _status,
                                 Ipopt::Index _n,
                                 const Ipopt::Number* _x,
                                 const Ipopt::Number* _z_L,
                                 const Ipopt::Number* _z_U,
                                 Ipopt::Index _m,
                                 const Ipopt::Number* _g,
                                 const Ipopt::Number* _lambda,
                                 Ipopt::Number _obj_value,
                                 const Ipopt::IpoptData* _ip_data,
                                 Ipopt::IpoptCalculatedQuantities* _ip_cq);

private:
  /// \brief DART optimization problem
  Problem* mProblem;

  /// \brief Objective value
  Ipopt::Number mObjValue;

  /// \brief Objective gradient
  Eigen::VectorXd mObjGradient;

  /// \brief Objective Hessian
  Eigen::MatrixXd mObjHessian;
};

}  // namespace optimizer
}  // namespace dart

#endif  // DART_OPTIMIZER_IPOPT_IPOPTSOLVER_H_

