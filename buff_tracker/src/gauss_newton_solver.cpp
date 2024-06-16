// Copyright (C) 2024 Zheng Yu
// Licensed under the MIT License.

#include "buff_tracker/gauss_newton_solver.hpp"

namespace rm_buff
{
GaussNewtonSolver::GaussNewtonSolver(
  const VecMatFunc & u_fx, const VecMatFunc & u_J, const BoolMatFunc & constraint, int max_iter,
  double min_step, int obs_max_size)
: u_fx_(u_fx),
  u_J_(u_J),
  constraint_(constraint),
  max_iter_(max_iter),
  min_step_(min_step),
  obs_max_size_(obs_max_size)
{
}

void GaussNewtonSolver::addObservation(double x, double y)
{
  std::vector<double> ob;
  ob.push_back(x);
  ob.push_back(y);
  obs.push_back(ob);
  if (int(obs.size()) > obs_max_size_) {
    obs.erase(obs.begin());
  }
}

void GaussNewtonSolver::setStartValue(const Eigen::VectorXd & x) { x_ = x; }

GaussNewtonSolver::SolverStatus GaussNewtonSolver::solve()
{
  fx_.resize(obs.size(), 1);
  J_.resize(obs.size(), x_.size());
  for (int i = 0; i < max_iter_; i++) {
    // Update the function and Jacobian
    for (int j = 0; j < int(obs.size()); j++) {
      fx_(j, 0) = u_fx_(x_, obs.at(j))(0, 0);
      for (int k = 0; k < int(x_.size()); k++) {
        J_(j, k) = u_J_(x_, obs.at(j))(0, k);
      }
    }

    // Calculate H and B
    H_ = J_.transpose() * J_;
    B_ = -J_.transpose() * fx_;

    delta_x_ = H_.ldlt().solve(B_);
    x_ += delta_x_;

    if (!constraint_(x_)) {
      return SolverStatus::INVALID_START_VALUE;
    }
    if (delta_x_.norm() < min_step_) {
      return SolverStatus::SUCCESS;
    }
  }
  return SolverStatus::NO_CONVERGENCE;
}

Eigen::VectorXd GaussNewtonSolver::getState() { return x_; }

}  // namespace rm_buff