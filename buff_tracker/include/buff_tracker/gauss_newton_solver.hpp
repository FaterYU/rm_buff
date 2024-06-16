// Copyright (C) 2024 Zheng Yu
// Licensed under the MIT License.

#ifndef BUFF_TRACKER__GAUSS_NEWTON_SOLVER_HPP_
#define BUFF_TRACKER__GAUSS_NEWTON_SOLVER_HPP_

#include <chrono>
#include <eigen3/Eigen/Cholesky>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/QR>
#include <eigen3/Eigen/SVD>
#include <functional>
#include <vector>

namespace rm_buff
{
class GaussNewtonSolver
{
public:
  GaussNewtonSolver() = default;
  using VecMatFunc =
    std::function<Eigen::MatrixXd(const Eigen::VectorXd &, const std::vector<double> &)>;
  using BoolMatFunc = std::function<bool(const Eigen::VectorXd &)>;

  enum SolverStatus { SUCCESS, NO_CONVERGENCE, INVALID_START_VALUE };

  explicit GaussNewtonSolver(
    const VecMatFunc & u_fx, const VecMatFunc & u_J, const BoolMatFunc & constraint, int max_iter,
    double min_step, int obs_max_size);

  void addObservation(double x, double y);

  void setStartValue(const Eigen::VectorXd & x);

  SolverStatus solve();

  Eigen::VectorXd getState();

  std::vector<std::vector<double>> obs;

private:
  Eigen::VectorXd x_;

  VecMatFunc u_fx_;
  Eigen::MatrixXd fx_;

  VecMatFunc u_J_;
  Eigen::MatrixXd J_;

  Eigen::MatrixXd H_;
  Eigen::VectorXd B_;
  Eigen::VectorXd delta_x_;

  BoolMatFunc constraint_;

  int max_iter_;
  double min_step_;
  int obs_max_size_;

};  // class GaussNewtonSolver

}  // namespace rm_buff

#endif  // BUFF_TRACKER__GAUSS_NEWTON_SOLVER_HPP_