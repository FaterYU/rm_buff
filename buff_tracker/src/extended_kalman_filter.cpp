#include "buff_tracker/extended_kalman_filter.hpp"

namespace rm_buff
{
ExtendedKalmanFilter::ExtendedKalmanFilter(
  const VecVecFunc & f, const VecVecFunc & h, const VecMatFunc & j_f, const VecMatFunc & j_h,
  const VoidMatFunc & u_q, const VecMatFunc & u_r, const Eigen::MatrixXd & P0)
: f(f),
  h(h),
  jacobian_f(j_f),
  jacobian_h(j_h),
  update_Q(u_q),
  update_R(u_r),
  P_post(P0),
  n(P0.rows()),
  I(Eigen::MatrixXd::Identity(n, n)),
  x_pri(n),
  x_post(n)
{
}

void ExtendedKalmanFilter::setState(const Eigen::VectorXd & x0) { x_post = x0; }

void ExtendedKalmanFilter::setInitState(const Eigen::VectorXd & x0)
{
  x_post = x0;
  Eigen::DiagonalMatrix<double, 9> p0;
  p0.setIdentity();
  P_post = p0;
  P_pri = p0;
  x_pri = x0;
}

Eigen::MatrixXd ExtendedKalmanFilter::predict()
{
  F = jacobian_f(x_post), Q = update_Q();

  x_pri = f(x_post);
  P_pri = F * P_post * F.transpose() + Q;

  // handle the case when there will be no measurement before the next predict
  x_post = x_pri;
  P_post = P_pri;

  return x_pri;
}

Eigen::MatrixXd ExtendedKalmanFilter::update(const Eigen::VectorXd & z)
{
  H = jacobian_h(x_pri), R = update_R(z);

  K = P_pri * H.transpose() * (H * P_pri * H.transpose() + R).inverse();
  printf("z: %f, %f, %f, %f, %f, %f\n", z(0), z(1), z(2), z(3), z(4), z(5));
  printf(
    "x_pri: %f, %f, %f, %f, %f, %f, %f, %f, %f\n", x_pri(0), x_pri(1), x_pri(2), x_pri(3), x_pri(4),
    x_pri(5), x_pri(6), x_pri(7), x_pri(8));
  printf(
    "h(x_pri): %f, %f, %f, %f, %f, %f\n", h(x_pri)(0), h(x_pri)(1), h(x_pri)(2), h(x_pri)(3),
    h(x_pri)(4), h(x_pri)(5));
  x_post = x_pri + K * (z - h(x_pri));
  P_post = (I - K * H) * P_pri;

  return x_post;
}

}  // namespace rm_buff