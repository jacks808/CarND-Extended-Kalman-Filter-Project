#include "kalman_filter.h"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

/**
 * predict the state
 */
void KalmanFilter::Predict() {
  // using state transition matrix F_ to predict new state state x
  x_ = F_ * x_;

  // using state covariance matrix P_ and process noise matrix Q_ to
  // predict new state covariance matrix P
  P_ = F_ * P_ * F_.transpose() + Q_;
}

/**
 * update state vector x using Kalman filter
 * @param z raw measurement data
 */
void KalmanFilter::Update(const VectorXd &z) {
  /**
   * update the state by using Kalman Filter equations
   */
  VectorXd y(2); // px, py
  MatrixXd S(2, 2);
  MatrixXd K(4, 2);

  y = z - H_ * x_;
  S = H_ * P_ * H_.transpose() + R_;
  K = P_ * H_.transpose() * S.inverse();

  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
  x_ = x_ + (K * y); // update state vector x
  P_ = (I - K * H_) * P_; // update state covariance matrix P
}

/**
 * update state vector x using Extended Kalman Filter equations
 * @param z raw measurement data
 */
void KalmanFilter::UpdateEKF(const VectorXd &z) {
  VectorXd y(3);
  MatrixXd Hj(3, 4);
  MatrixXd S(3, 3);
  MatrixXd K(4, 3);


  // use function h to convert
  y = z - h(x_);

  y[1] = Tools::ThetaValueCorrection(y[1]);

  Hj = Tools::CalculateJacobian(x_);

  S = Hj * P_ * Hj.transpose() + R_;
  K = P_ * Hj.transpose() * S.inverse();

  x_ = x_ + (K * y); // update state vector x

  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
  P_ = (I - K * Hj) * P_; // update covariance matrix P
}

/**
 * convert x which is a 4*1 vector to 4*1 vector contains range, bearing, range_rate
 * @param x
 * @return
 */
VectorXd KalmanFilter::h(const VectorXd &x) {
  float px = x[0];
  float py = x[1];
  float vx = x[2];
  float vy = x[3];

  float range = sqrt(pow(px, 2) + pow(py, 2));
  float bearing = atan2(py, px);
  float range_rate = (px * vx + py * vy) / range;

  VectorXd result(3);
  result << range, bearing, range_rate;
  return result;
}
