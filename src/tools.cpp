#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

/**
 * Calculate the RMSE here.
 * @param estimations
 * @param ground_truth
 * @return
 */
VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  // This code is basically right from the Udacity lectures.
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0; // init

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  unsigned long estimation_size = estimations.size();
  unsigned long ground_truth_size = ground_truth.size();

  if (estimation_size != ground_truth_size || estimation_size == 0) {
    std::cout << "Invalid estimation or ground_truth data" << std::endl;
    return rmse;
  }

  //accumulate squared residuals
  for (unsigned int i = 0; i < estimation_size; ++i) {
    VectorXd residual = estimations[i] - ground_truth[i];

    //coefficient-wise multiplication
    residual = residual.array() * residual.array();
    rmse += residual;
  }

  //calculate the mean
  rmse = rmse / estimation_size;

  //calculate the squared root
  rmse = rmse.array().sqrt();

  //return the result
  return rmse;
}

/**
 * Calculate a Jacobian here.
 * @param x
 * @return
 */
MatrixXd Tools::CalculateJacobian(const VectorXd &x) {
  double px, py, vx, vy;

  MatrixXd Hj(3,4);

  //recover state parameters
  px = x[0];
  py = x[1];
  vx = x[2];
  vy = x[3];

  double px_2 = pow(px, 2);
  double py_2 = pow(py, 2);
  double norm = sqrt(px_2 + py_2);

  //check division by zero
  if (fabs(norm) < 0.0001) {
    std::cout << "Error. Divide by zero.";
    throw 1;
  } else {
    Hj <<
      px / norm,                                          py / norm,                                            0,          0,
      -1 * py / (px_2 + py_2),                            px / (px_2 + py_2),                                   0,          0,
      py * (vx * py - vy * px) / pow(px_2 + py_2, 3/2.),  px * (vy * px - vx * py) / pow(px_2 + py_2, 3/2.),    px / norm,  py / norm;
  }

  return Hj;
}


float Tools::ThetaValueCorrection(float theta) {
  while (true) {
    if (theta <= M_PI && theta >= -M_PI) {
      break;
    }

    if (theta > M_PI) {
      theta -= (2 * M_PI);
    }

    if (theta < -M_PI) {
      theta += (2 * M_PI);
    }
  }
  return theta;
}