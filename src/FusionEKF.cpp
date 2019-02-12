#include <iostream>
#include "config.h"
#include "FusionEKF.h"
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  H_laser_ = MatrixXd(2, X_SIZE);

  R_radar_ = MatrixXd(3, 3); // 3 * 3 is because radar measurement range, bearing and range_rate
  Hj_ = MatrixXd(3, X_SIZE); // jacobian matrix

  //measurement covariance matrix - laser
  R_laser_ <<
      0.0225, 0,
      0,      0.0225;

  //measurement matrix - laser
  H_laser_ <<
      1, 0, 0, 0, // px
      0, 1, 0, 0; // py

  //measurement covariance matrix - radar
  R_radar_ <<
      0.09, 0,      0,
      0,    0.0009, 0,
      0,    0,      0.09;

  /**
   * Initializing the FusionEKF.
   * initialize variables and matrices (x, F, H_laser, H_jacobian, P, etc.)
   * Set the process and measurement noises
   */

  double init_dt = 0;

  // the initial state transition matrix F_
  ekf_.F_ = MatrixXd(X_SIZE, X_SIZE);
  ekf_.F_ <<
      1, 0, init_dt, 0,       // px + motion_x = px + dt * vx
      0, 1, 0,       init_dt, // py + motion_y = py + dt * vy
      0, 0, 1,       0,       // vx
      0, 0, 0,       1;       // vy

  // state covariance matrix P_
  ekf_.P_ = MatrixXd(X_SIZE, X_SIZE);
  ekf_.P_ <<
      1,  0,  0,    0,    // px
      0,  1,  0,    0,    // py
      0,  0,  1000, 0,    // vx * 1000
      0,  0,  0,    1000; // vy * 1000
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

/**
 * Process measurement package.
 * @param measurement_pack
 */
void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * Initialize the state ekf_.x_ with the first measurement.
     * Create the covariance matrix.
     * Need to convert radar from polar to cartesian coordinates.
     */

    // first measurement, create a 4D state vector, we don't know yet the values of the x state
    if (DEVELOP_MODE) {
      cout << "EKF first measurement " << endl;
    }
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    double px, py, vx, vy;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
       * Convert radar from polar to cartesian coordinates and initialize state.
       */
      VectorXd m = measurement_pack.raw_measurements_;
      double range = m[0]; // The distance from the origin to our pedestrian.
      double bearing = m[1]; // The angle between the ray and x axis.
      double range_rate = m[2]; // The radial velocity along this ray

      px = range * cos(bearing);
      py = range * sin(bearing);
      vx = range_rate * cos(bearing);
      vy = range_rate * sin(bearing);

    } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
       * Initialize state.
       */
      VectorXd m = measurement_pack.raw_measurements_;
      px = m[0];
      py = m[1];

      // Lidar doesn't measurement velocity value, so vx and vy value is not need to set.
      vx = 0;
      vy = 0;
    }

    // update state vector
    ekf_.x_ << px, py, vx, vy;

    // update last measurement timestamp, if this value doesn't set, then the measurement value will error at first serial steps.
    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  /**
   * Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   */
  double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  if (dt < 0.0001) {
    cout << "Delta time value error, current value is: " << dt
         << ". Please reset your simulator and try again. " << endl;
    return;
  }

  if (DEVELOP_MODE) {
    cout << "dt: " << dt << "\n" << "s" << endl;
  }

  // set delta t for state transition matrix
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  /**
   * Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  int noise_ax = 9;
  int noise_ay = 9;

  double dt_2 = pow(dt, 2);
  double dt_3 = pow(dt, 3);
  double dt_4 = pow(dt, 4);

  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ <<
      dt_4 / 4 * noise_ax,  0,                    dt_3 / 2 * noise_ax,  0,
      0,                    dt_4 / 4 * noise_ay,  0,                    dt_3 / 2 * noise_ay,
      dt_3 / 2 * noise_ax,  0,                    dt_2 * noise_ax,      0,
      0,                    dt_3 / 2 * noise_ay,  0,                    dt_2 * noise_ay;

  // call predict function
  ekf_.Predict();

  /**
   * Update
   */

  /**
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  if (DEVELOP_MODE) {
    cout << "x_ = " << ekf_.x_ << endl;
    cout << "P_ = " << ekf_.P_ << endl;
  }
}
