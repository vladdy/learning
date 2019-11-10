#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.3 * M_PI;
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
  
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  n_x_   = 5;
  n_aug_ = 7;

  lambda_ = 3 - n_aug_;

  weights_ = VectorXd(2 * n_aug_ + 1).setOnes();
  weights_ = 1/(2 * (lambda_ + n_aug_)) * weights_;
  weights_(0) = lambda_/(lambda_ + n_aug_);

  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  P_ = MatrixXd::Identity(5,5);

  x_ = VectorXd(5);
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */

  /*****************************************************************************
  *  Initialization
  ****************************************************************************/
  if (!is_initialized_) {
    if (use_radar_ && (meas_package.sensor_type_ == MeasurementPackage::RADAR)) {
      float rho   = meas_package.raw_measurements_(0);
      float theta = meas_package.raw_measurements_(1);

      while (theta >  M_PI) { theta -= 2. * M_PI; };
      while (theta < -M_PI) { theta += 2. * M_PI; };

      x_ << rho * cos(theta), rho * sin(theta), 4, 0, 0;
      P_(0,0) = 0.3 * cos(0.03);
      P_(1,1) = 0.3 * sin(0.03);
    }
    else if (use_laser_ && (meas_package.sensor_type_ == MeasurementPackage::LASER)) {
      x_ <<  meas_package.raw_measurements_(0), meas_package.raw_measurements_(1), 4, 0, 0;
      P_(0,0) = 0.15;
      P_(1,1) = 0.15;
    }

    time_us_ = meas_package.timestamp_;
    is_initialized_ = true;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
  float dt = (meas_package.timestamp_ - time_us_) / 1000000.0;

  time_us_ = meas_package.timestamp_;

  Prediction(dt);

  /*****************************************************************************
   *  Update
   ****************************************************************************/
  if (use_radar_ && (meas_package.sensor_type_ == MeasurementPackage::RADAR)) {
    UpdateRadar(meas_package);
  } else if (use_laser_ && (meas_package.sensor_type_ == MeasurementPackage::LASER)) {
    UpdateLidar(meas_package);
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */


  // Create Sigma points
  VectorXd x_aug = VectorXd(n_aug_);
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  x_aug.setZero();
  x_aug.head(n_x_) = x_;

  P_aug.setZero();
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(n_x_, n_x_) = std_a_ * std_a_;
  P_aug(n_x_ + 1, n_x_ + 1) = std_yawdd_ * std_yawdd_;

  MatrixXd A_aug = P_aug.llt().matrixL();

  Xsig_aug.col(0)  = x_aug;
  for (int i = 0; i < n_aug_; i++)
  {
    Xsig_aug.col(i + 1) = x_aug + sqrt(lambda_ + n_aug_) * A_aug.col(i);
    Xsig_aug.col(i + 1+ n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * A_aug.col(i);
  }

  // Sigma point prediction
  for (int i = 0; i< 2 * n_aug_ + 1; i++)
  {
      double p_x = Xsig_aug(0,i);
      double p_y = Xsig_aug(1,i);
      double v = Xsig_aug(2,i);
      double yaw = Xsig_aug(3,i);
      double yawd = Xsig_aug(4,i);
      double nu_a = Xsig_aug(5,i);
      double nu_yawdd = Xsig_aug(6,i);

      double px_p, py_p;

      if (fabs(yawd) > 0.001) {
        px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
        py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
      }
      else {
        px_p = p_x + v*delta_t*cos(yaw);
        py_p = p_y + v*delta_t*sin(yaw);
      }

      double v_p = v;
      double yaw_p = yaw + yawd*delta_t;
      double yawd_p = yawd;

      px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
      py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
      v_p = v_p + nu_a*delta_t;

      yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
      yawd_p = yawd_p + nu_yawdd*delta_t;

      Xsig_pred_(0,i) = px_p;
      Xsig_pred_(1,i) = py_p;
      Xsig_pred_(2,i) = v_p;
      Xsig_pred_(3,i) = yaw_p;
      Xsig_pred_(4,i) = yawd_p;
  }

  // Predict mean and covariance
  x_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    x_ = x_ + weights_(i) * Xsig_pred_.col(i);
  }

  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose() ;
  }
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */

  /*
   * LIDAR here is linear. This means that we do not need to use the unscented transformation here.
   * Using the linear Kalman filter update equations just like in the EKF project should give the same results and
   * has the advantage to require less computational resources.
   */
  int n_z_ = 2;

  MatrixXd H(n_z_, n_x_);
  H << 1, 0, 0, 0, 0,
       0, 1, 0, 0, 0;

  MatrixXd R(n_z_, n_z_);
  R << std_laspx_ * std_laspx_, 0,
       0, std_laspy_ * std_laspy_;

  VectorXd z = meas_package.raw_measurements_;

  VectorXd y  = z - H * x_;
  MatrixXd Ht = H.transpose();
  MatrixXd S = H * P_ * Ht + R;
  MatrixXd K  = P_ * Ht * S.inverse();

  x_ = x_ + K * y;
  P_ = (MatrixXd::Identity(n_x_, n_x_) - K * H) * P_;

  double epsilon = y.transpose() * S.inverse() * y;

  cout << "UpdateLidar(): time=" << time_us_ << " epsilon=" << epsilon << endl;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */

  int n_z = 3;

  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
  VectorXd z_pred = VectorXd(n_z);
  MatrixXd S = MatrixXd(n_z,n_z);
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  VectorXd z = VectorXd(n_z);
  z = meas_package.raw_measurements_;

  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v  = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;


    Zsig(0,i) = sqrt(p_x * p_x + p_y * p_y);
    Zsig(1,i) = atan2(p_y,p_x);
    Zsig(2,i) = (p_x * v1 + p_y * v2 ) / sqrt(p_x * p_x + p_y * p_y);
  }

  z_pred.fill(0.0);
  for (int i=0; i < 2 * n_aug_ +1; i++) {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    VectorXd zdiff = Zsig.col(i) - z_pred;
    while (zdiff(1) > M_PI) {zdiff(1) -= 2.*M_PI;}
    while (zdiff(1) < -M_PI) {zdiff(1) += 2.*M_PI;}

    S += weights_(i) * zdiff * zdiff.transpose();
  }

  S(0,0) += std_radr_ * std_radr_;
  S(1,1) += std_radphi_ * std_radphi_;
  S(2,2) += std_radrd_ * std_radrd_;

  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    VectorXd z_diff = Zsig.col(i) - z_pred;

    while (z_diff(1) >  M_PI) {z_diff(1) -= 2.*M_PI; }
    while (z_diff(1) < -M_PI) {z_diff(1) += 2.*M_PI; }


    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    while (x_diff(3) >  M_PI) {x_diff(3) -= 2.*M_PI; }
    while (x_diff(3) < -M_PI) {x_diff(3) += 2.*M_PI; }

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  MatrixXd K = Tc * S.inverse();

  VectorXd z_diff = z - z_pred;

  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();

  double epsilon = z_diff.transpose() * S.inverse() * z_diff;
  cout << "UpdateRadar(): time=" << time_us_ << " epsilon=" << epsilon << endl;
}
