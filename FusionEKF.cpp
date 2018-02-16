#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
  //initialize measurement matrix for laser
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;     
  //set the acceleration noise components
  noise_ax = 5;
  noise_ay = 5;

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    VectorXd x_ = VectorXd(4);
    //state covariance matrix P
    MatrixXd P_ = MatrixXd(4, 4);
    P_ << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1000, 0,
          0, 0, 0, 1000;

    //the initial transition matrix F_
    MatrixXd F_ = MatrixXd(4, 4);
    F_ << 1, 0, 1, 0,
          0, 1, 0, 1,
          0, 0, 1, 0,
          0, 0, 0, 1;

    //the process covariance matrix Q
    MatrixXd Q_ = MatrixXd(4, 4);
    Q_ << 0, 0, 0, 0,
          0, 0, 0, 0,
          0, 0, 0, 0,
          0, 0, 0, 0;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      float rho = measurement_pack.raw_measurements_[0];
      float phi = measurement_pack.raw_measurements_[1];
      float rhodot = measurement_pack.raw_measurements_[2];
      //convert to cartesian coordinates

      x_ << rho*sin(phi), rho*cos(phi), rhodot*sin(phi), rhodot*cos(phi);
      Hj_ = CalculateJacobian(x_);
      ekf_.Init(x_,P_,F_,Hj_,R_radar_,Q_);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      //set the state with the initial location and zero velocity
      x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
      ekf_.Init(x_,P_,F_,H_laser_,R_radar_,Q_);
    }

    previous_timestamp_ = measurement_pack.timestamp_;
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  //compute the time elapsed between the current and previous measurements
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0; //dt - expressed in seconds
  previous_timestamp_ = measurement_pack.timestamp_;
  
  //time is integrated
  
  ekf_.F_(0,2) = dt;
  ekf_.F_(1,3) = dt;
  
  //2. Set the process covariance matrix Q
  ekf_.Q_ = MatrixXd(4,4);
  ekf_.Q_ << 1 , 0 , 1 , 0,
           0 , pow(dt,4)/4*noise_ay , 0 , pow(dt,3)/2*noise_ay,
           pow(dt,3)/2*noise_ax , 0 , pow(dt,2)*noise_ax , 0,
           0 , pow(dt,3)/2*noise_ay , 0 , pow(dt,2)*noise_ay;
  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    //set Jacobian measurement matrix and measurement covariance matrix for radar
    ekf_.Hj_ = CalculateJacobian(ekf_.x_);
    ekf_.R_ = R_radar_;
  } else {
    //set measurement matrix and measurement covariance matrix for lidar
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
  }

  //measurement update
  ekf_.Update(measurement_pack.raw_measurements_);

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
