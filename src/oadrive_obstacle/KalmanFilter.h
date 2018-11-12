// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// This program is free software licensed under the CDDL
// (COMMON DEVELOPMENT AND DISTRIBUTION LICENSE Version 1.0).
// You can find a copy of this license in LICENSE in the top
// directory of the source code.
//
// © Copyright 2018 FZI Forschungszentrum Informatik, Karlsruhe, Germany
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Robin Andlauer <robin.andlauer@student.kit.edu>
 * \date    2017-8-31
 *
 */
//----------------------------------------------------------------------
#include <iostream>
#include <memory>
#include <Eigen/Dense>
#include <ctime>
#include <ratio>
#include <chrono>

#ifndef OADRIVE_OBSTACLE_KALMANFILTER_H
#define OADRIVE_OBSTACLE_KALMANFILTER_H
/*!
   \brief simple Kalman filter
 */
using namespace Eigen;
using namespace std::chrono;

namespace oadrive{
namespace obstacle{
//!
//! \brief The SystemModel struct defines the system model used for the Kalman filter.
//! Abstract struct i.e. a new model derived from SystemModel has to be defined
//!
struct SystemModel
{
  virtual MatrixXf stateMatrix(const float dt) const = 0;
  virtual MatrixXf systemNoiseCovarianceMatrix(const float dt) const = 0;
  virtual MatrixXf outputMatrix() const = 0;
};

//!
//! \brief discrete KalmanFilter to track objects
//! General system model:
//! x_k = Phi*X_k + w_k
//! x = state; x_k_1 = followed state; Phi = system matrix; w_k = system noise
//! Q = system noise covariance matrix
//! P = Error covariance
//!
//! y_k = H*x_k + v_k
//! y_k = output/measurement; H = transition matrix; v_k = measurement noise
//! R = measurement noise covariance matrix
//!
class KalmanFilter
{
  public:
//    KalmanFilter();
    //! The Kalman filter can be defined by a custom system model derived from SystemModel or by assigning Phi, H and Q
    KalmanFilter(const MatrixXf &Phi, const MatrixXf &H, const MatrixXf &Q);
    KalmanFilter(const std::shared_ptr<SystemModel> systemModel);

    ~KalmanFilter();

    //! the Kalman Filter can be initialized either with or without a constant measurement Covariance.
    //! If no measurement covariance matrix was given before this->update was called, an assertion will be raised
    void init(const VectorXf &X, const MatrixXf &P);
    void init(const VectorXf &X, const MatrixXf &P, const MatrixXf &constR);

    //! 3 ways to update filter:
    //! 1. Without measurement -> update based on the system model
    //! 2. Without measurement covariance matrix -> has to be preliminary defined in this->init
    //! 3. With measurement covariance matrix
    VectorXf update();
    VectorXf update(const VectorXf& Z);
    VectorXf update(const VectorXf &Z, const MatrixXf &R);

    //! returns the Mahalanobis distance to the measurement but does not update the state
    float getMahalanobisDistance(const VectorXf &Z, const MatrixXf &R);
    //! returns P
    MatrixXf getErrorCovariance();
    //! returns X
    VectorXf getState();

    //! set X
    void setState(const VectorXf &X);
    //! set X and P
    void setState(const VectorXf &X, const MatrixXf &P);

  private:
    //! calculates sample time
    float getTimePassedSinceLastUpdate(high_resolution_clock::time_point currentTime);

    //! system matrix Phi
    MatrixXf mPhi;
    //! transition matrix H
    MatrixXf mH;
    //! System noise covariance matrix Q = (E{w w'})
    MatrixXf mQ;
    //! measurement noise covariance matrix  R = ((E{v v'}))
    MatrixXf mR;
    //! error covariance matrix P
    MatrixXf mP;
    //! state X
    VectorXf mX;

    //! in case a custom system model is given
    std::shared_ptr<SystemModel> mSystemModel;

    //! last update time
    high_resolution_clock::time_point mLastUpdateTime;
    //! initialisation flags
    bool mStateInitialized;
    bool mMeasurementCovarianceMatrixInitialized;
    bool mCustomSystemModel;
    bool mClockInitialized;
};

} // namespace
}
#endif

