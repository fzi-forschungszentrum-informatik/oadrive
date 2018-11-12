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
 * \author  Simon Roesler <simon.roesler@student.kit.edu>
 * \date    2018
 *
 */
//----------------------------------------------------------------------

#include "KalmanFilter.h"

namespace oadrive{
namespace obstacle{

//KalmanFilter::KalmanFilter()
//{

//}

KalmanFilter::KalmanFilter(const MatrixXf& Phi, const MatrixXf& H, const MatrixXf& Q)
  : mPhi(Phi)
  , mH(H)
  , mQ(Q)
  , mStateInitialized(false)
  , mMeasurementCovarianceMatrixInitialized(false)
  , mCustomSystemModel(false)
  , mClockInitialized(false)
{

}

KalmanFilter::KalmanFilter(const std::shared_ptr<SystemModel> systemModel)
  : mSystemModel(systemModel)
  , mH(systemModel->outputMatrix())
  , mStateInitialized(false)
  , mMeasurementCovarianceMatrixInitialized(false)
  , mCustomSystemModel(true)
  , mClockInitialized(false)
{

}

KalmanFilter::~KalmanFilter()
{

}

void KalmanFilter::init(const VectorXf& X, const MatrixXf& P)
{
  mP = P;
  mX = X;
  mStateInitialized = true;
}

void KalmanFilter::init(const VectorXf& X, const MatrixXf& P, const MatrixXf& constR)
{
  mR = constR;
  mMeasurementCovarianceMatrixInitialized = true;
  this->init(X,P);
}

VectorXf KalmanFilter::update()
{
    assert(mStateInitialized && "Kalman filter not initialized!");

    // calculate Phi and Q if they depend on dt (custom system model)
    if(mCustomSystemModel)
    {
      high_resolution_clock::time_point currentTime = high_resolution_clock::now();
      float dt = this->getTimePassedSinceLastUpdate(currentTime);
      mLastUpdateTime = currentTime;

      mPhi = mSystemModel->stateMatrix(dt);
      mQ = mSystemModel->systemNoiseCovarianceMatrix(dt);
    }

    // Prediction
    mX = mPhi * mX;
    mP = mPhi * mP * mPhi.transpose() + mQ;
  return mX;
}

VectorXf KalmanFilter::update(const VectorXf &Z)
{
  assert(mStateInitialized && "Kalman filter not initialized!");
  assert(mMeasurementCovarianceMatrixInitialized && "No measurement covariance matrix R found!");

  // calculate Phi and Q if they depend on dt (custom system model)
  if(mCustomSystemModel)
  {
    high_resolution_clock::time_point currentTime = high_resolution_clock::now();
    float dt = this->getTimePassedSinceLastUpdate(currentTime);
    mLastUpdateTime = currentTime;

    mPhi = mSystemModel->stateMatrix(dt);
    mQ = mSystemModel->systemNoiseCovarianceMatrix(dt);
  }

  // Prediction
  VectorXf X_apriori = mPhi * mX;
  MatrixXf P_apriori = mPhi * mP * mPhi.transpose() + mQ;

  // Estimate measurement
  VectorXf Z_estimate = mH * X_apriori;
  // Calculate covariance of residuum
  MatrixXf S = mH * P_apriori * mH.transpose() + mR;
  // Calculate gain
  MatrixXf K = P_apriori * mH.transpose() * S.inverse();
  // Calculate residuum
  VectorXf Residuum = Z - Z_estimate;
  // Innovation
  mX = X_apriori + (K * Residuum);
  mP = P_apriori - (K * mH * P_apriori);
  return mX;
}

VectorXf KalmanFilter::update(const VectorXf& Z, const MatrixXf& R)
{
  mR = R;
  mMeasurementCovarianceMatrixInitialized = true;
  this->update(Z);
  return mX;
}

float KalmanFilter::getMahalanobisDistance(const VectorXf& Z, const MatrixXf& R)
{
  // calculate Phi and Q if they depend on dt (custom system model)
  MatrixXf Phi, Q;
  if(mCustomSystemModel)
  {
    float dt = this->getTimePassedSinceLastUpdate(high_resolution_clock::now());
    Phi = mSystemModel->stateMatrix(dt);
    Q = mSystemModel->systemNoiseCovarianceMatrix(dt);
  }
  else
  {
    Phi = mPhi;
    Q = mQ;
  }

  // Prediction
  VectorXf X_apriori = Phi * mX;
  MatrixXf P_apriori = Phi * mP * Phi.transpose() + Q;

  // Calculate Mahalanobis
  VectorXf residuum = Z - (mH * X_apriori);
  MatrixXf S = mH * P_apriori * mH.transpose() + R;
  float mahalanobisDistance = residuum.transpose()*S.inverse()*residuum;
  // std::cout << "MahalanobisDistance: " << mahalanobisDistance << std::endl;
  return mahalanobisDistance;
}

VectorXf KalmanFilter::getState()
{
  return mX;
}

MatrixXf KalmanFilter::getErrorCovariance()
{
  return mP;
}

void KalmanFilter::setState(const VectorXf &X)
{
  mX = X;
}

void KalmanFilter::setState(const VectorXf &X, const MatrixXf &P)
{
  mX = X;
  mP = P;
}

float KalmanFilter::getTimePassedSinceLastUpdate(high_resolution_clock::time_point currentTime)
{
   duration<float> dt;

  if(mClockInitialized)
  {
    dt = duration_cast<duration<float>>(currentTime-mLastUpdateTime);
    return dt.count();
  }
  else
  {
    mClockInitialized = true;
    return 0.0f;
  }
}

} // namespace
}
