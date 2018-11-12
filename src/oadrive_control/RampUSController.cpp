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
 * \author  Max Zipfl
 * \date    2018
 * 
 */
//----------------------------------------------------------------------
#include "RampUSController.h"
#include <iostream>

namespace oadrive {
namespace control {

/*!
 * \brief The RampUSController determines and controlls the optimal Angle
*/
RampUSController::RampUSController()
: mKp(0.07)
, mKi(0.01)
, mMaxPI_Angle(.35)
, mMaxDistanceDifferenceSum(mMaxPI_Angle) // sum*ki must not exceed mMaxPI_Angle/2 to prevent wind-up effects
, mDistanceSum_left(0)
, mDistanceSum_right(0)
, mNumberOfAveragedValues(3) {

}

float RampUSController::update()
{
  float weighting = 4.0;  //how much is the right sensor trusted in contrast to the left sensor..
  const float targetDistance = 13;
  float targetDistance_left = targetDistance + 43;
  // get sample time
  /*
  clock_t currentTime = clock();
  float dt = ((float)(currentTime - mLastUpdateTime)) / CLOCKS_PER_SEC;
  //security check
  if(fabs(dt) > 1.0f)
    dt = 0;
  mLastUpdateTime = currentTime;
  */
  calculateAverageDistance();
  //mCalculatedDistance = (averageDistance_left + averageDistance_right + 33) / 2 - 25 - 16;
  //mCalculatedDistance = averageDistance_right;
  if (averageDistance_right < 5)
  {
    weighting  = 10;
  }
  else
  {
    weighting = 4;
  }
  float DistanceDifference_right = targetDistance - averageDistance_right;
  float DistanceDifference_left = targetDistance_left - averageDistance_left;
  float DistanceDifference = (-DistanceDifference_left+(DistanceDifference_right*weighting))/(1+weighting);  //calculated with weighting

  std::cout <<"left " <<averageDistance_left << " right " <<averageDistance_right  <<"   Distance Difference: " << DistanceDifference <<std::endl; 



  // P controlling

  float PI_Angle = mKp * DistanceDifference;

  //security check:
  if (PI_Angle > mMaxPI_Angle)
    PI_Angle = mMaxPI_Angle;
  if (PI_Angle < -mMaxPI_Angle)
    PI_Angle = -mMaxPI_Angle;
 
  float controlledAngle = -PI_Angle;

  if (averageDistance_left > 70 || averageDistance_right > 30)
  {
    std::cout << "None or only one barrier is found  - drive straight" <<std::endl;
    return 0;
  }

  return controlledAngle;
}

};	//namespace
};	//namespace
