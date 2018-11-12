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
 */
//----------------------------------------------------------------------

#ifndef RAMPUSCONTROLLER_H
#define RAMPUSCONTROLLER_H

#include <ctime>
#include <math.h>
#include <vector>
#include <numeric>

namespace oadrive {
namespace control {

/*!
 * \brief The RAMPUSCONTROLLER determines and controlls the optimal speed
*/
class RampUSController
{
public:
    RampUSController();

    ~RampUSController(){};

    /*!
     * \brief PI controller to controll desired speed
     * \param desired Angle
     * \param measured Angle
     * \return controlled Angle
    */
    float update();

    void resetAngleDifferenceSum()
    {
      mDistanceSum_left = 0;
      mDistanceSum_right = 0;
    }
  

    //! get averaged speed
    void calculateAverageDistance()  
    {
      averageDistance_left = std::accumulate(measuredDistanceList_left.begin(),measuredDistanceList_left.end(),0.0f) / measuredDistanceList_left.size();
      averageDistance_right = std::accumulate(measuredDistanceList_right.begin(),measuredDistanceList_right.end(),0.0f) / measuredDistanceList_right.size();
      //return 0;
    }
    //! To reduce the noise -> moving average LP filter
    void addUSSample(const float measuredDistance_left, const float measuredDistance_right)
    {
      if (2 < measuredDistance_left && measuredDistance_left < 400)
      {
        if(measuredDistanceList_left.size() == mNumberOfAveragedValues)
          measuredDistanceList_left.erase(measuredDistanceList_left.begin());
        measuredDistanceList_left.push_back(measuredDistance_left);
      }
        
      if (2 < measuredDistance_right && measuredDistance_right < 400)
      {
        if(measuredDistanceList_right.size() == mNumberOfAveragedValues)
          measuredDistanceList_right.erase(measuredDistanceList_right.begin());
        measuredDistanceList_right.push_back(measuredDistance_right);
      }
    }

private:
    // P of the PI speed controller
    const float mKp;
    // I of the PI speed controller
    const float mKi;
    // maximal additional PI_controller_speed (output_speed = desires_speed + controller_speed)
    const float mMaxPI_Angle;
    // cap I-part of the PI controller
    const float mMaxDistanceDifferenceSum;
    // sample time is needed to calculate the integral
    clock_t mLastUpdateTime;
    // I-sum for the speed controller
    float mDistanceDifferenceSum;

    float mDistanceSum_left;
    float mDistanceSum_right;


    // over how many distance measurements should we average?
    const float mNumberOfAveragedValues;
    std::vector<float> measuredDistanceList_left;
    std::vector<float> measuredDistanceList_right;

    float averageDistance_right;
    float averageDistance_left;
    float mCalculatedDistance;
};
}
}
#endif // RAMPUSCONTROLLER_H
