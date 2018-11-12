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
 * \date    2017-7-15
 * 
 * \author  Simon Roesler <simon.roesler@student.kit.edu>
 * \date    2018
 */
//----------------------------------------------------------------------

#ifndef SPEEDCONTROLLER_H
#define SPEEDCONTROLLER_H

#include <ctime>
#include <vector>

namespace oadrive {
namespace control {

/*!
 * \brief The SpeedController determines and controlls the optimal speed
*/
class SpeedController
{
public:
    SpeedController();
    virtual ~SpeedController();

    /*!
     * \brief PI controller to controll desired speed
     * \param desired speed
     * \param measured speed
     * \return controlled speed
    */
    float update(const float targetSpeed, const float steeringAngle, const bool justRoll);
    /*!
     * \brief The I-part of the PI controller should be resetted every time we stop or change direction.
    */
    void resetSpeedDifferenceSum();
    //! get averaged speed
    float calculateAverageSpeed();
    //! speed measurements were very noise. Hence the speed is averaged over the last X measurements
    void addSpeedSample(const float measuredSpeed);

private:
    // P of the PI speed controller
    const float mKp;
    // I of the PI speed controller
    const float mKi;
    // maximal additional PI_controller_speed (output_speed = desires_speed + controller_speed)
    const float mMaxPI_Speed;
    // cap I-part of the PI controller
    const float mMaxSpeedDifferenceSum;
    // sample time is needed to calculate the integral
    clock_t mLastUpdateTime;
    // I-sum for the speed controller
    float mSpeedDifferenceSum;

    int mTargetDirection;

    // over how many speed measurements should we average?
    const float mNumberOfAveragedValues;
    std::vector<float> measuredSpeedList;
};

}	//namespace
}	//namespace
#endif // SPEEDCONTROLLER_H
