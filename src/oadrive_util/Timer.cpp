// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// This program is free software licensed under the CDDL
// (COMMON DEVELOPMENT AND DISTRIBUTION LICENSE Version 1.0).
// You can find a copy of this license in LICENSE in the top
// directory of the source code.
//
// © Copyright 2016 FZI Forschungszentrum Informatik, Karlsruhe, Germany
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Peter Zimmer <peter-zimmer@gmx.net>
 * \author  Micha Pfeiffer <ueczz@student.kit.edu>
 * \date    2016-01-22
 *
 */
//----------------------------------------------------------------------

#include "Timer.h"

#include "utilLogging.h"
using icl_core::logging::endl;
using icl_core::logging::flush;

namespace oadrive {
namespace util {
Timer::Timer()
  : mTime(0)
  , mMaxId(1)
{
}

void Timer::setTimeIncrement(int milliSeconds)
{
  mTime += milliSeconds;
  checkTimers();
}

int Timer::getTime()
{
  return mTime;
}

unsigned long Timer::setTimer(int timeMilliseconds, timerType type )
{
  structTimers newTimer;
  newTimer.endTime = mTime+timeMilliseconds;
  newTimer.type = type;
  mMaxId++;
  newTimer.uniqueTimerID = mMaxId;
  newTimer.markedForDeletion = false;
  mTimers.push_back(newTimer);

//  LOGGING_INFO( utilLogger, "New timer registered (type " << type << ")" << endl );

  return newTimer.uniqueTimerID;
}

void Timer::removeTimer( unsigned long timerID )
{
  std::list<structTimers>::iterator timerIt = mTimers.begin();
  for( timerIt = mTimers.begin(); timerIt != mTimers.end(); timerIt++ )
  {
    if( timerIt->uniqueTimerID == timerID )
    {
      // There's a timer with this ID, so mark it for deletion:
      timerIt->markedForDeletion = true;
      return;
    }
  }
}

void Timer::checkTimers()
{
  std::list<structTimers>::iterator timerIt = mTimers.begin();
  while(timerIt != mTimers.end())
  {
    // First, check if the timer is to be deleted:
    if( timerIt->markedForDeletion )
    {
      LOGGING_INFO( utilLogger, "Deleting timer of type " << timerIt->type << endl );
      // Remove the timer:
      timerIt = mTimers.erase( timerIt );
    } else if( timerIt->endTime <= mTime ) {
      // If the timer should not be deleted and its time has passed, fire the event.
//      LOGGING_INFO( utilLogger, "Firing timer of type " << timerIt->type << endl );

      std::list<TimerEventListener*>::iterator listenerIt;
      for( listenerIt = mListeners.begin(); listenerIt != mListeners.end(); listenerIt++ )
      {
        // Notify the listeners:
        (*listenerIt)->eventTimerFired( timerIt->type, timerIt->uniqueTimerID );
      }
      // Remove the timer:
      timerIt = mTimers.erase(timerIt);
    } else {
      timerIt++;
    }
  }
}

void Timer::addListener( TimerEventListener* listener )
{
//  LOGGING_INFO( utilLogger, "New timer event listener registered." << endl );
  mListeners.push_back( listener );
}

void Timer::removeListener( TimerEventListener* listener )
{
  mListeners.remove( listener );
}

void Timer::clearAllTimers()
{
  mTimers.clear();
}

}
}
