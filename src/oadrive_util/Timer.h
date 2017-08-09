// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// This program is free software licensed under the CDDL
// (COMMON DEVELOPMENT AND DISTRIBUTION LICENSE Version 1.0).
// You can find a copy of this license in LICENSE in the top
// directory of the source code.
//
// © Copyright 2017 FZI Forschungszentrum Informatik, Karlsruhe, Germany
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Peter Zimmer <peter-zimmer@gmx.net>
 * \author Micha Pfeiffer <ueczz@student.kit.edu>
 * \date    2016-01-22
 *
 */
//----------------------------------------------------------------------

#ifndef OADRIVE_UTIL_TIMER_H
#define OADRIVE_UTIL_TIMER_H
#include <list>
#include <boost/function.hpp>
#include <boost/bind.hpp>

#include "TimerTypes.h"
#include "TimerEventListener.h"
#include <boost/shared_ptr.hpp>

namespace oadrive {
namespace util {

struct structTimers{
  int endTime;
  unsigned long uniqueTimerID;
  timerType type;
  bool markedForDeletion;
};
class TimerEventListener;
typedef boost::shared_ptr<TimerEventListener> TimerEventListenerPtr;
typedef boost::shared_ptr<const TimerEventListener> ConstTimerEventListenerPtrPtr;
typedef std::list<TimerEventListenerPtr> TimerEventListenerPtrList;

class Timer
{
public:
  typedef boost::shared_ptr<Timer> Ptr;
  typedef boost::shared_ptr<const Timer> ConstPtr;


  Timer();
  //!
  //! \brief setTimeIncrement Increments the internal counter
  //! \param milliSeconds milliSeconds which elapsed since last call
  //!
  void setTimeIncrement(int milliSeconds);

  //!\brief returns the time in milli seconds since system start
  int getTime();
  //!
  //! \brief setTimer add a timer. funcPtr will be call after timeMilliseconds. It is not garanteed, that funcPtr is called exact after N milliseconds.
  //! \note delay depens on setTimeIncrement. It is only guranted, that it is called afte time Millisenconds.
  //! \param funcPtr Ptr to function (can be produced with boost::bind( &testclass::testFunc2, this ))
  //! \param timeMilliseconds time in Milliseconds after that funcPtr will be called
  //! \return uniqueTimerID, can be used to identify the timer later.
  //!
  unsigned long setTimer(int timeMilliseconds, timerType type );

  /*! Sets the timer given by timerID to be deleted.*/
  void removeTimer( unsigned long timerID );

  void addListener( TimerEventListenerPtr listener );
  void removeListener( TimerEventListenerPtr listener );

  void clearAllTimers();

private:
  //! \brief mTime holds the time in milliSeconds. Overflow in 8 Years. Happy debugging.
  int mTime;
  //!brief holds the maximum ID
  unsigned long mMaxId;
  //!\brief holds all actual timers
  std::list <structTimers> mTimers;
  //!\brief checks all timers if a call is needed
  void checkTimers();

  TimerEventListenerPtrList mListeners;
};
}
}
#endif // TIMER_H
