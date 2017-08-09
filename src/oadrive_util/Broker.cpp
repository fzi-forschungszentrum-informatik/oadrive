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
 * \author  Micha Pfeiffer <ueczz@student.kit.edu>
 * \author  Vitali Kaiser <vitali.kaiser@live.de>
 * \date    2016-03-03
 *
 */
//----------------------------------------------------------------------

#include "Broker.h"
#include "utilLogging.h"
#include <oadrive_util/Config.h>
#include <boost/thread/thread.hpp>
#include <algorithm>
using icl_core::logging::endl;
using icl_core::logging::flush;

namespace oadrive
{
namespace util
{

boost::shared_ptr<Broker> Broker::mInstance;

// Called when new direction for next intersection is received:
void Broker::eventReceivedCommand(
    boost::asio::io_service &ioService, const std::vector<char> &buf )
{
  std::string msg(buf.begin(), buf.end());

  LOGGING_INFO( utilLogger, "Received new command: " << msg << endl );

  if( msg == "stop" )   // End Redis connection
    ioService.stop();
  else if( msg == "left" || msg == "right" || msg == "straight" )
    setLastReceivedManeuver( msg );
  else if( msg == "turn" )
    setTurnCommandReceived();
  else if( msg == "speed_boost" || msg == "speed_normal" )
    setNewSpeedCommand( msg );
  else if( msg == "game_finished" )
    setHasReceivedStopCommand();
}

#ifdef _IC_BUILDER_REDISCLIENT_
void Broker::handleConnectedEventSub(
    boost::asio::io_service &ioService, RedisAsyncClient &redis,
    bool ok, const std::string &errmsg)
{
  if( ok )
  {
    mSubscriber.subscribe( mChannelNames[CHANNEL_RECEIVE_COMMAND],
        boost::bind( &Broker::eventReceivedCommand, this, boost::ref(ioService), _1) );
    LOGGING_INFO( utilLogger, "Subscribed to channel: " << endl );
    LOGGING_INFO( utilLogger, "\t" << mChannelNames[CHANNEL_RECEIVE_COMMAND] << endl );
    setConnected(true);
  }
  else
  {
    LOGGING_ERROR( utilLogger, "Broker could not connect to redis!" << endl );
  }
}


void Broker::handleConnectedEventPub(
    boost::asio::io_service &ioService, RedisAsyncClient &redis,
    bool ok, const std::string &errmsg)
{
  if( ok )
  {
    LOGGING_INFO( utilLogger, "Publisher connected." << endl );            

    publish( CHANNEL_SEND_STATUS, "connected" );
  }
  else
  {
    LOGGING_ERROR( utilLogger, "Broker could not connect to redis!" << endl );
  }
}
#endif



Broker::Broker ()
  : mConnected( false )
  , mPort( 6379 )
  , mIOService()
#ifdef _IC_BUILDER_REDISCLIENT_
  , mSubscriber( mIOService )
  , mPublisher( mIOService )
#endif
  , mLastReceivedManeuver( MANEUVER_RIGHT )
  , mReceivedTurnCommand( false )
  , mLastSpeedCommand( SPEED_COMMAND_NONE )
  , mGameFinished( false )
{

  LOGGING_INFO( utilLogger, "Broker constructed." << endl );

  std::string ip = Config::getString( "Kuer", "ServerIP", "10.0.0.1" );
  boost::asio::ip::address address = boost::asio::ip::address::from_string( ip );

  LOGGING_INFO( utilLogger, "Connecting broker to: " << ip << ":" << mPort << endl );            
  std::string carName = Config::getCarName();
  std::transform(carName.begin(), carName.end(), carName.begin(), ::tolower);

  mChannelNames.insert( std::pair<ChannelEnum, std::string>(
        CHANNEL_RECEIVE_COMMAND, "oadrive/" + carName + "/commands") );
  mChannelNames.insert( std::pair<ChannelEnum, std::string>(
        CHANNEL_SEND_STATUS, "oadrive/" + carName + "/info/status") );
  mChannelNames.insert( std::pair<ChannelEnum, std::string>(
        CHANNEL_SEND_DISTANCE, "oadrive/" + carName + "/info/distance") );
  mChannelNames.insert( std::pair<ChannelEnum, std::string>(
        CHANNEL_SEND_MAP, "oadrive/" + carName + "/info/map") );
  mChannelNames.insert( std::pair<ChannelEnum, std::string>(
      CHANNEL_SEND_CHECKPOINT, "oadrive/" + carName + "/info/checkpoint") );
  mChannelNames.insert( std::pair<ChannelEnum, std::string>(
      CHANNEL_SEND_CROSSING, "oadrive/" + carName + "/info/crossing") );

  std::map<ChannelEnum,std::string>::iterator it;
  for( it = mChannelNames.begin(); it != mChannelNames.end(); it++ )
  {
    LOGGING_INFO( utilLogger, "Channel: " << it->first << ") " << it->second << endl );
  } 

#ifdef _IC_BUILDER_REDISCLIENT_
  mSubscriber.asyncConnect( address, mPort,
      boost::bind(
        &Broker::handleConnectedEventSub, this,
        boost::ref( mIOService ),
        boost::ref( mSubscriber ),
        _1, _2 ) );

  mPublisher.asyncConnect( address, mPort,
      boost::bind(
        &Broker::handleConnectedEventPub, this,
        boost::ref( mIOService ),
        boost::ref( mSubscriber ),
        _1, _2 ) );
#else
  LOGGING_WARNING( utilLogger, "Built without redisclient. Broker functions won't work as expected!" << endl );
#endif

  //mIOService.run();
  // Run in seperate thread:
  boost::thread ioThread(boost::bind(&boost::asio::io_service::run, &mIOService));
}

Broker::~Broker()
{
  publish( CHANNEL_SEND_STATUS, "disconnected" );
}

BrokerPtr Broker::getInstance()
{
  if(!mInstance) {
    mInstance = boost::shared_ptr<Broker>(new Broker());
  }
  return mInstance;
}

void Broker::publish( ChannelEnum channel, std::string msg )
{
  if( !isConnected() ) return;

  if( msg.size() < 20 )
  {
    LOGGING_INFO( utilLogger, "Publishing: " << endl <<
        "\t" << mChannelNames[channel] << ": \"" << msg << "\"" << endl );
  } else {
    LOGGING_INFO( utilLogger, "Publishing: " << endl <<
        "\t" << mChannelNames[channel] << " (long message)" << endl );
  }

#ifdef _IC_BUILDER_REDISCLIENT_
  mPublisher.publish( mChannelNames[channel], msg );
#else
  LOGGING_ERROR( utilLogger, "Not publishing a message via redisclient! This was built without redisclient support!" << endl );
#endif
}

bool Broker::isConnected()
{
  bool connected;
  mtx.lock();
  connected = mConnected;
  mtx.unlock();
  return connected;
}

void Broker::setConnected(bool b)
{
  mtx.lock();
  mConnected = b;
  mtx.unlock();
}


void Broker::setLastReceivedManeuver( std::string msg )
{
  mtx.lock();
  if( msg == "left" )
  {
    mLastReceivedManeuver = MANEUVER_LEFT;
  } else if( msg == "straight" ) {
    mLastReceivedManeuver = MANEUVER_STRAIGHT;
  } else {
    mLastReceivedManeuver = MANEUVER_RIGHT;
  }
  mtx.unlock();
}

enumManeuver Broker::getLastReceivedManeuver()
{
  enumManeuver tmp;
  mtx.lock();
  tmp = mLastReceivedManeuver;
  mtx.unlock();
  return tmp;
}

void Broker::setTurnCommandReceived()
{
  mtx.lock();
  mReceivedTurnCommand = true;
  mtx.unlock();
}
bool Broker::getTurnCommandReceived()
{
  bool tmp;
  mtx.lock();
  tmp = mReceivedTurnCommand;
  mReceivedTurnCommand = false;
  mtx.unlock();
  return tmp;
}
void Broker::setNewSpeedCommand( std::string msg )
{
  mtx.lock();
  if( msg == "speed_boost" )
  {
    mLastSpeedCommand = SPEED_COMMAND_BOOST;
  } else if( msg == "speed_normal" ) {
    mLastSpeedCommand = SPEED_COMMAND_NORMAL;
  } else {
    mLastSpeedCommand = SPEED_COMMAND_NONE;
  }
  mtx.unlock();
}
enumSpeedCommand Broker::getLastSpeedCommand()
{
  enumSpeedCommand command;
  mtx.lock();
  command = mLastSpeedCommand;
  mLastSpeedCommand = SPEED_COMMAND_NONE;   // reset
  mtx.unlock();
  return command;
}
void Broker::setHasReceivedStopCommand()
{
  mtx.lock();
  mGameFinished = true;
  mtx.unlock();
}
bool Broker::getHasReceivedStopCommand()
{
  bool tmp;
  mtx.lock();
  tmp = mGameFinished;
  mGameFinished = false;
  mtx.unlock();
  return tmp;
}


bool Broker::isActive()
{
  if( mInstance )
  {
    if( mInstance->isConnected() )
    {
      return true;
    }
  }
  return false;
}

}   // namespace
}   // namespace
