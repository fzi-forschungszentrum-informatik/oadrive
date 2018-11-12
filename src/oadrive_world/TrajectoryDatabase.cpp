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
 * \author  David Zimmerer <dzimmerer@gmail.com>
 * \author  Micha Pfeiffer <ueczz@student.kit.edu>
 * \date    2016-02-06
 * 
 * \author  Mark Hueneberg <hueneber@fzi.de>
 * \date    2018
 *
 */
//----------------------------------------------------------------------

#include "TrajectoryDatabase.h"

#include <fstream>
#include <boost/filesystem.hpp>
#include <oadrive_world/worldLogging.h>
#include <boost/regex.hpp>

using icl_core::logging::endl;
using icl_core::logging::flush;

using namespace oadrive::core;
using namespace boost::filesystem;
using namespace boost;

namespace oadrive{
namespace world{

std::map<std::string, MultiTrajectory> TrajectoryDatabase::database;

void TrajectoryDatabase::load( std::string directory )
{
  clear();
  path p( directory );
  LOGGING_INFO(worldLogger, "TrajectoryDatabase loading database:" << endl);
  LOGGING_INFO(worldLogger, "\t" << p.string() << endl);

  // Match any trajectory with a _[number].txt at the end:
  regex filenamePattern("(.*)_([0-9]+)");

  if( is_directory( p ) )
  {
    std::vector<path> files;             // store paths,

    copy(directory_iterator(p), directory_iterator(), back_inserter(files));

    sort(files.begin(), files.end());             // sort, since directory iteration
    // is not ordered on some file systems

    for (std::vector<path>::iterator it (files.begin()); it != files.end(); ++it)
    {
      std::string extension = (*it).extension().string();
      if( strcmp( extension.c_str(), ".txt" ) == 0 )
      {
        std::string filename = (*it).string();
        //LOGGING_INFO( worldLogger, "Loading " << filename << endl );
        std::string filenameStem = (*it).stem().string();
        std::ifstream inFile( filename.c_str() );

        Trajectory2d traj;
        inFile >> traj;
        inFile.close();

        boost::match_results<std::string::const_iterator> res;
        if(regex_match( filenameStem, res, filenamePattern ))
        {
          // res[0] contains the whole string
          // res[1] contains the base name
          // res[2] contains the number
          std::string trajectoryName = res[1];

          if( !database.count( trajectoryName ) )
          {
            MultiTrajectory newTraj;
            newTraj.trajectories.push_back( traj );
            database.insert(
                  std::pair<std::string, MultiTrajectory>
                  ( trajectoryName, newTraj ) );
          } else {
            database[trajectoryName].trajectories.push_back( traj );
          }
        } else {
          LOGGING_WARNING( worldLogger, "\tCannot interpret " << filenameStem <<
                           " as part of a multi-trajectory." << endl );
        }
      }
    }
  }
  LOGGING_INFO( worldLogger, "TrajectoryDatabase loaded " << database.size()
                << " trajectories:" << endl );

  std::map< std::string, MultiTrajectory >::iterator it;
  for( it = database.begin(); it != database.end(); it++ )
  {
    LOGGING_INFO( worldLogger, "\tTrajectory: '" << it->first << "':" << endl );
    for( unsigned int j = 0; j < (it->second).trajectories.size(); j++ )
    {
      LOGGING_INFO( worldLogger, "\t\t" << j << ": " << (it->second).trajectories[j].size() << " points" << endl );
    }
  }
}

MultiTrajectory TrajectoryDatabase::getTrajectory( std::string name )
{
  if( database.count( name ) )
  {
    return database[name];
  } else {
    std::string msg;
    msg.append( "Trajectory " );
    msg.append( name );
    msg.append( " does not exist in TrajectoryDatabase." );
    if( database.size() == 0 )
      msg.append( " Make sure to call TrajectoryDatabase::load first!" );
    std::cout << msg << std::endl;
    throw( msg.c_str() );
  }
}

Trajectory2d TrajectoryDatabase::getSingleTrajectory( std::string name )
{
  if( database.count( name ) )
  {
    return database[name].trajectories[0];
  } else {
    std::string msg;
    msg.append( "Trajectory " );
    msg.append( name );
    msg.append( " does not exist in TrajectoryDatabase." );
    if( database.size() == 0 )
      msg.append( " Make sure to call TrajectoryDatabase::load first!" );
    std::cout << msg << std::endl;
    throw( msg.c_str() );
  }
}

bool TrajectoryDatabase::hasTrajectory( std::string name )
{
  if( database.count( name ) )
  {
    return true;
  } else {
    return false;
  }
}

void TrajectoryDatabase::clear()
{
  database.clear();
}

}		// namespace
}		// namespace
