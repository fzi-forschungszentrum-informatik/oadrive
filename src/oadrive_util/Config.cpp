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
 * \author  Micha Pfeiffer <ueczz@student.kit.edu>
 * \date    2016-02-22
 *
 */
//----------------------------------------------------------------------

#include "Config.h"
#include "utilLogging.h"
#include <opencv2/core/core.hpp>
using icl_core::logging::endl;
using icl_core::logging::flush;

namespace oadrive {
namespace util {

std::map< std::string, std::map<std::string, std::string> > Config::mDatabaseString;
std::map< std::string, std::map<std::string, double> > Config::mDatabaseDouble;
std::map< std::string, std::map<std::string, int> > Config::mDatabaseInt;
std::map< std::string, std::map<std::string, cv::Mat> > Config::mDatabaseMat;
std::map< std::string, std::map<std::string, bool> > Config::mDatabaseBool;
boost::filesystem::path Config::mConfigFolder;
boost::filesystem::path Config::mCarConfigFolder;
std::string Config::mCarName;
bool Config::mInitialized = false;

std::string Config::setConfigPath( boost::filesystem::path folder, std::string carName )
{
  if( !boost::filesystem::exists(folder) || !boost::filesystem::is_directory(folder))
  {
       std::cout<<"[CONFIG Warning]"<<"Given car config folder doesn't exist!" << std::endl;
      //don't use the Logger because maybe it is not initialised
//    LOGGING_WARNING( utilLogger, "Given config folder doesn't exist!" << endl );
    return "";
  }

  if( !boost::filesystem::exists(folder/carName) ||
      !boost::filesystem::is_directory(folder/carName) )
  {
      std::cout<<"[CONFIG Warning]"<<"Given car config folder doesn't exist!" << std::endl;
      //don't use the Logger because maybe it is not initialised
//    LOGGING_WARNING( utilLogger, "Given car config folder doesn't exist!" << endl );
    return "";
  }

  mConfigFolder = folder;
  mCarConfigFolder = folder / carName;
  mCarName = carName;
  std::cout<<"Config folders:" << std::endl
        << "\t" << mConfigFolder.string() << std::endl
        << "\t" << mCarConfigFolder.string() << std::endl;
  //don't use the Logger because maybe it is not initialised
//  LOGGING_INFO( utilLogger, "Config folders:" << endl
//      << "\t" << mConfigFolder.string() << endl
//      << "\t" << mCarConfigFolder.string() << endl );

  mInitialized = true;
  return mConfigFolder.string();
}

std::string Config::getString( std::string category, std::string key, std::string defaultVal )
{
  assert( mInitialized == true );

  // Check if the value has already been loaded. If so, reuse:
  if( mDatabaseString.count( category ) > 0 )
  {
    if( mDatabaseString[category].count( key ) > 0 )
    {
      return mDatabaseString[category][key];
    }
  }

  try
  {
    boost::filesystem::path filename = getConfigFile( category );

    // Open the file
    cv::FileStorage fs(filename.string(), cv::FileStorage::READ);

    // read the value:
    cv::FileNode node = fs[key.c_str()];
    if( node.empty() )
      throw( "Value doesn't exist" );

    std::string value = (std::string)node;

    // insert the value into the database:
    if( mDatabaseString.count( category ) == 0 )
    {
      std::map<std::string, std::string> newMap;
      mDatabaseString[category] = newMap;
    }
    mDatabaseString[category][key] = value;
    return value;
  } catch( const char* c ) {
    LOGGING_WARNING( utilLogger, "Could not find config value for " << category << "/" << key <<
        " (" << c << ")" << 
        ", using default: " << defaultVal << endl );
  } catch( ... ) {
    LOGGING_WARNING( utilLogger, "Could not find config value for " << category << "/" << key <<
        ", using default: " << defaultVal << endl );
  }
  mDatabaseString[category][key] = defaultVal;    // If we didn't find a value, save the default.
  return defaultVal;
}

double Config::getDouble( std::string category, std::string key, double defaultVal )
{
  assert( mInitialized == true );

  // Check if the value has already been loaded. If so, reuse:
  if( mDatabaseDouble.count( category ) > 0 )
  {
    if( mDatabaseDouble[category].count( key ) > 0 )
    {
      return mDatabaseDouble[category][key];
    }
  }

  try
  {
    boost::filesystem::path filename = getConfigFile( category );

    // Open the file
    cv::FileStorage fs(filename.string(), cv::FileStorage::READ);

    // read the value:
    cv::FileNode node = fs[key.c_str()];
    if( node.empty() )
      throw( "Value doesn't exist" );

    double value = (double)node;

    // insert the value into the database:
    if( mDatabaseDouble.count( category ) == 0 )
    {
      std::map<std::string, double> newMap;
      mDatabaseDouble[category] = newMap;
    }
    mDatabaseDouble[category][key] = value;
    return value;
  } catch( const char* c ) {
    LOGGING_WARNING( utilLogger, "Could not find config value for " << category << "/" << key <<
        " (" << c << ")" << 
        ", using default: " << defaultVal << endl );
  } catch( ... ) {
    LOGGING_WARNING( utilLogger, "Could not find config value for " << category << "/" << key <<
        ", using default: " << defaultVal << endl );
  }
  mDatabaseDouble[category][key] = defaultVal;    // If we didn't find a value, save the default.
  return defaultVal;
}

int Config::getInt( std::string category, std::string key, int defaultVal )
{
  assert( mInitialized == true );

  // Check if the value has already been loaded. If so, reuse:
  if( mDatabaseInt.count( category ) > 0 )
  {
    if( mDatabaseInt[category].count( key ) > 0 )
    {
      return mDatabaseInt[category][key];
    }
  }

  try
  {
    boost::filesystem::path filename = getConfigFile( category );

    // Open the file
    cv::FileStorage fs(filename.string(), cv::FileStorage::READ);

    // read the value:
    cv::FileNode node = fs[key.c_str()];
    if( node.empty() )
      throw( "Value doesn't exist" );

    int value = (int)node;

    // insert the value into the database:
    if( mDatabaseInt.count( category ) == 0 )
    {
      std::map<std::string, int> newMap;
      mDatabaseInt[category] = newMap;
    }
    mDatabaseInt[category][key] = value;
    return value;
  } catch( const char* c ) {
    LOGGING_WARNING( utilLogger, "Could not find config value for " << category << "/" << key <<
        " (" << c << ")" << 
        ", using default: " << defaultVal << endl );
  } catch( ... ) {
    LOGGING_WARNING( utilLogger, "Could not find config value for " << category << "/" << key <<
        ", using default: " << defaultVal << endl );
  }

  mDatabaseInt[category][key] = defaultVal;    // If we didn't find a value, save the default.
  return defaultVal;
}

cv::Mat Config::getMat( std::string category, std::string key )
{
  assert( mInitialized == true );

  // Check if the value has already been loaded. If so, reuse:
  if( mDatabaseMat.count( category ) > 0 )
  {
    if( mDatabaseMat[category].count( key ) > 0 )
    {
      return mDatabaseMat[category][key];
    }
  }

  try
  {
    boost::filesystem::path filename = getConfigFile( category );

    // Open the file
    cv::FileStorage fs(filename.string(), cv::FileStorage::READ);

    // read the value:
    cv::FileNode node = fs[key.c_str()];
    if( node.empty() )
      throw( "Value doesn't exist" );

    cv::Mat value;

    // Ugly, but works:
    if( node.type() == cv::FileNode::SEQ )
    {
      std::vector<double> seq;
      fs[key.c_str()] >> seq;
      for( size_t i = 0; i < seq.size(); i++ )
      {
        std::cout << i << ": " << seq[i] << std::endl;
      }
      value = cv::Mat( seq, true );
    } else {
      cv::read( node, value );
    }

    // insert the value into the database:
    if( mDatabaseMat.count( category ) == 0 )
    {
      std::map<std::string, cv::Mat> newMap;
      mDatabaseMat[category] = newMap;
    }
    mDatabaseMat[category][key] = value;
    return value;
  } catch( const char* c ) {
    LOGGING_WARNING( utilLogger, "Could not find config value for " << category << "/" << key <<
        " (" << c << ")" << endl );
  } catch( ... ) {
    LOGGING_WARNING( utilLogger, "Could not find config value for " << category << "/" << key <<
        endl );
  }
  mDatabaseMat[category][key] = cv::Mat();    // If we didn't find a value, save the default.
  return cv::Mat();
}

bool Config::getBool( std::string category, std::string key, bool defaultVal )
{
  assert( mInitialized == true );

  // Check if the value has already been loaded. If so, reuse:
  if( mDatabaseBool.count( category ) > 0 )
  {
    if( mDatabaseBool[category].count( key ) > 0 )
    {
      return mDatabaseBool[category][key];
    }
  }

  try
  {
    boost::filesystem::path filename = getConfigFile( category );

    // Open the file
    cv::FileStorage fs(filename.string(), cv::FileStorage::READ);

    // read the value:
    cv::FileNode node = fs[key.c_str()];
    if( node.empty() )
      throw( "Value doesn't exist" );

    int value = (int)node;

    // insert the value into the database:
    if( mDatabaseBool.count( category ) == 0 )
    {
      std::map<std::string, bool> newMap;
      mDatabaseBool[category] = newMap;
    }
    mDatabaseBool[category][key] = value;
    return (value != 0);
  } catch( const char* c ) {
    LOGGING_WARNING( utilLogger, "Could not find config value for " << category << "/" << key <<
        " (" << c << ")" << 
        ", using default: " << defaultVal << endl );
  } catch( ... ) {
    LOGGING_WARNING( utilLogger, "Could not find config value for " << category << "/" << key <<
        ", using default: " << defaultVal << endl );
  }
  mDatabaseBool[category][key] = defaultVal;    // If we didn't find a value, save the default.
  return defaultVal;
}

boost::filesystem::path Config::getConfigFile( std::string category )
{
  assert( mInitialized == true );

  boost::filesystem::path p = mConfigFolder;
  if( !exists( p ) )
    throw( "Folder doesn't exist" );

  // Construct the filename:
  boost::filesystem::path filename = mCarConfigFolder;
  filename /= (category + ".yml" );
  if( !exists( filename ) || !is_regular_file( filename ) )
  {
    filename = p;
    filename /= (category + ".yml" );
  }

  if( !exists( filename ) || !is_regular_file( filename ) )
    throw( "File doesn't exist" );

  return filename;
}

std::string Config::getCarName()
{
  assert( mInitialized == true );
  return mCarName;
}

} // namespace
} // namespace
