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

#ifndef OADRIVE_UTIL_CONFIG_H
#define OADRIVE_UTIL_CONFIG_H

#include <string>
#include <map>
#include <boost/filesystem.hpp>
#include <opencv2/core/core.hpp>

namespace oadrive {
namespace util {

class Config
{
  public:
    static std::string setConfigPath( boost::filesystem::path folder, std::string carName );

    static double getDouble( std::string category, std::string key, double defaultVal );
    static int getInt( std::string category, std::string key, int defaultVal );
    static std::string getString( std::string category, std::string key, std::string defaultVal );
    static cv::Mat getMat( std::string category, std::string key );
    static bool getBool( std::string category, std::string key, bool defaultVal );

    static boost::filesystem::path getConfigFile( std::string category );

    static std::string getCarName();
  private:
    Config();   // private constructor -> "static class"

    /*! Holds all values which were already loaded
     * indexing: map[category][key] = value; */
    static std::map< std::string, std::map<std::string, std::string> > mDatabaseString;
    static std::map< std::string, std::map<std::string, double> > mDatabaseDouble;
    static std::map< std::string, std::map<std::string, int> > mDatabaseInt;
    static std::map< std::string, std::map<std::string, cv::Mat> > mDatabaseMat;
    static std::map< std::string, std::map<std::string, bool> > mDatabaseBool;
    static boost::filesystem::path mConfigFolder;
    static boost::filesystem::path mCarConfigFolder;
    static std::string mCarName;
    static bool mInitialized;
};

} // namespace
} // namespace

#endif // OADRIVE_UTIL_CONFIG_H
