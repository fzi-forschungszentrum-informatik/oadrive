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
/*!
   \brief The Config class read the config files (yaml file format).
You have to init this class with the path and the carname.
It search first in the car folder. If there is no file avaible it searchs in the config folder.
 */
class Config
{
  public:
  /*!
       \brief setConfigPath sets the config Path
       \param folder path to the config folder (It must contain a folder with the carname)
       \param carName name of the car (you can have differnt car folders)
       \return
     */
    static std::string setConfigPath( boost::filesystem::path folder, std::string carName );
    /*!
       \brief getDouble returns a double from the config file
       \param category config file name (without .yml)
       \param key name of the config
       \param defaultVal value which is returned if the entry isn't available
       \return
     */
    static double getDouble( std::string category, std::string key, double defaultVal );
    /*!
       \brief getInt returns a Int from the config file
       \param category config file name (without .yml)
       \param key name of the config
       \param defaultVal value which is returned if the entry isn't available
       \return
     */
    static int getInt( std::string category, std::string key, int defaultVal );
    /*!
       \brief getString returns a String from the config file
       \param category config file name (without .yml)
       \param key name of the config
       \param defaultVal value which is returned if the entry isn't available
       \return
     */
    static std::string getString( std::string category, std::string key, std::string defaultVal );
    /*!
       \brief getMat returns a opencv Matrix from the config file
       \param category file name
       \param key name of the config in the file
       \return
     */
    static cv::Mat getMat( std::string category, std::string key );
    /*!
       \brief getBool returns a bool from the config file
       \param category config file name (without .yml)
       \param key name of the config
       \param defaultVal value which is returned if the entry isn't available
       \return
     */
    static bool getBool( std::string category, std::string key, bool defaultVal );

    static boost::filesystem::path getConfigFile( std::string category );
    /*!
       \brief getCarName returns the car name wich is provided  which is provided by setConfigPath()
       \return car name
     */
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
