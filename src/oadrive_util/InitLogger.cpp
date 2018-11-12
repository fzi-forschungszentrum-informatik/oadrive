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
 * \author  Peter Zimmer <peter-zimmer@gmx.net>
 * \date    2016-01-17
 *
 */
//----------------------------------------------------------------------

#include "InitLogger.h"
#include "string.h"
#include "sstream"
#include <fstream>
#include <icl_core_logging/Logging.h>
#include <boost/filesystem.hpp>
using namespace boost::filesystem;
namespace oadrive {
namespace util {
InitLogger::InitLogger(std::string filename)
{
  // Look for a log config in the current directory:
  std::ifstream fileTest( "log.xml" );
  if( fileTest.is_open() )
  {
    std::cout << "Warning:\n\tFound 'log.xml' in current directory. Will use this instead of " << std::endl
              << "\t'" << filename << "'." << std::endl;
    filename = "log.xml";
  }

  // Create the folder to log to, just in case.
  create_directories( "/tmp/recordedData/" );

  if( ! icl_core::logging::LoggingManager::instance().initialized() )
  {
    //lets Fake a commandline
    int argcspoof = 2;
    std::string exPath = "fakeExe";
    std::stringstream sstr;
    sstr<<"--configfile="<<filename;
    //std::string parameter = "--configfile=/home/peter/Eigene_dateien/AADC/FZI/robot_folders/checkout/main/build-ic_workspace-Desktop-Default/bin/log.conf";
    char buffer[exPath.length()+1];
    char buffer2[sstr.str().length()+1];
    std::strcpy (buffer, exPath.c_str());
    std::strcpy (buffer2, sstr.str().c_str());
    char* arg[2];
    arg[0] = buffer;
    arg[1] = buffer2;

    icl_core::logging::initialize(argcspoof,arg);
  }
  //ToDo
  // boost::shared_ptr<icl_core::logging::LifeCycle> icl_core::logging::autoStart(argcspoof,arg);

}

InitLogger::~InitLogger()
{
  icl_core::logging::shutdown();
  std::cerr<<"Logger Shutdown"<<std::endl;

}
}
}
