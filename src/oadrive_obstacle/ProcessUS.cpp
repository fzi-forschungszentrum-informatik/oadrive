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
 * \date    2015-11-01
 *
 */
//----------------------------------------------------------------------

#include "ProcessUS.h"
#include "obstacleLogging.h"
#include <oadrive_world/Environment.h>

using namespace oadrive::world;
using icl_core::logging::endl;
using icl_core::logging::flush;
namespace oadrive{
namespace obstacle{

ProcessUS::ProcessUS():
  MAXDIST(0.8)
,MINDIST(0.08)
{
  for(int sensor=0;sensor<NUMBERSENSOR;sensor++)
  {
    for(int calPointNumber = 0; calPointNumber<NUMBERCALPOINTSPERSENSOR; calPointNumber++)
    {
      mCalPoints[sensor][calPointNumber].x = 0;
      mCalPoints[sensor][calPointNumber].y = 0;
      mCalPoints[sensor][calPointNumber].distance = 0;

    }
    mUsSensorPos[sensor].angle = 0;
    mUsSensorPos[sensor].distanceOffset = 0;
    mUsSensorPos[sensor].yOffset = 0;
  }

}
ProcessUS::ProcessUS(std::string calFile):
  MAXDIST(1)
,MINDIST(0.05)
{
  for(int sensor=0;sensor<NUMBERSENSOR;sensor++)
  {
    for(int calPointNumber = 0; calPointNumber<NUMBERCALPOINTSPERSENSOR; calPointNumber++)
    {
      mCalPoints[sensor][calPointNumber].x = 0;
      mCalPoints[sensor][calPointNumber].y = 0;
      mCalPoints[sensor][calPointNumber].distance = 0;

    }
    mUsSensorPos[sensor].angle = 0;
    mUsSensorPos[sensor].distanceOffset = 0;
    mUsSensorPos[sensor].yOffset = 0;
  }
  loadCalPoints(calFile);

}

void ProcessUS::saveCal(std::string path)
{
  cv::FileStorage fs(path, cv::FileStorage::WRITE);
  fs << "sensorCalPoints" << "[";
  for(int sensor=0;sensor<NUMBERSENSOR;sensor++)
  {
    fs << "{:" << "sensorNumber" << sensor <<"xyd"<<"[:";
    for(int calPointNumber = 0; calPointNumber<NUMBERCALPOINTSPERSENSOR; calPointNumber++)
    {
      fs<<mCalPoints[sensor][calPointNumber].x<<mCalPoints[sensor][calPointNumber].y<<mCalPoints[sensor][calPointNumber].distance;

    }
    fs << "]" << "}";

  }
  fs<<"]";
  fs.release();
}

void ProcessUS::loadCalPoints(std::string path)
{
  cv::FileStorage fs2;
  fs2.open(path, cv::FileStorage::READ);
  if (!fs2.isOpened())
  {
    LOGGING_ERROR(obstacleLogger,"Failed to open cal file for us-sensor (can't open file)"<<endl);
  }
  else
  {
    cv::FileNode sensorNode = fs2["sensorCalPoints"];
    if(sensorNode.empty())
    {
      LOGGING_ERROR(obstacleLogger,"Failed to open cal file for us-sensor (can't open node)"<<endl);
    }
    else
    {
      cv::FileNodeIterator it = sensorNode.begin(), it_end = sensorNode.end();
      // iterate through a sequence using FileNodeIterator
      for( ; it != it_end; ++it )
      {
        int sensorNumber = 0;
        sensorNumber = (int)(*it)["sensorNumber"];

        for(int calPointNumber = 0; calPointNumber<NUMBERCALPOINTSPERSENSOR; calPointNumber++)
        {
          cv::FileNode calPointIt = (*it)["xyd"];
          calPointIt[(calPointNumber*3)+0]>>mCalPoints[sensorNumber][calPointNumber].x;
          calPointIt[(calPointNumber*3)+1]>>mCalPoints[sensorNumber][calPointNumber].y;
          calPointIt[(calPointNumber*3)+2]>>mCalPoints[sensorNumber][calPointNumber].distance;
        }
      }
      LOGGING_INFO(obstacleLogger,"Open cal file for us-sensor"<<endl);

      calcSensorPos();
    }
  }

}

oadrive::core::ExtendedPose2d ProcessUS::transformToCar(int sensorNumber, double distance)
{

  double x = std::sin(mUsSensorPos[sensorNumber].angle)*(distance+mUsSensorPos[sensorNumber].distanceOffset);
  double y = std::cos(mUsSensorPos[sensorNumber].angle)*(distance+mUsSensorPos[sensorNumber].distanceOffset)+mUsSensorPos[sensorNumber].yOffset;
  oadrive::core::ExtendedPose2d postion(x,y,0);

  return postion;
}

oadrive::core::ExtendedPose2dVectorPtr ProcessUS::getObjects(usSensor sensor)
{
  usSensor* limits = Environment::getInstance()->getCurrentUSSensorLimits();

  oadrive::core::ExtendedPose2dVectorPtr objectPoses(new oadrive::core::ExtendedPose2dVector());
  if( sensor.frontLeft < limits->frontLeft && sensor.frontLeft>MINDIST)
  {
    objectPoses->push_back(transformToCar(0,sensor.frontLeft));
  }
  if( sensor.frontCenterLeft < limits->frontCenterLeft && sensor.frontCenterLeft > MINDIST )
  {
    objectPoses->push_back(transformToCar(1,sensor.frontCenterLeft));
  }
  if( sensor.frontCenter < limits->frontCenter && sensor.frontCenter > 0 )
  {
    objectPoses->push_back(transformToCar(2,sensor.frontCenter));
  }
  if( sensor.frontCenterRight < limits->frontCenterRight && sensor.frontCenterRight > MINDIST )
  {
    objectPoses->push_back(transformToCar(3,sensor.frontCenterRight));
  }
  if( sensor.frontRight < limits->frontRight && sensor.frontRight > MINDIST )
  {
    objectPoses->push_back(transformToCar(4,sensor.frontRight));
  }
  if( sensor.sideRight < limits->sideRight && sensor.sideRight > MINDIST )
  {
    objectPoses->push_back(transformToCar(5,sensor.sideRight));
  }
  if( sensor.rearRight < limits->rearRight && sensor.rearRight > MINDIST )
  {
    objectPoses->push_back(transformToCar(6,sensor.rearRight));
  }
  if( sensor.rearCenter < limits->rearCenter && sensor.rearCenter > MINDIST )
  {
    objectPoses->push_back(transformToCar(7,sensor.rearCenter));
  }
  if( sensor.rearLeft < limits->rearLeft && sensor.rearLeft > MINDIST )
  {
    objectPoses->push_back(transformToCar(8,sensor.rearLeft));
  }
  if( sensor.sideLeft < limits->sideLeft && sensor.sideLeft > MINDIST )
  {
    objectPoses->push_back(transformToCar(9,sensor.sideLeft));
  }

  return objectPoses;

}

void ProcessUS::printCalPoints()
{
  for(int sensor=0;sensor<NUMBERSENSOR;sensor++)
  {
    std::cout <<"[convertUs] Calibration Points of Sensor" << sensor <<std::endl <<" [convertUs] Points:"<<std::endl;
    for(int calPointNumber = 0; calPointNumber<NUMBERCALPOINTSPERSENSOR; calPointNumber++)
    {
      std::cout<<"[convertUs] Nr."<<calPointNumber<<": X:"<<mCalPoints[sensor][calPointNumber].x<<" Y:"<<mCalPoints[sensor][calPointNumber].y<<" D:"<<mCalPoints[sensor][calPointNumber].distance <<std::endl;
    }
  }
}

void ProcessUS::printSensorPos()
{
  for(int sensor=0;sensor<NUMBERSENSOR;sensor++)
  {
    std::cout <<"[convertUs] Position of Sensor" << sensor <<std::endl;
    std::cout<<"yOffset"<<mUsSensorPos[sensor].yOffset<<" Angle"<<(mUsSensorPos[sensor].angle/(2*M_PI))*360<<" D Offset:"<<mUsSensorPos[sensor].distanceOffset<<std::endl;
  }
}

void ProcessUS::calcSensorPos()
{
  //calculate yOffset
  //we asume the angle a is constant
  //tan(a)= x/(y-yOffset)
  //we have 2 Points (x1 y1) so we can equate tan(a) with a second equation for the yOffset

  //(doffset+d)^2 = x^2+(y-yoffset)^2
  for(int sensor=0;sensor<NUMBERSENSOR;sensor++)
  {
    if(sensor == 2)
    {
      mUsSensorPos[sensor].yOffset = mCalPoints[sensor][0].y-mCalPoints[sensor][0].distance;
      mUsSensorPos[sensor].distanceOffset = 0;
      mUsSensorPos[sensor].angle = 0;

    }
    else if(sensor == 7)
    {
      mUsSensorPos[sensor].yOffset = mCalPoints[sensor][0].y+mCalPoints[sensor][0].distance;
      mUsSensorPos[sensor].distanceOffset = 0;
      mUsSensorPos[sensor].angle = -M_PI;
    }
    else if(sensor == 5)
    {
      mUsSensorPos[sensor].yOffset = mCalPoints[sensor][0].y;
      mUsSensorPos[sensor].distanceOffset = mCalPoints[sensor][0].x-mCalPoints[sensor][0].distance;
      mUsSensorPos[sensor].angle = (M_PI)/2;

    }
    else if(sensor == 9)
    {
      mUsSensorPos[sensor].yOffset = mCalPoints[sensor][0].y;
      mUsSensorPos[sensor].distanceOffset = -mCalPoints[sensor][0].x-mCalPoints[sensor][0].distance;
      mUsSensorPos[sensor].angle = -(M_PI)/2;
    }
    else
    {
      double x0 = mCalPoints[sensor][0].x;
      double x1 = mCalPoints[sensor][1].x;
      double y0 = mCalPoints[sensor][0].y;
      double y1 = mCalPoints[sensor][1].y;
      double d0 = mCalPoints[sensor][0].distance;
      mUsSensorPos[sensor].yOffset = (x0*y1-x1*y0)/(x0-x1);
      mUsSensorPos[sensor].angle = atan2(x0,(y0-mUsSensorPos[sensor].yOffset));
      double yHelp = y0-mUsSensorPos[sensor].yOffset;
      mUsSensorPos[sensor].distanceOffset = std::sqrt(x0*x0+(yHelp*yHelp))-d0;
    }

  }



}

void ProcessUS::setCalPoint(int sensorNumber, double x, double y, double d, int calPointNumber)
{
  assert (sensorNumber <= NUMBERSENSOR-1);
  assert (calPointNumber <= NUMBERCALPOINTSPERSENSOR-1);
  mCalPoints[sensorNumber][calPointNumber].x = x;
  mCalPoints[sensorNumber][calPointNumber].y = y;
  mCalPoints[sensorNumber][calPointNumber].distance = d;
  //std::cout<<"setCalPoint Nr." <<calPointNumber<< "sensor Nr."<<sensorNumber<< " x:"<<x<<" y:"<<y<<" distance "<<d<<std::endl;
}

void ProcessUS::getValuesFromCons()
{
  std::cout<<"distanz offset eingeben"<<std::endl;
  double dOffset;
  std::cin>>dOffset;
  while (1)
  {
    std::cout<<"Sensor Nummer eingeben"<<std::endl;
    int sensor;
    std::cin>>sensor;
    for(int i = 0; i<NUMBERCALPOINTSPERSENSOR;i++)
    {
      std::cout<<"X eingeben"<<std::endl;
      double x;
      std::cin>>x;
      std::cout<<"Y eingeben"<<std::endl;
      double y;
      std::cin>>y;
      std::cout<<"Distance eingeben"<<std::endl;
      double d;
      std::cin>>d;
      //offset korrigieren
      d = d-dOffset;
      setCalPoint(sensor,x,y,d,i);
      std::cout<<"Bitte Punkt Nr"<<i+1<<"eingeben"<<std::endl;
    }

    std::cout<<"s eingeben fuer speicher w fuer weiter"<<std::endl;
    char next;
    std::cin>>next;
    if(next == 's')
    {
      saveCal("/tmp/test/test.yml");
      std::cout<<"gespeichert"<<std::endl;
      break;
    }
  }
}

}	// namespace
}	// namespace
