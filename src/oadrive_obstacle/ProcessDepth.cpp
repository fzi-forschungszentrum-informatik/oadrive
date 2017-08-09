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
 * \author  Micha Pfeiffer <ueczz@student.kit.edu>
 * \date    2015-11-01
 *
 */
//----------------------------------------------------------------------

#include "ProcessDepth.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <algorithm>    //for median
#include <functional>   //for median
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <list>
#include <string>
#include <oadrive_obstacle/obstacleLogging.h>
#include <oadrive_world/Environment.h>

#define noPng //if you deactivate this you must pass a png file as input file
using icl_core::logging::endl;
using icl_core::logging::flush;

namespace oadrive{
namespace obstacle{

ProcessDepth::ProcessDepth( std::string refPath, oadrive::util::CoordinateConverter *converter )
  : mPosConv(converter)
  , MIN_AREA(100 )
  , mReferenceImageThreshold( 0.05 )	// in meters
  , mMaxViewDepth( 3.0 )  // in meters
{

  /*#ifdef noPng
    LOGGING_INFO(obstacleLogger,"Try to read Depth Ref Image (as yml) "<<refPath<<endl);
    cv::Mat refImg;
    cv::FileStorage fs2(refPath, cv::FileStorage::READ);
    fs2["refImg"]>>refImg;
#else
    LOGGING_INFO(obstacleLogger,"Try to read Depth Ref Image(as png) "<<refPath<<endl);
    cv::Mat refImg = cv::imread(refPath, CV_LOAD_IMAGE_ANYDEPTH|CV_LOAD_IMAGE_GRAYSCALE);
#endif*/

  generateDepthReferenceImage();
  cv::imwrite("/tmp/depthRef.png",mRefImage);

  /*if(refImg.empty())
    {
       LOGGING_ERROR(obstacleLogger,"There is no Ref Image for Depth Processing "<<endl);
    }
    setRef(refImg,true);*/

}

void ProcessDepth::processDepthImageDebug(cv::Mat image)
{
  cv::Mat imageBackup;
  image.copyTo(imageBackup);
  cv::Mat output;
  cv::cvtColor( image, output, CV_GRAY2BGRA );
  imshow( "input", image );
  cv::Mat clean = calcCleanImg(image);

  cv::Mat show;
  clean.convertTo(show,CV_8U);
  cv::cvtColor(show,show,cv::COLOR_GRAY2BGR);

  std::vector<cv::Point2d> objects;
  std::vector<int> depth;

  findObjects(clean,imageBackup,objects,depth);
  for( unsigned int i = 0; i< objects.size(); i++ )
  {
    cv::circle(show,objects[i],4,cv::Scalar(0,0,255));
    //std::cout<<mPosConv.depthPixel2World(carPose,objects[i])<<std::endl;
  }

  cv::imshow("conturs",show);
  cv::waitKey(1);

}

ExtendedPose2dVectorPtr ProcessDepth::getObjects(cv::Mat image)
{
  cv::Mat imageBackup = image.clone();//TODO remove this
  cv::Mat clean = calcCleanImg(image);
  std::vector<cv::Point2d> objects;
  std::vector<int> depth;
  findObjects(clean,imageBackup,objects,depth);
  ExtendedPose2dVectorPtr objectPoses(new oadrive::core::ExtendedPose2dVector());
  for( unsigned int i = 0; i< objects.size(); i++ )
  {
    oadrive::core::ExtendedPose2d ObjectPosition;
    if(depth[i] != 0){
      //            std::cout<<"Depth: "<<depth[i]<<"   "<<std::endl;
      //            std::cout<<"Pos in Car "<<mPosConv->depthPixel2Car(objects[i],(float)((float)depth[i]/1000.0))<<"   "<<std::endl;
      ObjectPosition = mPosConv->car2World(
          Environment::getInstance()->getCarPose(),
          mPosConv->depthPixel2Car(objects[i],(float)((float)depth[i]/1000.0)));
      objectPoses->push_back(ObjectPosition);
    }

    //        if(mPosConv->depthPixel2World(mEnviroment->getCarPose(),objects[i],ObjectPosition))
    //        {objectPoses->push_back(ObjectPosition);}
  }
  return objectPoses;
}



void ProcessDepth::setRef(cv::Mat image, bool improve)
{
  if(improve)
  {
    for(int imgY= 0; imgY<image.rows;imgY++)
    {
      cv::Mat lineOfImage = image.row(imgY);
      std::vector<u_int16_t>  tempSort;
      //            std::nth_element(line.begin<u_int16_t>, line.begin<u_int16_t> + line.cols/2, line.end<u_int16_t>());
      cv::MatIterator_<u_int16_t> it, end;
      for( it = lineOfImage.begin<u_int16_t>(), end = lineOfImage.end<u_int16_t>(); it != end; ++it)
      {
        tempSort.push_back(*it);
      }
      std::nth_element(tempSort.begin(), tempSort.begin() + tempSort.size()/2, tempSort.end());
      u_int16_t median = tempSort[tempSort.size()/2];
      cv::line( image,cv::Point2i(0,imgY),cv::Point2i(image.cols,imgY),median,1,8);
    }
    image.convertTo(mRefImage,CV_32FC1);
    // RefImage = image;
  }
  else
  {
    mRefImage = image;

  }
}

cv::Mat ProcessDepth::calcCleanImg(cv::Mat image)
{
  cv::Mat convert;
  image.convertTo(convert,CV_32FC1);
  cv::Mat noZero;
  cv::threshold(convert,noZero,1,65534,cv::THRESH_BINARY_INV);
  noZero = noZero+convert;
  cv::Mat ret;
  cv::threshold(mRefImage-noZero,ret,1,65534,cv::THRESH_BINARY);
  return ret;
}
cv::Mat ProcessDepth::getRefImage() const
{
  return mRefImage;
}

void ProcessDepth::applyClosing(cv::Mat &image)
{
  int morph_size = 3;
  cv::Mat element = getStructuringElement( cv::MORPH_RECT, cv::Size( 2*morph_size + 1, 2*morph_size+1 ), cv::Point( morph_size, morph_size ) );
  // Apply the specified morphology operation
  cv::morphologyEx( image, image, CV_MOP_CLOSE, element, cv::Point(-1,-1), 3);

  int erodeSize = 1;
  element = getStructuringElement( cv::MORPH_RECT, cv::Size( 2*erodeSize + 1, 2*erodeSize+1 ) );
  // Apply the erosion operation
  cv::erode( image, image, element );
}

oadrive::util::CoordinateConverter *ProcessDepth::getPosConv() const
{
  return mPosConv;
}

void ProcessDepth::setPosConv(oadrive::util::CoordinateConverter *value)
{
  mPosConv = value;
}

cv::Mat ProcessDepth::getDebugImage(cv::Mat image)
{
  cv::Mat clean = calcCleanImg(image);
  std::vector<cv::Point2d> objects;
  std::vector<int> depth;
  findObjects(clean,image,objects,depth);
  cv::cvtColor(clean,clean,CV_GRAY2BGR);
  clean.convertTo(clean,CV_8UC3);
  for( unsigned int i = 0; i< objects.size(); i++ )
  {
    cv::circle(clean,objects.at(i),3,cv::Scalar(0,0,255));
  }
  cv::line( clean,
      cv::Point2d( 0, mHorizonRow ),
      cv::Point2d( clean.cols, mHorizonRow ),
      cv::Scalar(0,255,255));
  cv::line( clean,
      cv::Point2d( 0, mMaxDepthRow ),
      cv::Point2d( clean.cols, mMaxDepthRow ),
      cv::Scalar(0,128,255));
  //cv::line(clean,cv::Point2d(0,mPosConv->getFirstRow()/2),cv::Point2d(clean.cols,mPosConv->getFirstRow()/2),cv::Scalar(0,255,0));
  //cv::line(clean,cv::Point2d(0,mPosConv->getLastRow()/2),cv::Point2d(clean.cols,mPosConv->getLastRow()/2),cv::Scalar(0,255,0));
  return clean;
}

void ProcessDepth::findObjects(cv::Mat imageU,cv::Mat original,std::vector<cv::Point2d> &objects,std::vector<int> &depth)
{
  //cv::imwrite("imageU.png", imageU);
  //cv::imwrite("imageOriginal.png", original);
  cv::Mat imageBinary;
  imageU.convertTo(imageBinary,CV_8U);
  //cv::imwrite("imageBinary.png", imageBinary);
  applyClosing(imageBinary);
  //cv::imwrite("imageClosed.png", imageBinary);

  std::vector<std::vector<cv::Point> > contours;
  cv::findContours(imageBinary,contours,CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
  std::vector<std::vector<cv::Point> > contours_poly;
  //simplyfy contours and cheack area
  for(unsigned int i = 0; i < contours.size(); i++ )
  {
    std::vector<cv::Point> tmp;
    cv::approxPolyDP( cv::Mat(contours[i]), tmp, 3, true );
    if(cv::contourArea(tmp)>MIN_AREA)
    {
      contours_poly.push_back(tmp);
    }
  }

  //find bounding rectangle and estimate the front of the object
  for( unsigned int i = 0; i < contours_poly.size(); i++ )
  {
    cv::Rect boundRect;
    cv::Point2d pos;
    boundRect = cv::boundingRect( cv::Mat(contours_poly[i]) );

    // Discretize: Search along the x axis and generate multiple (up to maxNumObstacles)
    // obstacles:
    //int maxNumObstacles = 5;
    int dx = boundRect.width/5;
    char filled;
    ushort currentDepth;
    for( int xPos = 0; xPos < boundRect.width; xPos += dx )
    {
      for( int yPos = boundRect.height-1; yPos >= 0; --yPos )
      {
        pos.x = boundRect.tl().x + xPos;
        pos.y = boundRect.tl().y + yPos;
        filled = imageBinary.at<char>( pos.y, pos.x );
        if( filled > 0 )	// bottom of object at this xPos found:
        {
          currentDepth = original.at<ushort>( pos.y, pos.x );
          // Only accept objects which are in front of the car. Other objects are probably
          // phantoms which were created by erosion/dilation of the images.
          if( currentDepth > 250 )
          {
            // TODO: Remove?
            // Hack: because we detect object side, let's approximate its
            // center by moving back the object by 7 cm:
            //currentDepth = currentDepth + 70;

            depth.push_back( currentDepth );
            objects.push_back( pos );
            break;
          }
        }
      }
    }
  }
}

void ProcessDepth::generateDepthReferenceImage()
{
  // Generates a depth-reference image (i.e. a "perfect" ground plane).
  // This image is subtracted from each new depth image.
  // Note: We found too little documentation on the depth image of the
  // xtion camera used. The way the ground plane is calculated here seems to be
  // precise enough for our purposes, but if you want to do things which depend on
  // a higher precision, please do not take for granted that this method will work!
  mRefImage = cv::Mat( cv::Size( DEPTH_IMAGE_WIDTH, DEPTH_IMAGE_HEIGHT ), CV_32FC1 );

  bool maxDepthReached = false;
  float maxDepth = 0;
  for( unsigned int i = DEPTH_IMAGE_HEIGHT-1; i > 0; i -- )
  {
    // Only the y pixel coordinate value is used in calculation, so set x to 0:
    float depth = mPosConv->calcZeroPlaneDepth( cv::Point2f( 0, i ),
                                                mReferenceImageThreshold );


    if(depth < mMaxViewDepth && !maxDepthReached)
    {
      mRefImage.row( i ) = cv::Scalar( (unsigned int)abs(depth*1000) );
      mMaxDepthRow = i;
    }
    else
    {
      maxDepthReached = true;
      mRefImage.row( i ) = cv::Scalar( (unsigned int)0 ) ;
    }

    // Search for the line which is the furthest away. This is our horizon.
    if( depth > maxDepth )
    {
      maxDepth = depth;
      mHorizonRow = i;
    }
  }

  //cv::imwrite( "mRefImage.png", mRefImage );
}

}   // namespace
}   // namespace


