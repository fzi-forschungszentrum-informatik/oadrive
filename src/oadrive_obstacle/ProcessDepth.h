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
 * \date    2015-11-01
 *
 */
//----------------------------------------------------------------------

#ifndef OADRIVE_OBSTACLE_DEPTHIMAGE_H
#define OADRIVE_OBSTACLE_DEPTHIMAGE_H

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

#include <oadrive_world/Environment.h>
namespace oadrive{
namespace obstacle{

class ProcessDepth
{
public:
  //! \brief DepthImage
  //! \param refPath path to ref Image. (Must be an image without any obstacles)
  ProcessDepth(std::string refPath, util::CoordinateConverter *converter);

  //! \brief processImage processes the depthImage from segmenting to enviroment
  //! \param image depthImage
  void processDepthImageDebug(cv::Mat image);

  ExtendedPose2dVectorPtr getObjects(cv::Mat image);

  /*!
     * \brief setRef set refernce Image
     * \param improve if True at each Row the median is calculated an set to the whole row
     */
  void setRef(cv::Mat, bool improve);
  /*!
    * \brief calcCleanImg extract objects based on a refernce picture (RefImage)
    * \param image Image to clean
    * \return clean Binary Image
    */
  cv::Mat calcCleanImg(cv::Mat image);
  /*!
     * \brief getRefImage get RefImage (also the Improved one for debugging)
     * \return
     */
  cv::Mat getRefImage() const;
  /*!
     * \brief findObjects applys closing to a binary image and search for objects larger than MIN_AREA
     * \param imageU binary Image
     * \param objects
     */
  void findObjects(cv::Mat imageU,cv::Mat original,std::vector<cv::Point2d> &objects,std::vector<int> &depth);

  void applyClosing(cv::Mat &image);
  /*!
     * \brief mergeObjectsToEnviroment check if Objects in Environment. If not add them
     * \param objects All Objects which should be in the Environment
     */
  //void mergeObjectsToEnviroment(std::vector<cv::Point2d> &objects);

  oadrive::util::CoordinateConverter *getPosConv() const;
  //! \brief setPosConv Set postionen Converter
  //! \param value
  void setPosConv(oadrive::util::CoordinateConverter *value);
  //! get debug image
  cv::Mat getDebugImage(cv::Mat image);

private:
  cv::Mat mRefImage;
  oadrive::util::CoordinateConverter *mPosConv;
  const double MIN_AREA;

  //! Size in meters above the real ground from where on obstacles are considered obstacles.
  float mReferenceImageThreshold;

  //! View distance in meters. Anything beyond this will be ignored.
  float mMaxViewDepth;

  void generateDepthReferenceImage();

  int mMaxDepthRow;
  int mHorizonRow;

public:
  // use a proper alignment when calling the constructor.
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW



};
}
}

#endif
