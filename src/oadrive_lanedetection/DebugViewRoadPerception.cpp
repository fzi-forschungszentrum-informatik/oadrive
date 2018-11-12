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
 * \author  Fabian Dürr
 * \date    2017-9-26
 * 
 * \author  Simon Roesler <simon.roesler@student.kit.edu>
 * \date    2018
 * 
 * \author  Mark Hueneberg <hueneber@fzi.de>
 * \date    2018
 */
//----------------------------------------------------------------------

#include <chrono>
#include "DebugViewRoadPerception.h"
#include "oadrive_lanedetection/RoadPatching/RoadPatcher.h"
#include "oadrive_lanedetection/RoadPatching/RoadPerception.h"
#include "oadrive_lanedetection/FeatureDetection/HaarFilter.h"
#include "oadrive_lanedetection/RoadPatching/VotingSpace.h"


using namespace oadrive::core;
using namespace oadrive::util;


namespace oadrive
{
namespace lanedetection
{

HaarFilter* DebugViewRoadPerception::haarFilterToDebug;


#ifdef DEBUG_ACTIVE
cv::Mat DebugViewRoadPerception::debugImage;
float DebugViewRoadPerception::debugScale;

const VotingSpace* DebugViewRoadPerception::mVotingSpaceToDebug;
const RoadPatcher* DebugViewRoadPerception::roadPatcherToDebug;
RoadPerception* DebugViewRoadPerception::roadPerceptionToDebug;


cv::Size2i DebugViewRoadPerception::dbgBirdviewSize;
unsigned short DebugViewRoadPerception::dbgRowsVS;
unsigned short DebugViewRoadPerception::dbgColsVS;
CoordinateConverter* DebugViewRoadPerception::dbgConverter;
float DebugViewRoadPerception::dbgScaleVS;

cv::Size2i DebugViewRoadPerception::dbgOffsetBirdviewToVS;
cv::Size2f DebugViewRoadPerception::dbgPatchSize;


cv::Mat*
DebugViewRoadPerception::generateHoughSpaceDebugImage(const VotingSpace* votingSpaceToDebug,
                                                      const RoadPatcher* patcher)
{
  int currentAngleRegion = votingSpaceToDebug->dbgCurrentAngleRegion;
  setVotingSpace(votingSpaceToDebug);
  setRoadPatcher(patcher);

  cv::Mat dbgHoughSpace(dbgRowsVS * 2 + 5, dbgColsVS * 3 + 10, CV_8UC1, cv::Scalar(0));

  visualizeAngleRegionVotes(dbgHoughSpace, currentAngleRegion, cv::Size2f(dbgColsVS + 5, 0));
  cv::imshow("test", dbgHoughSpace);
  //cv::waitKey(1);

  int nextAngleRegion = votingSpaceToDebug->getNeighbourAngleRegion(currentAngleRegion, -1);
  visualizeAngleRegionVotes(dbgHoughSpace, nextAngleRegion, cv::Size2f(0, 0));

  nextAngleRegion = votingSpaceToDebug->getNeighbourAngleRegion(currentAngleRegion, -2);
  visualizeAngleRegionVotes(dbgHoughSpace, nextAngleRegion, cv::Size2f(0, dbgRowsVS + 5));


  nextAngleRegion = votingSpaceToDebug->getNeighbourAngleRegion(currentAngleRegion, +1);
  visualizeAngleRegionVotes(dbgHoughSpace, nextAngleRegion, cv::Size2f(2 * (dbgColsVS + 5), 0));


  nextAngleRegion = votingSpaceToDebug->getNeighbourAngleRegion(currentAngleRegion, +2);
  visualizeAngleRegionVotes(dbgHoughSpace, nextAngleRegion,
                            cv::Size2f(2 * (dbgColsVS + 5), dbgRowsVS + 5));

  debugImage = cv::Mat(debugScale * (dbgRowsVS * 2 + 5), debugScale * (dbgColsVS * 3 + 10),
                       CV_8UC1);

  cv::resize(dbgHoughSpace, debugImage, cv::Size(), debugScale, debugScale, cv::INTER_NEAREST);
  cv::cvtColor(debugImage, debugImage, CV_GRAY2BGR);

  // ----------------------------------------------------------

  visualizeAngleRegion(debugImage, currentAngleRegion, cv::Size2f(debugScale * (dbgColsVS + 5), 0));
  nextAngleRegion = votingSpaceToDebug->getNeighbourAngleRegion(currentAngleRegion, -1);

  visualizeAngleRegion(debugImage, nextAngleRegion, cv::Size2f(0, 0));
  nextAngleRegion = votingSpaceToDebug->getNeighbourAngleRegion(currentAngleRegion, -2);

  visualizeAngleRegion(debugImage, nextAngleRegion, cv::Size2f(0, debugScale * (dbgRowsVS + 5)));

  nextAngleRegion = votingSpaceToDebug->getNeighbourAngleRegion(currentAngleRegion, +1);
  visualizeAngleRegion(debugImage, nextAngleRegion, cv::Size2f(debugScale * 2 * (dbgColsVS + 5), 0));

  nextAngleRegion = votingSpaceToDebug->getNeighbourAngleRegion(currentAngleRegion, +2);
  visualizeAngleRegion(debugImage, nextAngleRegion,
                       cv::Size2f(debugScale * 2 * (dbgColsVS + 5), debugScale * (dbgRowsVS + 5)));

  visualizeInBirdview(debugImage);

  cv::resize(debugImage, debugImage, cv::Size(), 0.4, 0.4, cv::INTER_CUBIC);

  resetOtherClasses();

  return &debugImage;
}

void
DebugViewRoadPerception::visualizeAngleRegionVotes(cv::Mat &dbgImage, int angleRegion,
                                                  cv::Size2f offset)
{
  // ---------------------- draw votes ---------------------------------
  cv::Mat tmp;
  mVotingSpaceToDebug->dbgRegionsToDebug.front()->regions.at(angleRegion).copyTo(tmp);

  for (int i = 1; i < mVotingSpaceToDebug->dbgRegionsToDebug.size(); i++)
  {
    cv::add(mVotingSpaceToDebug->dbgRegionsToDebug.at(i)->regions.at(angleRegion), tmp, tmp);
  }

  cv::blur(tmp, tmp, cv::Size(mVotingSpaceToDebug->mBlurSize, mVotingSpaceToDebug->mBlurSize));
  tmp = tmp * (1.0 / 100 * 255);
  tmp.copyTo(dbgImage(cv::Rect(offset.width, offset.height, tmp.cols, tmp.rows)));

  // create borders and highlight the region where the max was found
  cv::Rect border(offset.width, offset.height, tmp.cols, tmp.rows);
  if (angleRegion == mVotingSpaceToDebug->dbgMaxAngleRegion)
  {
    rectangle(dbgImage, border, cv::Scalar(255));
  }
  else
  {
    rectangle(dbgImage, border, cv::Scalar(128));
  }

  // create smaller borders to show the area which is covered by the birdview
  int birdviewWith = dbgBirdviewSize.width * dbgScaleVS;
  int birdviewHeight = dbgBirdviewSize.height * dbgScaleVS;

  int diffX = dbgColsVS - birdviewWith;
  int diffY = dbgRowsVS - birdviewHeight;

  cv::Rect birdview(offset.width + diffX * 0.5, offset.height + diffY * 0.5,
                    birdviewWith, birdviewHeight);
  rectangle(dbgImage, birdview, cv::Scalar(75));
}


void
DebugViewRoadPerception::visualizeAngleRegion(cv::Mat &dbgImage, int angleRegion,
                                             cv::Size2f offset)
{

  // color the maximum
  cv::Point2i maxPos(mVotingSpaceToDebug->dbgPosOfMax.x * debugScale + offset.width,
                     mVotingSpaceToDebug->dbgPosOfMax.y * debugScale + offset.height);

  if (angleRegion == mVotingSpaceToDebug->dbgMaxAngleRegion)
  {
    cv::Rect marker(maxPos.x - 2, maxPos.y - 2, 4, 4);
    cv::rectangle(dbgImage, marker, COL_MAX_LOCATION, 2);
  }


  // **************************** draw ROI ******************************
  if (mVotingSpaceToDebug->dbgHasROI)
  {
    cv::Size2f roiSize(debugScale * mVotingSpaceToDebug->dbgCurrentRegion.width,
                       debugScale * mVotingSpaceToDebug->dbgCurrentRegion.length);

    cv::Point2f roiCenterPixel = dbgConverter->car2VotingSpace(mVotingSpaceToDebug->mCenterPose,
                                                               mVotingSpaceToDebug->mCenterPixel,
                                                               mVotingSpaceToDebug->dbgCurrentRegion.center,
                                                               dbgScaleVS) * debugScale;


    roiCenterPixel.x += offset.width;
    roiCenterPixel.y += offset.height;

    cv::RotatedRect roi(roiCenterPixel, roiSize,
                        -mVotingSpaceToDebug->dbgCurrentRegion.center.getYaw() * 180.0 / M_PI);
    drawPolygon(dbgImage, dbgConverter->scaleRectangle(roi, dbgScaleVS), COL_ROI, 2);
  }

  // **************************** draw new patch ******************************
  if (roadPatcherToDebug->dbgFoundNewPatch)
  {
    cv::Point2i patchPos = dbgConverter->car2Pixel(roadPatcherToDebug->dbgNewPatch.getPose());

    patchPos.x += offset.width + dbgOffsetBirdviewToVS.width;
    patchPos.y += offset.height + dbgOffsetBirdviewToVS.height;

    cv::RotatedRect newPatch(patchPos,
                             roadPatcherToDebug->dbgNewPatch.getPatchSize() * debugScale,
                             -roadPatcherToDebug->dbgNewPatch.getYaw() * 180.0 / M_PI);

    cv::Scalar col;
    if (roadPatcherToDebug->dbgNewPatch.getBasedOn() == PatchHypothesis::BasedOn::PREDICTION)
    {
      col = COL_NEW_PATCH_PRED;
    }
    else if (roadPatcherToDebug->dbgNewPatch.getBasedOn() ==
             PatchHypothesis::BasedOn::PREVIOUS_HYP)
    {
      col = COL_NEW_PATCH_PREV;
    }
    else
    {
      col = COL_NEW_PATCH;
    }
    if (mVotingSpaceToDebug->dbgMaxAngleRegion == angleRegion)
    {
      drawPolygon(dbgImage, dbgConverter->scaleRectangle(newPatch, dbgScaleVS), col,
                  2);
    }
  }
  // **************************** draw ref patch ******************************
  cv::Point2f refPatchPixelPos = mVotingSpaceToDebug->mCoordConverter->car2Pixel(
          roadPatcherToDebug->dbgCurrentRefPatch.getPose());

  refPatchPixelPos.x += offset.width + dbgOffsetBirdviewToVS.width;
  refPatchPixelPos.y += offset.height + dbgOffsetBirdviewToVS.height;

  cv::RotatedRect refPatch(refPatchPixelPos,
                           roadPatcherToDebug->dbgCurrentRefPatch.getPatchSize() * debugScale,
                           -roadPatcherToDebug->dbgCurrentRefPatch.getYaw() * 180.0
                           / M_PI);

  drawPolygon(dbgImage, dbgConverter->scaleRectangle(refPatch, dbgScaleVS), COL_REF_PATCH,
              2);

  // ---------------------- Draw current region index ---------------------------------
  cv::Point2f pos((offset.width + 3),
                  (offset.height + debugScale * dbgRowsVS - 5));
  cv::putText(dbgImage, std::to_string(angleRegion), pos, cv::FONT_HERSHEY_SIMPLEX,
              2.0, cv::Scalar(255, 255, 255), 1.5);
}

void
DebugViewRoadPerception::drawPolygon(cv::Mat &image, Polygon polygon, cv::Scalar color,
                                     int thickness)
{
  for (int i = 0; i < polygon.vertices.size(); i++)
  {
    cv::line(image, polygon.vertices.at(i), polygon.vertices.at((i + 1) % polygon.vertices.size()),
             color, thickness,
             cv::LINE_AA);
  }
}

void DebugViewRoadPerception::visualizeInBirdview(cv::Mat &dbgImage)
{
  int offsetX = debugScale * (dbgColsVS + 5);
  int offsetY = debugScale * (dbgRowsVS + 5);

  // draw birdview to debug image
  cv::Mat tmp;
  cv::cvtColor(roadPerceptionToDebug->mCurrentImage, tmp, CV_GRAY2BGR);

  cv::Mat channels[4];
  cv::split(roadPerceptionToDebug->dbgFeatureImage, channels);
  cv::Mat sum;
  cv::bitwise_or(tmp, 0, sum, 255 - channels[3]);
  cv::Mat featuresRGB;
  cv::cvtColor(roadPerceptionToDebug->dbgFeatureImage, featuresRGB, CV_BGRA2BGR);
  tmp = sum + featuresRGB;

  tmp.copyTo(dbgImage(cv::Rect(offsetX + dbgOffsetBirdviewToVS.width,
                               offsetY + dbgOffsetBirdviewToVS.height,
                               tmp.cols, tmp.rows)));

  // ---------------------- Draw new patch ---------------------------------
  if (roadPatcherToDebug->dbgFoundNewPatch)
  {
    cv::Point2i patchPos = roadPatcherToDebug->mCoordConverter->car2Pixel
            (roadPatcherToDebug->dbgNewPatch.getPose());

    patchPos.x += offsetX + dbgOffsetBirdviewToVS.width;
    patchPos.y += offsetY + dbgOffsetBirdviewToVS.height;

    cv::RotatedRect newPatch(patchPos,
                             roadPatcherToDebug->dbgNewPatch.getPatchSize() * debugScale,
                             -roadPatcherToDebug->dbgNewPatch.getYaw() * 180.0
                             / M_PI);

    cv::Scalar col;
    if (roadPatcherToDebug->dbgNewPatch.getBasedOn() == PatchHypothesis::BasedOn::PREVIOUS_HYP)
    {
      drawPolygon(dbgImage, dbgConverter->scaleRectangle(newPatch, dbgScaleVS), COL_NEW_PATCH_PREV,
                  2);
      cv::circle(dbgImage, patchPos, 2, COL_NEW_PATCH_PREV);
      cv::putText(dbgImage, std::to_string(roadPatcherToDebug->dbgNewPatch.getPatchID()),
                  patchPos, cv::FONT_HERSHEY_SIMPLEX, 1.0, COL_NEW_PATCH_PREV);
    }
    else if (roadPatcherToDebug->dbgNewPatch.getBasedOn() == PatchHypothesis::BasedOn::FEATURE)
    {
      drawPolygon(dbgImage, dbgConverter->scaleRectangle(newPatch, dbgScaleVS), COL_NEW_PATCH,
                  2);
      cv::circle(dbgImage, patchPos, 2, COL_NEW_PATCH);
      cv::putText(dbgImage, std::to_string(roadPatcherToDebug->dbgNewPatch.getPatchID()),
                  patchPos, cv::FONT_HERSHEY_SIMPLEX, 1.0, COL_NEW_PATCH);
    }
    else
    {
      drawPolygon(dbgImage, dbgConverter->scaleRectangle(newPatch, dbgScaleVS), COL_NEW_PATCH_PRED,
                  2);
      cv::circle(dbgImage, patchPos, 2, COL_NEW_PATCH_PRED);
      cv::putText(dbgImage, std::to_string(roadPatcherToDebug->dbgNewPatch.getPatchID()),
                  patchPos,
                  cv::FONT_HERSHEY_SIMPLEX, 1.0, COL_NEW_PATCH_PRED);

    }
  }

  // ---------------------- Draw ref patch ---------------------------------
  cv::Point2i patchPos = roadPatcherToDebug->mCoordConverter->car2Pixel
          (roadPatcherToDebug->dbgCurrentRefPatch.getPose());

  patchPos.x += offsetX + dbgOffsetBirdviewToVS.width;
  patchPos.y += offsetY + dbgOffsetBirdviewToVS.height;

  cv::RotatedRect refPatch(patchPos,
                           roadPatcherToDebug->dbgCurrentRefPatch.getPatchSize() * debugScale,
                           -roadPatcherToDebug->dbgCurrentRefPatch.getYaw() * 180.0
                           / M_PI);

  drawPolygon(dbgImage, dbgConverter->scaleRectangle(refPatch, dbgScaleVS), COL_REF_PATCH,
              2);
  cv::circle(dbgImage, patchPos, 3, COL_REF_PATCH);
  cv::putText(dbgImage, std::to_string(roadPatcherToDebug->dbgCurrentRefPatch.getPatchID()),
              patchPos, cv::FONT_HERSHEY_SIMPLEX, 1.0, COL_REF_PATCH);

  // ---------------------- Draw current max ---------------------------------
  cv::Point2f posMax((offsetX + debugScale * (dbgColsVS * 0.5 + 5)),
                     (offsetY + 50));
//  cv::putText(dbgImage, "Max: " + std::to_string(votingSpaceToDebug->dbgValueOfMax), posMax,
//              cv::FONT_HERSHEY_SIMPLEX,
//              2.0, cv::Scalar(255, 255, 255), 2);

  cv::Point2f posIt((offsetX + debugScale * (dbgColsVS * 0.5 - 100)),
                    (offsetY + 50));
  cv::putText(dbgImage, "Iteration: " + std::to_string(roadPatcherToDebug->dbgCurrentIt), posIt,
              cv::FONT_HERSHEY_SIMPLEX,
              2.0, cv::Scalar(255, 255, 255), 2);

}

void DebugViewRoadPerception::setVotingSpace(const VotingSpace* votingSpaceToDebug)
{
  mVotingSpaceToDebug = votingSpaceToDebug;

  dbgBirdviewSize = votingSpaceToDebug->mSize;
  dbgRowsVS = votingSpaceToDebug->mRows;
  dbgColsVS = votingSpaceToDebug->mCols;
  dbgConverter = votingSpaceToDebug->mCoordConverter;
  dbgScaleVS = votingSpaceToDebug->mScale;

  debugScale = 1.0 / dbgScaleVS;

  int offsetX = dbgColsVS - dbgBirdviewSize.width * dbgScaleVS;
  int offsetY = dbgRowsVS - dbgBirdviewSize.height * dbgScaleVS;
  dbgOffsetBirdviewToVS = cv::Size2i(debugScale * offsetX * 0.5, debugScale * offsetY * 0.5);

  dbgPatchSize = cv::Size2f(PATCH_WIDTH_STRAIGHT * debugScale, PATCH_LENGTH_STRAIGHT * debugScale);
}

#endif

cv::Mat DebugViewRoadPerception::generateFeatureDebugImage(const FeaturePointerVector features)
{
  cv::Mat output(haarFilterToDebug->mInputSize, CV_8UC4, cv::Scalar(0, 0, 0, 0));
  std::ostringstream oss;
  for (size_t i = 0; i < features.size(); i++)
  {
    const Feature* f = features[i];
    cv::Point2f pos = haarFilterToDebug->mCoordConverter->car2Pixel(f->localPose);
    if (pos.x > 0 && pos.x < haarFilterToDebug->mInputSize.width && pos.y > 0 &&
        pos.y < haarFilterToDebug->mInputSize.height)
    {
      cv::Point2f dist = cv::Point2f(-18.0 * sin(f->localPose.getYaw()),
                                     -18.0 * cos(f->localPose.getYaw()));
      oss.clear();
      oss.str("");
      oss << f->probability;
      // cv::putText(output, oss.str(), pos, cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 0, 255, 0));

      if (f->type == CENTER_LINE)
      {
        cv::Scalar color = cv::Scalar(0, 0, 255, 255);
        // oss.clear();
        // oss.str("");
        // oss << "CENTER";
        // cv::putText(output, oss.str(), cv::Point2f(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.6, color);

        cv::line(output, pos, pos + dist, color, 3);
        cv::circle(output, pos, 3, color);
      }
      else if (f->type == LEFT_SIDE_LINE)
      {
        cv::Scalar color = cv::Scalar(255, 0, 0, 255);
        // oss.clear();
        // oss.str("");
        // oss << "LEFT";
        // cv::putText(output, oss.str(), cv::Point2f(10, 40), cv::FONT_HERSHEY_SIMPLEX, 0.6,
        //             color);

        cv::line(output, pos, pos + dist, color, 3);
        cv::circle(output, pos, 3, color);
      }
      else if (f->type == RIGHT_SIDE_LINE)
      {
        cv::Scalar color = cv::Scalar(0, 255, 0, 255);
        // oss.clear();
        // oss.str("");
        // oss << "RIGHT";
        // cv::putText(output, oss.str(), cv::Point2f(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.6, color);

        cv::line(output, pos, pos + dist, color, 3);
        cv::circle(output, pos, 3, color);
      }
      else if (f->type == STREET_LINE)
      {
        cv::Scalar color = cv::Scalar(199, 21, 133, 255);
        // oss.clear();
        // oss.str("");
        // oss << "STREET_LINE";
        // cv::putText(output, oss.str(), cv::Point2f(10, 80), cv::FONT_HERSHEY_SIMPLEX, 0.6, color);

        cv::line(output, pos, pos + dist, color, 3);
        cv::circle(output, pos, 3, color);
      }
      else if (f->type == CROSSROAD_LINE)
      {
        cv::Scalar color = cv::Scalar(225, 225, 0, 255);
        // oss.clear();
        // oss.str("");
        // oss << "CROSSROAD_LINE";
        // cv::putText(output, oss.str(), cv::Point2f(10, 80), cv::FONT_HERSHEY_SIMPLEX, 0.6, color);

        cv::line(output, pos, pos + dist, color, 3);
        cv::circle(output, pos, 3, color);
      }
    }
  }


  cv::Vec4b colorLeft(183, 22, 247, 255);
  cv::Vec4b colorRight(108, 10, 147, 255);

  // draw calculated bird view cones
  for (int i = 0; i < haarFilterToDebug->mInputSize.height; i++)
  {
    int xConeLeft = ((float) i - haarFilterToDebug->bLeft) / haarFilterToDebug->mLeft;
    if (xConeLeft >= 0 && xConeLeft < haarFilterToDebug->mInputSize.width)
    {
      output.at<cv::Vec4b>(i, xConeLeft) = colorLeft;
    }
    int xConeRight = ((float) i - haarFilterToDebug->bRight) / haarFilterToDebug->mRight;
    if (xConeRight >= 0 && xConeRight < haarFilterToDebug->mInputSize.width)
    {
      output.at<cv::Vec4b>(i, xConeRight) = colorRight;
    }
  }

  return output;
}

void DebugViewRoadPerception::setHaarFilter(HaarFilter* hf)
{
  haarFilterToDebug = hf;
}

#ifdef DEBUG_ACTIVE

void DebugViewRoadPerception::setRoadPerception(RoadPerception* rp)
{
  roadPerceptionToDebug = rp;
}

void DebugViewRoadPerception::setRoadPatcher(const RoadPatcher* sp)
{
  roadPatcherToDebug = sp;
}

void DebugViewRoadPerception::resetOtherClasses()
{
  //streetPatcherToDebug->dbgCurrentIt = -1;
}

void DebugViewRoadPerception::visualizePatches(PatchHypothesisList &patchHyps) {
  cv::Mat dbgImage;
  
  cv::cvtColor(roadPerceptionToDebug->mCurrentImage, dbgImage, CV_GRAY2BGR);

  std::cout << patchHyps.size() << std::endl;

  PatchHypothesisList::const_reverse_iterator it;
  for (it = patchHyps.rbegin(); it != patchHyps.rend(); it++)
  {
    cv::Point2f center = roadPatcherToDebug->mCoordConverter->car2Pixel((*it).getPose());
    cv::Size2f size = (*it).getPatchSize();
    double angle = (*it).getYaw();
    cv::RotatedRect rRect(center, size, -angle * 180.0 / M_PI);

    cv::Point2f dist = cv::Point2f(-50.0 * sin(angle), -50.0 * cos(angle));

    util::PointList vertices = roadPatcherToDebug->mCoordConverter->scaleRectangle(rRect, 1.0).vertices;

    cv::Scalar col(0, 0, 255);
    for (uint i = 0; i < vertices.size(); i++)
    {
      cv::line(dbgImage, vertices[i], vertices[(i + 1) % vertices.size()], col, 1);
    }
    cv::line(dbgImage, center, center + dist, col, 1);
    cv::circle(dbgImage, center, 4, col, 2);

    cv::putText(dbgImage, std::to_string((*it).getPatchID()), center, cv::FONT_HERSHEY_SIMPLEX,
                0.7, cv::Scalar(255));

  }

  cv::imshow("crosspatches", dbgImage);
  
}

#endif

}
}
