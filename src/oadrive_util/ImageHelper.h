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
 * \date    2017
 *
 */
//----------------------------------------------------------------------

#ifndef OADRIVE_UTIL_IMAGEHELPER_H
#define OADRIVE_UTIL_IMAGEHELPER_H

#include <opencv2/core/core.hpp>

namespace oadrive
{
namespace util
{

typedef std::vector<cv::Point2f> PointList;

struct Polygon
{
  PointList vertices;
  cv::Point2f refPoint;

  Polygon(PointList vertices, cv::Point2f refPoint) : vertices(vertices), refPoint(refPoint) {}
};

class ImageHelper
{
public:

  /*!
 * Creates a axis aligned Mat which contains a mask for the rotated rectangle.
 * @param polygon the rotated rectangle for which the mask should be created.
 * @return axis aligned Mat which contains a mask for the rotated rectangle.
 */
  static cv::Mat createMask(const Polygon &polygon, const cv::Rect boundingRect);

  /*!
 * Adds in the area specified by the rotated rectangle the amount to every grid cell.
 * @param mat the mat representing the grid/voting space.
 * @param polygon the rotated rectangle which specifies the area.
 * @param amount the amount which should be added.
 */
  static void addPolygon(cv::Mat &mat, const Polygon polygon, cv::Scalar amount);
  static void addPolygon2(cv::Mat &mat, const Polygon polygon, cv::Scalar amount);

  static bool extractRegionOfInterest(const cv::Mat &votingSpace,
                                      const Polygon &regionOfInterest,
                                      cv::Mat &relevantVotingSpace,
                                      cv::Size2f* areaSize);

  /*!
   * Checks if the rectanlge lies inside the voting space.
   * @param votingSpace which should contain the rectangle
   * @param rectangle  which should lie inside the votingspace
   * @return if he rectanlge lies inside the voting space.
   */
  static bool isValidRectangle(const cv::Mat &image, const cv::RotatedRect &rectangle);

  static bool isValidRectangle(const cv::Mat &image, const cv::Rect &rectangle);

  /*!
   * Checks if the given location is located inside the given grid/image.
   * @param location which is checked
   * @param grid the grid the point should lie inside.
   * @return true if the point is located inside the grid.
   */
  static bool isInside(const cv::Point2i& point, const cv::Mat& image);

  /*!
 * Calculates the offset of the shrinked region of interest related to the original. This is
 * necessary when the roi doesn't lie completely inside the voting space.
 * @param roiBoundingBox th original region on interest
 * @return the offset of the shrinked region of interest related to the original roi.
 */
  static cv::Point2f calculateInvalidRoiOffset(const cv::Rect &roiBoundingBox,
                                               const cv::Mat  &votingSpace);

};

}
}
#endif //OADRIVE_UTIL_IMAGEHELPER_H
