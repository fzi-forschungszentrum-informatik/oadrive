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

#include <opencv2/imgproc.hpp>
#include "ImageHelper.h"

namespace oadrive
{
namespace util
{

cv::Mat ImageHelper::createMask(const util::Polygon &polygon, const cv::Rect boundingRect)
{
  // Create a mask to set all pixels in the bounding box outside the actual roi to zero
  cv::Mat mask(boundingRect.height, boundingRect.width, CV_16UC1, cv::Scalar(0));

  cv::Point verticesInRoi[4];
  cv::Point2f center(boundingRect.size().width * 0.5, boundingRect.size().height * 0.5);
  for (int i = 0; i < 4; i++)
  {
    int x = (polygon.vertices[i].x - polygon.refPoint.x) + center.x;
    int y = (polygon.vertices[i].y - polygon.refPoint.y) + center.y;

    verticesInRoi[i] = cv::Point2i(x, y);

    // If point is not valid return an empty mask
    if (!isInside(verticesInRoi[i], mask))
    {
      return mask;
    }
  }

  // Fill the mask
  cv::fillConvexPoly(mask, &verticesInRoi[0], 4, cv::Scalar(1.0));
  return mask;
}

void ImageHelper::addPolygon(cv::Mat &image, const util::Polygon polygon, cv::Scalar amount)
{
  // Make sure we're inside the image:
  cv::Rect boundings = cv::boundingRect(polygon.vertices);
  cv::Rect intersection = boundings;
  if (!isValidRectangle(image, boundings))
  {
    intersection = cv::Rect(0, 0, image.cols, image.rows) & boundings;

    if (intersection.height <= 0 || intersection.width <= 0)
    {
      return;
    }

    cv::Point2f offset = ImageHelper::calculateInvalidRoiOffset(boundings, image);
    offset.x = std::floor(offset.x);
    offset.y = std::floor(offset.y);

    cv::Point verticesInRoi[4];
    cv::Point2f center(boundings.size().width * 0.5, boundings.size().height * 0.5);
    for (uint i = 0; i < 4; i++)
    {
      verticesInRoi[i] = (polygon.vertices[i] - polygon.refPoint) + center;
    }

    cv::Mat convexPolygon(boundings.size(), CV_16UC1, cv::Scalar(0));
    cv::fillConvexPoly(convexPolygon, &verticesInRoi[0], 4, amount);

    cv::Rect adapted(offset, intersection.size());
    convexPolygon = convexPolygon(adapted);

    cv::Mat roi = image(intersection);
    cv::add(roi, convexPolygon, roi);
    return;
  }

  cv::Mat roi = image(boundings);
  cv::Point verticesInRoi[4];
  cv::Point2f center(roi.size().width * 0.5, roi.size().height * 0.5);
  for (uint i = 0; i < 4; i++)
  {
    verticesInRoi[i] = (polygon.vertices[i] - polygon.refPoint) + center;
  }

  cv::Mat convexPolygon(boundings.size(), CV_16UC1, cv::Scalar(0));
  cv::fillConvexPoly(convexPolygon, &verticesInRoi[0], 4, amount);

  cv::add(roi, convexPolygon, roi);
}

void ImageHelper::addPolygon2(cv::Mat &image, const util::Polygon polygon, cv::Scalar amount)
{
  // Make sure we're inside the image:
  cv::Rect boundings = cv::boundingRect(polygon.vertices);
  if (!isValidRectangle(image, boundings))
  {
    return;
  }

  cv::Mat roi = image(boundings);

  std::vector<cv::Point> verticesInRoi;
  cv::Point2f center(roi.size().width * 0.5, roi.size().height * 0.5);
  for (uint i = 0; i < polygon.vertices.size(); i++)
  {
    verticesInRoi.push_back((polygon.vertices[i] - polygon.refPoint) + center);
  }

  cv::Point2f offset(0.f, 0.f);
  for (const cv::Point& point : verticesInRoi)
  {
    for (int i = 0; i < verticesInRoi.size(); i++)
    {
      if (std::abs(verticesInRoi.at(i).x) > std::abs(offset.x))
      {
        offset.x = verticesInRoi.at(i).x;
      }

      if (std::abs(verticesInRoi.at(i).y) > std::abs(offset.y))
      {
        offset.y =verticesInRoi.at(i).y;
      }
    }
  }

  for (uint i = 0; i < verticesInRoi.size(); i++)
  {
    if (offset.x < 0 || offset.x >= boundings.width)
    {
      verticesInRoi.at(i).x -= offset.x;
    }
    if (offset.y < 0 || offset.y >= boundings.height)
    {
      verticesInRoi.at(i).y -= offset.y;
    }
  }


  cv::Mat convexPolygon(boundings.size(), CV_16UC1, cv::Scalar(0));
  cv::fillConvexPoly(convexPolygon, &verticesInRoi[0], verticesInRoi.size(), amount);

  cv::add(roi, convexPolygon, roi);
}


bool ImageHelper::extractRegionOfInterest(const cv::Mat &votingSpace,
                                          const Polygon &regionOfInterest,
                                          cv::Mat &relevantVotingSpace,
                                          cv::Size2f* areaSize)
{
  cv::Rect boundingBox = cv::boundingRect(regionOfInterest.vertices);
  *areaSize = cv::minAreaRect(regionOfInterest.vertices).size;
  cv::Mat mask = createMask(regionOfInterest, boundingBox);

  if (isValidRectangle(votingSpace, boundingBox))
  {
    cv::Mat selectedRegion = votingSpace(boundingBox);

    relevantVotingSpace = cv::Mat(selectedRegion.rows, selectedRegion.cols, CV_16UC1);
    for (int i = 0; i < selectedRegion.rows; i++)
    {
      for (int j = 0; j < selectedRegion.cols; j++)
      {
        relevantVotingSpace.at<ushort>(i, j) =
                selectedRegion.at<char>(i, j) * mask.at<ushort>(i, j);
      }
    }

    return true;
  }

  return false;
}

bool
ImageHelper::isValidRectangle(const cv::Mat &votingSpace, const cv::RotatedRect &rectangle)
{
  cv::Point2f vertices[4];
  rectangle.points(vertices);
  for (uint i = 0; i < 4; i++)
  {
    if (!isInside(vertices[i], votingSpace))
    {
      return false;
    }
  }
  return true;
}

bool ImageHelper::isValidRectangle(const cv::Mat &votingSpace, const cv::Rect &rectangle)
{
  return ((rectangle & cv::Rect(0, 0, votingSpace.cols, votingSpace.rows)) == rectangle);
}

bool ImageHelper::isInside(const cv::Point2i& location, const cv::Mat& grid)
{
  if (location.x < 0 || location.x >= grid.cols || location.y < 0 || location.y >= grid.rows)
  {
    return false;
  }

  return true;
}


cv::Point2f ImageHelper::calculateInvalidRoiOffset(const cv::Rect &roiBoundingBox,
                                                   const cv::Mat &votingSpace)
{
  cv::Rect intersection = cv::Rect(0, 0, votingSpace.cols, votingSpace.rows)
                          & roiBoundingBox;

  cv::Point2f offset(0.f, 0.f);
  cv::Point2f verticesOrig[4], verticesIntersection[4], offsets[4];

  cv::RotatedRect test(cv::Point2i(intersection.x + intersection.width * 0.5,
                                   intersection.y + intersection.height * 0.5),
                       intersection.size(), 0.f);
  test.points(verticesIntersection);

  cv::RotatedRect test2(cv::Point2i(roiBoundingBox.x + roiBoundingBox.width * 0.5,
                                    roiBoundingBox.y + roiBoundingBox.height * 0.5),
                        roiBoundingBox.size(), 0.f);
  test2.points(verticesOrig);

  for (int i = 0; i < 4; i++)
  {
    offsets[i] = verticesIntersection[i] - verticesOrig[i];

    if (offsets[i].x > offset.x)
    {
      offset.x = offsets[i].x;
    }

    if (offsets[i].y > offset.y)
    {
      offset.y = offsets[i].y;
    }
  }

  return offset;
}

}
}