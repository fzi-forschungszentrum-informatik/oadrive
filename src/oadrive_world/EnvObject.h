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
 * \author  David Zimmerer <dzimmerer@gmail.com>
 * \author  Peter Zimmer <peter-zimmer@gmx.net>
 * \author  Micha Pfeiffer <ueczz@student.kit.edu>
 * \date    2015-11-24
 * 
 * \author  Simon Roesler <simon.roesler@student.kit.edu>
 * \date    2018
 *
 */
//----------------------------------------------------------------------


#ifndef OADRIVE_WORLD_ENVOBJECT_H
#define OADRIVE_WORLD_ENVOBJECT_H


#include <ctime>
#include <boost/shared_ptr.hpp>
#include <list>
//#include  <tgmath.h>

#include <oadrive_core/ExtendedPose2d.h>

namespace oadrive
{
namespace world
{

enum EnvObjType
{
  PATCH = 1, OBSTACLE = 2, EVENT = 3
};

class EventRegion;  // predefine so shared_ptr can use it
typedef boost::shared_ptr<EventRegion> EventRegionPtr;    // define before class because it uses the PatchPtr
typedef std::list<EventRegionPtr> EventRegionPtrList;

class EnvObject;  // predefine so shared_ptr can use it
typedef boost::shared_ptr<EnvObject> EnvObjectPtr;    // define before class because it uses the PatchPtr
typedef std::list<boost::shared_ptr<EnvObject> > EnvObjectPtrList;

class EnvObject
{

public:
  EnvObject(core::ExtendedPose2d mPose, double mWidth = 0, double length = 0);

  virtual ~EnvObject();
  
  void mergeFrom(const EnvObjectPtr obj);

  uint64_t getId() const;

  void setId(uint64_t id);

  core::ExtendedPose2d getPose() const;

  void setPose(const core::ExtendedPose2d &mPose);

  core::ExtendedPose2d getVelocity() const;

  void setVelocity(const core::ExtendedPose2d &velocity);

  double getProbability() const;

  void setProbability(double probability);

  double getLikelyhood() const;

  void setLikelyhood(double likelyhood);

  int getType() const;

  void setType(int mType);

  double getX() const;

  void setX(double x);

  double getY() const;

  void setY(double y);

  double getYaw() const;

  void setYaw(double yaw);

  double getLength() const;

  void setHeight(double mHeight);

  double getWidth() const;

  void setWidth(double mWidth);

  double getTime() const;

  void setTime(double mTime);

  void updateTime();

  double calcDistTo(EnvObjectPtr other) const;

  double calcDistTo(const core::ExtendedPose2d &mPose) const;

  double calcDistTo(const core::Position2d &pos) const;

  double getTempDist();

  void calcMinDistanceToRefPoints(const core::ExtendedPose2d &pose);

  bool contains(double x, double y);

  double calcAngleOffset(EnvObject* otherObject);

  double calcAngleOffset(const core::ExtendedPose2d &mPose);

  bool isPointInside(const core::ExtendedPose2d &pose) const;
  //bool isPointInside( Eigen::Vector2d M );
  /*! Calculates the Corner Points of the patch and some scalar products..
   * These are used to find if a point is inside the rectangle or not.
   * \note MUST be called whenever the position or orientation of the patch changes! */
  void updateCorners();

  unsigned int
  checkIntersections(const core::Position2d &start, const core::Position2d &end);

  /*! Convert local coordinates pos to global coordinates. */
  oadrive::core::ExtendedPose2d toWorldCoords(const core::ExtendedPose2d &pos);

  oadrive::core::Position2d getCornerA();

  oadrive::core::Position2d getCornerB();

  oadrive::core::Position2d getCornerC();

  oadrive::core::Position2d getCornerD();

  bool checkOverlap(EnvObjectPtr other);

  int getTimeSinceLastUpdate();

  void addEventRegion(EventRegionPtr evRegion);

  void clearEventRegions();

  EventRegionPtrList getEventRegions() { return mEventRegions; }

  void removeEventRegion(EventRegionPtr evRegion);

  /*! Convenience function for objects which only have one event region.
   * Returns first event region. */
  EventRegionPtr getEventRegion();

protected:
  int mType;

  EventRegionPtrList mEventRegions;

private:

  unsigned int mId;

  core::ExtendedPose2d mPose;
  core::ExtendedPose2d mVelocity;
  double mProbability;
  double mLikelyhood;
  double mTime;

  double mWidth, mlength;
  double mTempDist;

  core::Position2d mCornerA;
  core::Position2d mCornerB;
  core::Position2d mCornerC;
  core::Position2d mCornerD;

  core::Position2d mAB;
  core::Position2d mAD;
  float mAB_AB;
  float mAD_AD;

  double crossProduct(const core::Position2d &p1, const core::Position2d &p2);

  bool lineSegmentIntersection(const core::Position2d &p,
                               const core::Position2d &q,
                               const core::Position2d &r,
                               const core::Position2d &s);

public:
  // use a proper alignment when calling the constructor.
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

}  // namespace
}  // namespace

#endif /* OADRIVE_WORLD_ENVOBJECT_H */
