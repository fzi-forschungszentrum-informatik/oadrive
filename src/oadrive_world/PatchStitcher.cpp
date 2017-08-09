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
 * \author  Micha Pfeiffer <ueczz@student.kit.edu>
 * \date    2015-12-24
 *
 */
//----------------------------------------------------------------------

#include "PatchStitcher.h"

#include <iostream>
#include "Environment.h"
#include "worldLogging.h"
using icl_core::logging::endl;
using icl_core::logging::flush;

using namespace oadrive::core;

// define PI/8 since it isn't implemented in C++ by default
#define M_PI_8 0.3926990816987241548078304229099378605246461749218882

namespace oadrive{
namespace world{

PatchStitcher::PatchStitcher()
  : mMaximumAngle( M_PI*0.1 )
  , mMinimumDistBetweenCrossings( 1.7 )
{
}

bool PatchStitcher::addStreetPatch( PatchPtr newPatch )
{
  bool wasMerged = mergeIntoList( newPatch );
  bool generatedNewReferencePatch = false;
  if( !wasMerged )
  {
    const PatchPtrList* street = Environment::getInstance()->getStreet();
    // Make new reference patch:
    if( street->size() == 0 )
    {
      // Start with the car:
      EnvObjectPtr previous = Environment::getInstance()->getCar();
      if( previous->calcDistTo( newPatch->getPose() ) > previous->getHeight()*0.5 + newPatch->getHeight()*0.5 )
      {
        if( previous->calcAngleOffset( &*newPatch ) < mMaximumAngle  )
        {
          //street->push_back( newPatch );
          Environment::getInstance()->addStreetPatch( newPatch );

          // Found a patch, reset angle:
          mMaximumAngle = M_PI*0.1;
          generatedNewReferencePatch = true;
        }
        // If the angle wasn't good, be less restrictive for the next patch:
        // (but stay below a 45° angle)
        mMaximumAngle = std::min( mMaximumAngle + M_PI*0.025, M_PI*0.25 );
      }

    } else {
      if( newPatch->getPatchType() != PARKING )
      {
        PatchPtrList* openPatches = Environment::getInstance()->getOpenPatches();
        if( openPatches->size() == 0 )
        {
          LOGGING_WARNING( worldLogger, "No open patches found - should not happen!" << endl );
        } else {

          float maximumScore = 0;
          PatchPtr bestFit;
          //LOGGING_INFO( worldLogger, "Searching for best fit:" << endl );
          for( PatchPtrList::reverse_iterator it = openPatches->rbegin();
              it != openPatches->rend(); it++ )
          {
            PatchPtr previous = *it;
            float score1 = previous->couldBeMyChild( newPatch );
            float score2 = newPatch->couldBeMyChild( previous );
            //LOGGING_INFO( worldLogger, "\tScores: " << score1 << " " << score2 << endl );
            if( score1 > 0 && score2 > 0 )
            {
              if( score1 + score2 > maximumScore )
              {
                maximumScore = score1 + score2;
                //LOGGING_INFO( worldLogger, "\t\tNew maximum score: " << maximumScore << "!" << endl );
                bestFit = previous;
              }
            }
          }
          if( bestFit )
          {
            generatedNewReferencePatch = true;

            bestFit->addChild( newPatch );
            newPatch->addChild( bestFit );

            //street->push_back( newPatch ):
            // Note: Should be done AFTER calling addChild!!
            Environment::getInstance()->addStreetPatch( newPatch );

            //std::cout << "Previous: has open sides: " << previous->hasOpenSides() << std::endl;

            // If there are no more open sides for the current "previous" patch
            // then delete it from the list of open patches:
            if( !bestFit->hasOpenSides() )
            {
              openPatches->remove( bestFit );
            }
          }
        }
      } else {
        Environment::getInstance()->addStreetPatch( newPatch );
        generatedNewReferencePatch = true;
      }
    }

    // If the patch could not be used as a new reference patch, "remember" it for
    // future sorting:
    //if( !generatedNewReferencePatch )
    //{
    /*mUnsortedPatches.push_back( newPatch );
    // Allow a list of up to 20 unsorted patches. If it gets too large, remove the
    // old patches first (FIFO):
    if( mUnsortedPatches.size() > 20 )
    {
    mUnsortedPatches.erase( mUnsortedPatches.begin() );
    }*/
    if( generatedNewReferencePatch )
    {
      if( newPatch->hasOpenSides() )
      {
        if( newPatch->getPatchType() != PARKING )
          Environment::getInstance()->getOpenPatches()->push_back( newPatch );
      }
      // A new patch was generated. Check if you can sort any of the unsorted list into
      // the new patch:
      /*PatchPtrList* unsorted = Environment::getInstance()->getUnsortedPatches();
      for( PatchPtrList::iterator it = unsorted->begin();
          it != unsorted->end(); )
      {
        if( mergeIntoList( *it ) )
        {
          it = unsorted->erase( it );
        } else {
          it ++;
        }
      }*/
    }
  }
  // If we used this patch, return true, otherwise false
  if( wasMerged || generatedNewReferencePatch )
  {
    return true;
  }

  return false;
}

// Returns true if sorting was "successful", otherwise false:
bool PatchStitcher::mergeIntoList( PatchPtr newPatch )
{
  PatchPtr referencePatch = findReferencePatchFor( newPatch );
  if( referencePatch )
  {
    if( newPatch->getPatchType() == CROSS_SECTION )
    {
      LOGGING_WARNING( worldLogger, "New CROSS_SECTIION."
          << endl );
      LOGGING_WARNING( worldLogger, referencePatch->getPatchType() 
          << endl );
    }
    // TODO: Maybe disallow patch updates if the car has already driven onto the patch.
    if( referencePatch->getPatchType() == newPatch->getPatchType() )
    {
      referencePatch->tryMerge( newPatch );

      if( referencePatch->getPatchType() == PARKING )
      {
        // Remove old event regions:
        EventRegionPtrList eventRegions = referencePatch->getEventRegions();
        if( eventRegions.size() > 0 )
        {
          // Parking patch should always only have one event region. Remove it:
          referencePatch->removeEventRegion( *eventRegions.begin() );
          Environment::getInstance()->removeEventRegion( *eventRegions.begin() );
        }
        // Add new event region:
        Environment::getInstance()->generateEventRegionsForPatch( referencePatch );
      }

      // TODO if after the update the referencePatch's couldBeMyChild no longer returns true
      // on its neighbour patches, it should possibly be removed from the street.
      return true;
    } else if(referencePatch->getPatchType() == STRAIGHT && newPatch->getPatchType()  == CROSS_SECTION) {

      // Check if this new Patch falls onto a CROSS_SECTION.
      if( tooCloseToCrossSection( newPatch, mMinimumDistBetweenCrossings ) )
      {
        // We didn't do anything to the patch, but return "true", meaning the calling
        // function will think the patch was sorted into the list. This makes sure the
        // caller will not use this patch.
        LOGGING_WARNING( worldLogger, "New CROSS_SECTIION overlaps an old one. Ignoring."
            << endl );
        return true;
      }

      // A CROSS_SECTION can replace a STRAIGHT patch.
      Environment::getInstance()->replaceStreetPatch(referencePatch, newPatch);

      if( newPatch->hasOpenSides() )
      {
        Environment::getInstance()->getOpenPatches()->push_back( newPatch );
      }
      // A new patch was generated. Check if you can sort any of the unsorted list into
      // the new patch:
      /*PatchPtrList* unsorted = Environment::getInstance()->getUnsortedPatches();
        for( PatchPtrList::iterator it = unsorted->begin();
        it != unsorted->end(); )
        {
        if( mergeIntoList( *it ) )
        {
        it = unsorted->erase( it );
        } else {
        it ++;
        }
        }*/

      return true;
    }

    // If this is a straight patch or parking lot and it has landed on another type of patch,
    // ignore this new patch:
    if( newPatch->getPatchType() == STRAIGHT || newPatch->getPatchType() == PARKING )
      return true;
  } else {
    if( newPatch->getPatchType() == CROSS_SECTION )
    {
      if( tooCloseToCrossSection( newPatch, mMinimumDistBetweenCrossings ) )
      {
        // We didn't do anything to the patch, but return "true", meaning the calling
        // function will think the patch was sorted into the list. This makes sure the
        // caller will not use this crossing as a new patch.
        LOGGING_WARNING( worldLogger, "New CROSS_SECTIION is too close to an old one. Ignoring."
            << endl );
        return true;
      }
    }
  }
  return false;
}

bool PatchStitcher::overlapsCrossSection( PatchPtr patch )
{
  // If this is a CROSS_SECTION, ignore it if it overlaps any other CROSS_SECTIONS.
  const PatchPtrList* patches = Environment::getInstance()->getStreet();
  for( PatchPtrList::const_reverse_iterator it = patches->rbegin();
      it != patches->rend(); it ++ )
  {
    if( (*it)->getPatchType() == CROSS_SECTION )
    {
      if( (*it)->checkOverlap( patch ) )
      {
        return true;
      }
    }
  }
  return false;
}

bool PatchStitcher::tooCloseToCrossSection( PatchPtr patch, float dist )
{
  // Find out if the center of the given Patch is within distance
  // to a CROSS_SECTION.
  const PatchPtrList* patches = Environment::getInstance()->getStreet();
  for( PatchPtrList::const_reverse_iterator it = patches->rbegin();
      it != patches->rend(); it ++ )
  {
    if( (*it)->getPatchType() == CROSS_SECTION )
    {
      if( (*it)->calcDistTo( patch ) < dist )
      {
        return true;
      }
    }
  }
  return false;
}

PatchPtr PatchStitcher::findReferencePatchFor( PatchPtr patch )
{
  unsigned int numCheckedPatches = 0;
  if( patch->getPatchType() == CROSS_SECTION )
  {
    LOGGING_INFO( worldLogger, "Checking for CROSS_SECTION reference patch" << endl );
  }
  if( patch->getPatchType() != PARKING )
  {
        LOGGING_INFO( worldLogger, "0" );
    const PatchPtrList* patches = Environment::getInstance()->getStreet();
    PatchPtr found;

    double minDist = std::numeric_limits<double>::max();

    for( PatchPtrList::const_reverse_iterator it = patches->rbegin();
        it != patches->rend(); it ++ )
    {
      // If the checked patch is a STRAIGHT patch, angles must match:
      if( patch->getPatchType() == STRAIGHT && (*it)->getPatchType() == STRAIGHT )
      {
        // If the angles aren't similar, skip:
        float yawDiff = Environment::angleModPi(
            (*it)->getPose().getYaw() - patch->getPose().getYaw() );
        if( yawDiff > M_PI_4 && yawDiff < M_PI-M_PI_4 )
        {
          continue;
        }
      }
      // Check if the patches "overlap", i.e. the center of one patch is inside the other patch:
      if( (*it)->isPointInside( patch->getPose() ) || (patch)->isPointInside( (*it)->getPose() ) )
      {
        LOGGING_INFO( worldLogger, "1" );
        if( (*it)->getPatchType() != CROSS_SECTION )
        {
          // If we've already found a CROSS_SECTION which could be a reference patch,
          // ignore other non-CROSS_SECTION patches:
          if( found && found->getPatchType() == CROSS_SECTION )
          {
            continue;
          } else {
            double dist = (*it)->calcDistTo( patch );
            if( dist < minDist )
            {
              found = *it;
              minDist = dist;
            }
          }
        } else {
        LOGGING_INFO( worldLogger, "2" );
          double dist = (*it)->calcDistTo( patch );
          if( found && found->getPatchType() != CROSS_SECTION ) // overwrite non-CROSS_SECTIONs:
          {
        LOGGING_INFO( worldLogger, "3" );
            found = *it;
            minDist = dist;
          } else {
            if( dist < minDist )
            {
        LOGGING_INFO( worldLogger, "4" );
              found = *it;
              minDist = dist;
            }
          }
        }
      }
    }
    if( found )
      return found;

  } else {
    const PatchPtrList* patches = Environment::getInstance()->getParkingLots();
    for( PatchPtrList::const_reverse_iterator it = patches->rbegin();
        it != patches->rend(); it ++ )
    {
      //if( (*it)->getPatchType() == patch->getPatchType() && (*it)->isPointInside( patch->getPose() ) )
      if( (*it)->isPointInside( patch->getPose() ) || (patch)->isPointInside( (*it)->getPose() ) )
      {
        return *it;
      }
    }

  }
  return PatchPtr();		// Return empty pointer
}

}	// namespace
}	// namespace
