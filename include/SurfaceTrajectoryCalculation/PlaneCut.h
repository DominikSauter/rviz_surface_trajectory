//
//    Copyright (c) 2014 Dominik Sauter
//
//    Permission is hereby granted, free of charge, to any person obtaining a copy
//    of this software and associated documentation files (the "Software"), to deal
//    in the Software without restriction, including without limitation the rights
//    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//    copies of the Software, and to permit persons to whom the Software is
//    furnished to do so, subject to the following conditions:
//
//    The above copyright notice and this permission notice shall be included in
//    all copies or substantial portions of the Software.
//
//    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//    THE SOFTWARE.
//

#ifndef PLANECUT_H_INCLUDED
#define PLANECUT_H_INCLUDED


#include "RaycastingToPolygonLevel.h"

class PlaneCut
{
    public:
        PlaneCut(const RaycastingToPolygonLevel& raycastingToPolygonLevel, const Ogre::Plane& trajectoryPlane);
        void cutWithPlaneProjLoyal(const Ogre::Vector3& startTrajPoint, const Ogre::Vector3& endTrajPoint, const Ogre::Vector3& startRaycastPoint, const Ogre::Vector3& endRaycastPoint, const std::vector<Ogre::Vector3>& startIntersectedTriangles, const std::vector<Ogre::Vector3>& endIntersectedTriangles, std::list<Ogre::Vector3>& resultPositions, std::list< std::vector<Ogre::Vector3> >& resultIntersectedTris);
        void cutWithPlaneSurfLoyal(const Ogre::Vector3& startTrajPoint, const Ogre::Vector3& endTrajPoint, const Ogre::Vector3& startRaycastPoint, const Ogre::Vector3& endRaycastPoint, const std::vector<Ogre::Vector3>& startIntersectedTriangles, const std::vector<Ogre::Vector3>& endIntersectedTriangles, const bool& detectCycles, std::list<Ogre::Vector3>& resultPositions, std::list< std::vector<Ogre::Vector3> >& resultIntersectedTris);
    private:
        void cutWithPlaneProjLoyalGatherAndSortIntersectionData(const Ogre::Vector3& triangleIntersection);
        void cutWithPlaneSurfLoyalGatherIntersectionData(const Ogre::Vector3& triangleIntersection);
        void cutWithPlaneSurfLoyalBuildIntersectionIndexChainWithoutCycleDetection(const unsigned& startIntersectionIndex, const unsigned& startTriangleIndex, const bool& direction, bool& cycleDetected);
        void cutWithPlaneSurfLoyalBuildIntersectionIndexChainWithCycleDetection(const unsigned& startIntersectionIndex, const unsigned& startTriangleIndex, const bool& direction, bool& cycleDetected);
        std::list<unsigned>::iterator cutWithPlaneSurfLoyalBuildIntersectionIndexChainGetBeginPosition();
        std::list<unsigned>::iterator cutWithPlaneSurfLoyalBuildIntersectionIndexChainGetEndPosition();
        void cutWithPlaneSurfLoyalCheckIntersectionsOfDownRay(const Ogre::Vector3& firstTriangleIntersection, const Ogre::Vector3& lastTriangleIntersection, const unsigned& firstPointTriangleIndex, const unsigned& lastPointTriangleIndex, const std::list< std::list<unsigned> >::iterator& itChains);

        RaycastingToPolygonLevel raycasting;

        size_t index_count;
        const Ogre::Vector3* vertices;
        const unsigned long* indices;

        Ogre::Plane trajPlane;
        Ogre::Vector3 projVec;

        Ogre::Vector3 startTrajPoint;
        std::list<Ogre::Vector3> intersectionsWithinStartEnd;
        std::list< std::vector<Ogre::Vector3> > intersectedTriangles;
        std::list<Ogre::Real> intersectionDistances;

        std::vector<Ogre::Vector3> intersectionsWithinStartEndBothSides;
        std::vector< std::vector<unsigned> > triangleIndexes;
        std::vector<Ogre::Vector3> trianglesWithinStartEndBothSides;
        std::vector< std::vector<unsigned> > intersectionIndexes;
        std::vector<bool> trianglesUnchecked;
        bool triangleAdded;
        std::list<unsigned> intersectionIndexChain;

        Ogre::Vector3 startEndTrajHalf;
        Ogre::Real distanceStartEndTrajHalf;
        Ogre::Vector3 pointA;
        Ogre::Vector3 pointB;
        Ogre::Vector3 pointC;
};

#endif // PLANECUT_H_INCLUDED
