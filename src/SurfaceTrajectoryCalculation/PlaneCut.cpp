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

#include "SurfaceTrajectoryCalculation/PlaneCut.h"

// epsilon value for checking equality of floating point numbers (Ogre::Real)
const Ogre::Real epsilon = std::numeric_limits< Ogre::Real >::epsilon() * 10;


PlaneCut::PlaneCut(const RaycastingToPolygonLevel& raycastingToPolygonLevel, const Ogre::Plane& trajectoryPlane)
: raycasting(raycastingToPolygonLevel), trajPlane(trajectoryPlane), projVec(trajectoryPlane.normal)
{
    size_t vertex_count;
    const Ogre::Entity* ent;
    raycasting.getPreloadedMeshInformation(vertex_count, vertices, index_count, indices, ent);
}


void PlaneCut::cutWithPlaneProjLoyal(const Ogre::Vector3& startTrajectoryPoint, const Ogre::Vector3& endTrajectoryPoint, const Ogre::Vector3& startRaycastPoint, const Ogre::Vector3& endRaycastPoint, const std::vector<Ogre::Vector3>& startIntersectedTriangles, const std::vector<Ogre::Vector3>& endIntersectedTriangles, std::list<Ogre::Vector3>& resultPositions, std::list< std::vector<Ogre::Vector3> >& resultIntersectedTris)
{
    startTrajPoint = startTrajectoryPoint;

    std::list<Ogre::Vector3> intersectionsWithinStartEnd_;
    intersectionsWithinStartEnd = intersectionsWithinStartEnd_;
    std::list< std::vector<Ogre::Vector3> > intersectedTriangles_;
    intersectedTriangles = intersectedTriangles_;
    std::list<Ogre::Real> intersectionDistances_;
    intersectionDistances = intersectionDistances_;

    intersectionsWithinStartEnd.push_back(startRaycastPoint);
    intersectedTriangles.push_back(startIntersectedTriangles);
    intersectionsWithinStartEnd.push_back(endRaycastPoint);
    intersectedTriangles.push_back(endIntersectedTriangles);

    // if the two raycast-points are not identical then proceed, else just skip the calculation and return them
    if ( !(Ogre::Math::RealEqual(startRaycastPoint.x, endRaycastPoint.x, epsilon) &&
        Ogre::Math::RealEqual(startRaycastPoint.y, endRaycastPoint.y, epsilon) &&
        Ogre::Math::RealEqual(startRaycastPoint.z, endRaycastPoint.z, epsilon)) )
    {
        intersectionDistances.push_back(0.0f);
        intersectionDistances.push_back(startTrajPoint.distance(endTrajectoryPoint));

        Ogre::Plane cuttingPlane(startRaycastPoint, endRaycastPoint, startRaycastPoint.operator+(projVec));

        startEndTrajHalf = (startTrajPoint.operator+(endTrajectoryPoint)) / 2.0f;
        distanceStartEndTrajHalf = startEndTrajHalf.distance(startTrajPoint);

        for (int i=0; i<static_cast<int>(index_count); i += 3)
        {
            pointA = vertices[indices[i]];
            pointB = vertices[indices[i+1]];
            pointC = vertices[indices[i+2]];

            Ogre::Vector3 directionAB = pointB.operator-(pointA);
            Ogre::Vector3 directionBC = pointC.operator-(pointB);
            Ogre::Vector3 directionCA = pointA.operator-(pointC);

            directionAB.normalise();
            directionBC.normalise();
            directionCA.normalise();

            Ogre::Real distanceAB = pointA.distance(pointB);
            Ogre::Real distanceBC = pointB.distance(pointC);
            Ogre::Real distanceCA = pointC.distance(pointA);

            Ogre::Ray rayAB(pointA, directionAB);
            Ogre::Ray rayBC(pointB, directionBC);
            Ogre::Ray rayCA(pointC, directionCA);

            std::pair<bool, Ogre::Real> intersectionAB = rayAB.intersects(cuttingPlane);
            std::pair<bool, Ogre::Real> intersectionBC = rayBC.intersects(cuttingPlane);
            std::pair<bool, Ogre::Real> intersectionCA = rayCA.intersects(cuttingPlane);

            if (intersectionAB.first && (intersectionAB.second < distanceAB || Ogre::Math::RealEqual(intersectionAB.second, distanceAB, epsilon)) )
            {
                Ogre::Vector3 triangleIntersection = rayAB.getPoint(intersectionAB.second);
                cutWithPlaneProjLoyalGatherAndSortIntersectionData(triangleIntersection);
            }

            if (intersectionBC.first && (intersectionBC.second < distanceBC || Ogre::Math::RealEqual(intersectionBC.second, distanceBC, epsilon)) )
            {
                Ogre::Vector3 triangleIntersection = rayBC.getPoint(intersectionBC.second);
                cutWithPlaneProjLoyalGatherAndSortIntersectionData(triangleIntersection);
            }

            if (intersectionCA.first && (intersectionCA.second < distanceCA || Ogre::Math::RealEqual(intersectionCA.second, distanceCA, epsilon)) )
            {
                Ogre::Vector3 triangleIntersection = rayCA.getPoint(intersectionCA.second);
                cutWithPlaneProjLoyalGatherAndSortIntersectionData(triangleIntersection);
            }
        }

    }

    resultPositions = intersectionsWithinStartEnd;
    resultIntersectedTris = intersectedTriangles;
}

void PlaneCut::cutWithPlaneProjLoyalGatherAndSortIntersectionData(const Ogre::Vector3& triangleIntersection)
{
    Ogre::Ray backRay(triangleIntersection, -projVec);
    std::pair<bool, Ogre::Real> lineIntersection = backRay.intersects(trajPlane);
    Ogre::Vector3 lineIntersectionPos = backRay.getPoint(lineIntersection.second);

    if ( (startEndTrajHalf.distance(lineIntersectionPos) < distanceStartEndTrajHalf) || (Ogre::Math::RealEqual(startEndTrajHalf.distance(lineIntersectionPos), distanceStartEndTrajHalf, epsilon)) )
    {
        Ogre::Vector3 position;
        std::vector<Ogre::Vector3> tris;
        raycasting.raycastFromPointPreloaded(lineIntersectionPos, projVec, true, false, position, tris);
        if ( Ogre::Math::RealEqual(position.x, triangleIntersection.x, epsilon) &&
            Ogre::Math::RealEqual(position.y, triangleIntersection.y, epsilon) &&
            Ogre::Math::RealEqual(position.z, triangleIntersection.z, epsilon) )
        {

            Ogre::Real distance = startTrajPoint.distance(lineIntersectionPos);
            std::list<Ogre::Vector3>::iterator itPos;
            std::list< std::vector<Ogre::Vector3> >::iterator itTris;
            std::list<Ogre::Real>::iterator itDists;
            bool inserted = false;
            for (itPos=intersectionsWithinStartEnd.begin(), itTris=intersectedTriangles.begin(), itDists=intersectionDistances.begin(); itPos!=intersectionsWithinStartEnd.end() && !inserted; ++itPos, ++itTris, ++itDists)
            {
                if ( Ogre::Math::RealEqual((*itPos).x, triangleIntersection.x, epsilon) &&
                Ogre::Math::RealEqual((*itPos).y, triangleIntersection.y, epsilon) &&
                Ogre::Math::RealEqual((*itPos).z, triangleIntersection.z, epsilon) )
                {
                    (*itTris).push_back(pointA);
                    (*itTris).push_back(pointB);
                    (*itTris).push_back(pointC);
                    inserted = true;
                } else if ((*itDists) > distance)
                {
                    intersectionDistances.insert(itDists, distance);
                    intersectionsWithinStartEnd.insert(itPos, triangleIntersection);
                    std::vector<Ogre::Vector3> triangle;
                    triangle.push_back(pointA);
                    triangle.push_back(pointB);
                    triangle.push_back(pointC);
                    intersectedTriangles.insert(itTris, triangle);
                    inserted = true;
                }
            }
            if (!inserted)
            {
                intersectionDistances.push_back(distance);
                intersectionsWithinStartEnd.push_back(triangleIntersection);
                std::vector<Ogre::Vector3> triangle;
                triangle.push_back(pointA);
                triangle.push_back(pointB);
                triangle.push_back(pointC);
                intersectedTriangles.push_back(triangle);
            }
        }
    }
}

void PlaneCut::cutWithPlaneSurfLoyal(const Ogre::Vector3& startTrajectoryPoint, const Ogre::Vector3& endTrajectoryPoint, const Ogre::Vector3& startRaycastPoint, const Ogre::Vector3& endRaycastPoint, const std::vector<Ogre::Vector3>& startIntersectedTriangles, const std::vector<Ogre::Vector3>& endIntersectedTriangles, const bool& detectCycles, std::list<Ogre::Vector3>& resultPositions, std::list< std::vector<Ogre::Vector3> >& resultIntersectedTris)
{
    startTrajPoint = startTrajectoryPoint;

    std::list<Ogre::Vector3> intersectionsWithinStartEnd_;
    intersectionsWithinStartEnd = intersectionsWithinStartEnd_;
    std::list< std::vector<Ogre::Vector3> > intersectedTriangles_;
    intersectedTriangles = intersectedTriangles_;

    std::vector<Ogre::Vector3> intersectionsWithinStartEndBothSides_;
    intersectionsWithinStartEndBothSides = intersectionsWithinStartEndBothSides_;
    std::vector< std::vector<unsigned> > triangleIndexes_;
    triangleIndexes = triangleIndexes_;
    std::vector<Ogre::Vector3> trianglesWithinStartEndBothSides_;
    trianglesWithinStartEndBothSides = trianglesWithinStartEndBothSides_;
    std::vector< std::vector<unsigned> > intersectionIndexes_;
    intersectionIndexes = intersectionIndexes_;
    trianglesUnchecked.clear();

    // if the two raycast-points are not identical then proceed, else just skip the calculation and return them
    if ( !(Ogre::Math::RealEqual(startRaycastPoint.x, endRaycastPoint.x, epsilon) &&
        Ogre::Math::RealEqual(startRaycastPoint.y, endRaycastPoint.y, epsilon) &&
        Ogre::Math::RealEqual(startRaycastPoint.z, endRaycastPoint.z, epsilon)) )
    {



        //////////////////////////////
        // GATHER INTERSECTION DATA //
        //////////////////////////////


        Ogre::Plane cuttingPlane(startRaycastPoint, endRaycastPoint, startRaycastPoint.operator+(projVec));

        startEndTrajHalf = (startTrajPoint.operator+(endTrajectoryPoint)) / 2.0f;
        distanceStartEndTrajHalf = startEndTrajHalf.distance(startTrajPoint);

        for (int i=0; i<static_cast<int>(index_count); i += 3)
        {
            pointA = vertices[indices[i]];
            pointB = vertices[indices[i+1]];
            pointC = vertices[indices[i+2]];

            Ogre::Vector3 directionAB = pointB.operator-(pointA);
            Ogre::Vector3 directionBC = pointC.operator-(pointB);
            Ogre::Vector3 directionCA = pointA.operator-(pointC);

            directionAB.normalise();
            directionBC.normalise();
            directionCA.normalise();

            Ogre::Real distanceAB = pointA.distance(pointB);
            Ogre::Real distanceBC = pointB.distance(pointC);
            Ogre::Real distanceCA = pointC.distance(pointA);

            Ogre::Ray rayAB(pointA, directionAB);
            Ogre::Ray rayBC(pointB, directionBC);
            Ogre::Ray rayCA(pointC, directionCA);

            std::pair<bool, Ogre::Real> intersectionAB = rayAB.intersects(cuttingPlane);
            std::pair<bool, Ogre::Real> intersectionBC = rayBC.intersects(cuttingPlane);
            std::pair<bool, Ogre::Real> intersectionCA = rayCA.intersects(cuttingPlane);

            triangleAdded = false;

            if (intersectionAB.first && (intersectionAB.second < distanceAB || Ogre::Math::RealEqual(intersectionAB.second, distanceAB, epsilon)) )
            {
                Ogre::Vector3 triangleIntersection = rayAB.getPoint(intersectionAB.second);
                cutWithPlaneSurfLoyalGatherIntersectionData(triangleIntersection);
            }

            if (intersectionBC.first && (intersectionBC.second < distanceBC || Ogre::Math::RealEqual(intersectionBC.second, distanceBC, epsilon)) )
            {
                Ogre::Vector3 triangleIntersection = rayBC.getPoint(intersectionBC.second);
                cutWithPlaneSurfLoyalGatherIntersectionData(triangleIntersection);
            }

            if (intersectionCA.first && (intersectionCA.second < distanceCA || Ogre::Math::RealEqual(intersectionCA.second, distanceCA, epsilon)) )
            {
                Ogre::Vector3 triangleIntersection = rayCA.getPoint(intersectionCA.second);
                cutWithPlaneSurfLoyalGatherIntersectionData(triangleIntersection);
            }
        }



        /////////////////////////////////////
        // BUILD INTERSECTION INDEX CHAINS //
        /////////////////////////////////////


        // final list that collects all chain-lists that are built
        std::list< std::list<unsigned> > intersectionIndexChains;

        // set the correct chain-building function (for cycle detection or not)
        void (PlaneCut::*buildIntersectionIndexChain) (const unsigned& startIntersectionIndex, const unsigned& startTriangleIndex, const bool& direction, bool& cycleDetected);
        if (detectCycles)
        {
            buildIntersectionIndexChain = &PlaneCut::cutWithPlaneSurfLoyalBuildIntersectionIndexChainWithCycleDetection;
        } else
        {
            buildIntersectionIndexChain = &PlaneCut::cutWithPlaneSurfLoyalBuildIntersectionIndexChainWithoutCycleDetection;
        }

        // go over all traingles which have not yet been checked and take their intersection/s as start to build the chain of intersections it/they belong to
        for (unsigned i = 0; i < trianglesUnchecked.size(); i++)
        {


            if (trianglesUnchecked[i])
            {
                std::list<unsigned> intersectionIndexChain_;
                intersectionIndexChain = intersectionIndexChain_;


                // set up start parameters for the chain-building algorithms if needed

                bool leftDirectionExists = false;
                bool rightDirectionExists = false;
                unsigned leftIntersectionIndex;
                unsigned rightIntersectionIndex;
                unsigned leftTriangleIndex;
                unsigned rightTriangleIndex;
                // if we already hit a triangle with two intersections:
                // directly set up start parameters for the chain-building algorithms
                if (intersectionIndexes[i].size() == 2)
                {
                    leftIntersectionIndex = intersectionIndexes[i][0];
                    leftTriangleIndex = i;
                    leftDirectionExists = true;

                    rightIntersectionIndex = intersectionIndexes[i][1];
                    rightTriangleIndex = i;
                    rightDirectionExists = true;

                    // mark the i'th triangle as checked (as it will not be considered by the two chain-building algorithms)
                    trianglesUnchecked[i] = false;

                // else we have a triangle with only one intersection and we have to check for several cases that may exist:
                //
                // 1.) the given intersection may be the only one in this chain ("peak" into the plane-cut area):
                // the only thing we have to do in this case is to insert it into the chain-list
                // 2.) there are more intersections in this chain and we have to find the one or two triangles which share the given intersection
                // and have two intersections in total (which means that the trajectory runs across them):
                // in this case we have to set up the start parameters for the chain-building algorithms, so the chain can be completed to the "left"
                // and "right" of the given intersection
                } else {
                    // go over the triangles of the given intersection and check whether there are triangles with two intersections
                    for (unsigned j = 0; j < triangleIndexes[intersectionIndexes[i][0]].size(); j++)
                    {
                        // if there is a triangle with two intersections
                        if (intersectionIndexes[triangleIndexes[intersectionIndexes[i][0]][j] / 3].size() == 2)
                        {

                            // if it is the first triangle found with two intersections: set up start parameters for the first chain-building algorithm (the direction in whitch the chain is built is referred to as "left")
                            if (!leftDirectionExists)
                            {
                                // get the correct intersection index (whitch is not the one whitch was originally given)
                                if (intersectionIndexes[triangleIndexes[intersectionIndexes[i][0]][j] / 3][0] != intersectionIndexes[i][0])
                                {
                                    leftIntersectionIndex = intersectionIndexes[triangleIndexes[intersectionIndexes[i][0]][j] / 3][0];
                                } else {
                                    leftIntersectionIndex = intersectionIndexes[triangleIndexes[intersectionIndexes[i][0]][j] / 3][1];
                                }
                                leftTriangleIndex = triangleIndexes[intersectionIndexes[i][0]][j] / 3;
                                leftDirectionExists = true;
                            // if it is the second triangle found with two intersections: set up start parameters for the second chain-building algorithm (the direction in whitch the chain is built is referred to as "right")
                            } else {
                                // get the correct intersection index (whitch is not the one whitch was originally given)
                                if (intersectionIndexes[triangleIndexes[intersectionIndexes[i][0]][j] / 3][0] != intersectionIndexes[i][0])
                                {
                                    rightIntersectionIndex = intersectionIndexes[triangleIndexes[intersectionIndexes[i][0]][j] / 3][0];
                                } else {
                                    rightIntersectionIndex = intersectionIndexes[triangleIndexes[intersectionIndexes[i][0]][j] / 3][1];
                                }
                                rightTriangleIndex = triangleIndexes[intersectionIndexes[i][0]][j] / 3;
                                rightDirectionExists = true;
                            }

                        }
                        // mark all triangles that belong to the originally given intersection as checked (as they will not be considered by the chain-building algorithms)
                        trianglesUnchecked[triangleIndexes[intersectionIndexes[i][0]][j] / 3] = false;
                    }
                    // insert the originally given intersection into the chain-list as the first element
                    // (if the chain consists of more than one intersection, it will be completed to the "left" and/or "right" direction by the chain-building algorithms that follow)
                    intersectionIndexChain.insert(intersectionIndexChain.begin(), intersectionIndexes[i][0]);
                }


                // now run chain-building algorithms if needed

                bool cycleDetected = false;

                // build chain to the "left" direction if needed
                if (leftDirectionExists)
                {
                    ((*this).*buildIntersectionIndexChain)(leftIntersectionIndex, leftTriangleIndex, true, cycleDetected);
                }
                // build chain to the "right" direction if needed
                if (rightDirectionExists)
                {
                    ((*this).*buildIntersectionIndexChain)(rightIntersectionIndex, rightTriangleIndex, false, cycleDetected);
                }

                // if there is no cycle in this chain of intersections: add constructed chain-list to the list of chain-lists
                if (!cycleDetected)
                {
                    intersectionIndexChains.push_back(intersectionIndexChain);
                // if there is a cycle: just ignore this chain
                } else
                {
                    printf("Warning: Cycle in a chain of intersections detected! This chain will be removed from the output.\n");
                }
            }


        }







/*
        std::list< std::list<unsigned> >::iterator it1;
        std::list<unsigned>::iterator it2;
        int cntr = 0;
        for (it1 = intersectionIndexChains.begin(); it1 != intersectionIndexChains.end(); ++it1)
        {
            for (it2 = (*it1).begin(); it2 != (*it1).end(); ++it2)
            {
                //printf("[%d] %d\n", cntr, (*it2));
                printf("[%d] (%f, %f, %f)\n", cntr, intersectionsWithinStartEndBothSides[(*it2)].x,intersectionsWithinStartEndBothSides[(*it2)].y,intersectionsWithinStartEndBothSides[(*it2)].z);
            }
            cntr++;

        }
        */







        ////////////////////////////////////
        // SORT INTERSECTION INDEX CHAINS //
        ////////////////////////////////////


        std::list< std::list<unsigned> > startLists;
        std::list<Ogre::Real> startListsDistances;
        std::list< std::list<unsigned> > endLists;
        std::list<Ogre::Real> endListsDistances;
        std::list< std::list<unsigned> > connectionLists;
        std::list<Ogre::Real> connectionListsDistancesStart;
        std::list<Ogre::Real> connectionListsDistancesEnd;
        std::list< std::list<unsigned> >::iterator itChains;

        // sort every chain-list into one of three different lists depending on whether the chain begins "on" the ray down the start/end raycast point
        // and returns to it (startLists/endLists) or whether it starts "on" one of the rays and leads to the other (connectionLists)

        // offset the actual starting points (startRaycastPoint/endRaycastPoint) for the rays a bit to ensure a ray-triangle intersection also hits them
        Ogre::Ray startDownRay(startTrajectoryPoint, projVec);
        Ogre::Ray endDownRay(endTrajectoryPoint, projVec);
        bool chainAdded = false;
        for (itChains = intersectionIndexChains.begin(); itChains != intersectionIndexChains.end(); ++itChains)
        {


            chainAdded = false;
            // go over all triangles of the first point in the current chain-list and check for an intersection with both DownRays (only facing triangles)
            for (unsigned i = 0; i < triangleIndexes[(*itChains).front()].size() && !chainAdded; i++)
            {
                std::pair<bool, Ogre::Real> startFirstFacingIntersection = Ogre::Math::intersects(startDownRay, trianglesWithinStartEndBothSides[triangleIndexes[(*itChains).front()][i]], trianglesWithinStartEndBothSides[triangleIndexes[(*itChains).front()][i] + 1], trianglesWithinStartEndBothSides[triangleIndexes[(*itChains).front()][i] + 2], true, false);
                std::pair<bool, Ogre::Real> endFirstFacingIntersection = Ogre::Math::intersects(endDownRay, trianglesWithinStartEndBothSides[triangleIndexes[(*itChains).front()][i]], trianglesWithinStartEndBothSides[triangleIndexes[(*itChains).front()][i] + 1], trianglesWithinStartEndBothSides[triangleIndexes[(*itChains).front()][i] + 2], true, false);

                // the begin of the current chain-list is "on" the startDownRay
                if (startFirstFacingIntersection.first)
                {
                    // go over all triangles of the last point in the current chain-list and check for an intersection with both DownRays (startDownRay: non-facing triangles, endDownRay: facing triangles)
                    for (unsigned j = 0; j < triangleIndexes[(*itChains).back()].size() && !chainAdded; j++)
                    {
                        std::pair<bool, Ogre::Real> startLastNonFacingIntersection = Ogre::Math::intersects(startDownRay, trianglesWithinStartEndBothSides[triangleIndexes[(*itChains).back()][j]], trianglesWithinStartEndBothSides[triangleIndexes[(*itChains).back()][j] + 1], trianglesWithinStartEndBothSides[triangleIndexes[(*itChains).back()][j] + 2], false, true);
                        std::pair<bool, Ogre::Real> endLastFacingIntersection = Ogre::Math::intersects(endDownRay, trianglesWithinStartEndBothSides[triangleIndexes[(*itChains).back()][j]], trianglesWithinStartEndBothSides[triangleIndexes[(*itChains).back()][j] + 1], trianglesWithinStartEndBothSides[triangleIndexes[(*itChains).back()][j] + 2], true, false);

                        // list is a startList
                        if (startLastNonFacingIntersection.first)
                        {
                            Ogre::Real distance = startFirstFacingIntersection.second;

                            // check if list is not at the back of concave geometry extending into the plane-cut area which means that the distance to the facing intersection point of the DownRay must be smaller than that to the non-facing one
                            if (distance < startLastNonFacingIntersection.second)
                            {

//printf("startList #1\n");


                                Ogre::Vector3 firstTriangleIntersection = startDownRay.getPoint(startFirstFacingIntersection.second);
                                Ogre::Vector3 lastTriangleIntersection = startDownRay.getPoint(startLastNonFacingIntersection.second);

                                // check if the intersection points of the DownRay already exist and add the data if not
                                cutWithPlaneSurfLoyalCheckIntersectionsOfDownRay(firstTriangleIntersection, lastTriangleIntersection, i, j, itChains);


                                // insert list at the right position
                                bool inserted = false;
                                std::list<Ogre::Real>::iterator itDists;
                                std::list< std::list<unsigned> >::iterator itLists;
                                for (itDists = startListsDistances.begin(), itLists = startLists.begin(); itDists != startListsDistances.end() && !inserted; ++itDists, ++itLists)
                                {
                                    if ((*itDists) > distance)
                                    {
                                        startListsDistances.insert(itDists, distance);
                                        startLists.insert(itLists, (*itChains));
                                        inserted = true;
                                    }
                                }
                                if (!inserted)
                                {
                                    startListsDistances.push_back(distance);
                                    startLists.push_back(*itChains);
                                }
                                chainAdded = true;
                            }

                        // list is a connectionList
                        } else if (endLastFacingIntersection.first)
                        {

//printf("connectionList #1\n");

                            Ogre::Real distanceStart = startFirstFacingIntersection.second;
                            Ogre::Real distanceEnd = endLastFacingIntersection.second;
                            Ogre::Vector3 firstTriangleIntersection = startDownRay.getPoint(startFirstFacingIntersection.second);
                            Ogre::Vector3 lastTriangleIntersection = endDownRay.getPoint(endLastFacingIntersection.second);

                            // check if the intersection points of the DownRay already exist and add the data if not
                            cutWithPlaneSurfLoyalCheckIntersectionsOfDownRay(firstTriangleIntersection, lastTriangleIntersection, i, j, itChains);


                            // insert list at the right position
                            bool inserted = false;
                            std::list<Ogre::Real>::iterator itDistsStart;
                            std::list<Ogre::Real>::iterator itDistsEnd;
                            std::list< std::list<unsigned> >::iterator itLists;
                            for (itDistsStart = connectionListsDistancesStart.begin(), itDistsEnd = connectionListsDistancesEnd.begin(), itLists = connectionLists.begin(); itDistsStart != connectionListsDistancesStart.end() && !inserted; ++itDistsStart, ++itDistsEnd, ++itLists)
                            {
                                if ((*itDistsStart) > distanceStart)
                                {
                                    connectionListsDistancesStart.insert(itDistsStart, distanceStart);
                                    connectionListsDistancesEnd.insert(itDistsEnd, distanceEnd);
                                    connectionLists.insert(itLists, (*itChains));
                                    inserted = true;
                                }
                            }
                            if (!inserted)
                            {
                                connectionListsDistancesStart.push_back(distanceStart);
                                connectionListsDistancesEnd.push_back(distanceEnd);
                                connectionLists.push_back(*itChains);
                            }
                            chainAdded = true;

                        }
                    }

                // the begin of the current chain-list is "on" the endDownRay
                } else if (endFirstFacingIntersection.first)
                {
                    // go over all triangles of the last point in the current chain-list and check for an intersection with both DownRays (endDownRay: non-facing triangles, startDownRay: facing triangles)
                    for (unsigned j = 0; j < triangleIndexes[(*itChains).back()].size() && !chainAdded; j++)
                    {
                        std::pair<bool, Ogre::Real> endLastNonFacingIntersection = Ogre::Math::intersects(endDownRay, trianglesWithinStartEndBothSides[triangleIndexes[(*itChains).back()][j]], trianglesWithinStartEndBothSides[triangleIndexes[(*itChains).back()][j] + 1], trianglesWithinStartEndBothSides[triangleIndexes[(*itChains).back()][j] + 2], false, true);
                        std::pair<bool, Ogre::Real> startLastFacingIntersection = Ogre::Math::intersects(startDownRay, trianglesWithinStartEndBothSides[triangleIndexes[(*itChains).back()][j]], trianglesWithinStartEndBothSides[triangleIndexes[(*itChains).back()][j] + 1], trianglesWithinStartEndBothSides[triangleIndexes[(*itChains).back()][j] + 2], true, false);

                        // list is an endList
                        if (endLastNonFacingIntersection.first)
                        {
                            Ogre::Real distance = endFirstFacingIntersection.second;

                            // check if list is not at the back of concave geometry extending into the plane-cut area which means that the distance to the facing intersection point of the DownRay must be smaller than that to the non-facing one
                            if (distance < endLastNonFacingIntersection.second)
                            {

//printf("endList #1\n");


                                Ogre::Vector3 firstTriangleIntersection = endDownRay.getPoint(endFirstFacingIntersection.second);
                                Ogre::Vector3 lastTriangleIntersection = endDownRay.getPoint(endLastNonFacingIntersection.second);

                                // check if the intersection points of the DownRay already exist and add the data if not
                                cutWithPlaneSurfLoyalCheckIntersectionsOfDownRay(firstTriangleIntersection, lastTriangleIntersection, i, j, itChains);


                                // reverse chain-list so the elements are already in the correct order for later insertion into a final list
                                (*itChains).reverse();


                                // insert list at the right position
                                bool inserted = false;
                                std::list<Ogre::Real>::iterator itDists;
                                std::list< std::list<unsigned> >::iterator itLists;
                                for (itDists = endListsDistances.begin(), itLists = endLists.begin(); itDists != endListsDistances.end() && !inserted; ++itDists, ++itLists)
                                {
                                    if ((*itDists) > distance)
                                    {
                                        endListsDistances.insert(itDists, distance);
                                        endLists.insert(itLists, (*itChains));
                                        inserted = true;
                                    }
                                }
                                if (!inserted)
                                {
                                    endListsDistances.push_back(distance);
                                    endLists.push_back(*itChains);
                                }
                                chainAdded = true;
                            }

                        // list is a connectionList
                        } else if (startLastFacingIntersection.first)
                        {


//printf("connectionList #2\n");


                            Ogre::Real distanceStart = startLastFacingIntersection.second;
                            Ogre::Real distanceEnd = endFirstFacingIntersection.second;
                            Ogre::Vector3 firstTriangleIntersection = endDownRay.getPoint(endFirstFacingIntersection.second);
                            Ogre::Vector3 lastTriangleIntersection = startDownRay.getPoint(startLastFacingIntersection.second);

                            // check if the intersection points of the DownRay already exist and add the data if not
                            cutWithPlaneSurfLoyalCheckIntersectionsOfDownRay(firstTriangleIntersection, lastTriangleIntersection, i, j, itChains);


                            // reverse chain-list so the elements are already in the correct order for later insertion into a final list
                            (*itChains).reverse();


                           // insert list at the right position
                            bool inserted = false;
                            std::list<Ogre::Real>::iterator itDistsStart;
                            std::list<Ogre::Real>::iterator itDistsEnd;
                            std::list< std::list<unsigned> >::iterator itLists;
                            for (itDistsStart = connectionListsDistancesStart.begin(), itDistsEnd = connectionListsDistancesEnd.begin(), itLists = connectionLists.begin(); itDistsStart != connectionListsDistancesStart.end() && !inserted; ++itDistsStart, ++itDistsEnd, ++itLists)
                            {
                                if ((*itDistsStart) > distanceStart)
                                {
                                    connectionListsDistancesStart.insert(itDistsStart, distanceStart);
                                    connectionListsDistancesEnd.insert(itDistsEnd, distanceEnd);
                                    connectionLists.insert(itLists, (*itChains));
                                    inserted = true;
                                }
                            }
                            if (!inserted)
                            {
                                connectionListsDistancesStart.push_back(distanceStart);
                                connectionListsDistancesEnd.push_back(distanceEnd);
                                connectionLists.push_back(*itChains);
                            }
                            chainAdded = true;

                        }
                    }

                }

            }

            // if we could not add the current chain-list yet then try it the other way round: check the triangles of the last point for a possible beginning of the chain
            // (note that we do not have to check for connectionLists anymore as they are fully covered by the previous code)
            if (!chainAdded)
            {
                // go over all triangles of the last point in the current chain-list and check for an intersection with both DownRays (only facing triangles)
                for (unsigned j = 0; j < triangleIndexes[(*itChains).back()].size() && !chainAdded; j++)
                {
                    std::pair<bool, Ogre::Real> startLastFacingIntersection = Ogre::Math::intersects(startDownRay, trianglesWithinStartEndBothSides[triangleIndexes[(*itChains).back()][j]], trianglesWithinStartEndBothSides[triangleIndexes[(*itChains).back()][j] + 1], trianglesWithinStartEndBothSides[triangleIndexes[(*itChains).back()][j] + 2], true, false);
                    std::pair<bool, Ogre::Real> endLastFacingIntersection = Ogre::Math::intersects(endDownRay, trianglesWithinStartEndBothSides[triangleIndexes[(*itChains).back()][j]], trianglesWithinStartEndBothSides[triangleIndexes[(*itChains).back()][j] + 1], trianglesWithinStartEndBothSides[triangleIndexes[(*itChains).back()][j] + 2], true, false);

                    // the begin of the current chain-list is "on" the startDownRay
                    if (startLastFacingIntersection.first)
                    {
                        // go over all triangles of the first point in the current chain-list and check for an intersection with startDownRay (non-facing triangles)
                        for (unsigned i = 0; i < triangleIndexes[(*itChains).front()].size() && !chainAdded; i++)
                        {
                            std::pair<bool, Ogre::Real> startFirstNonFacingIntersection = Ogre::Math::intersects(startDownRay, trianglesWithinStartEndBothSides[triangleIndexes[(*itChains).front()][i]], trianglesWithinStartEndBothSides[triangleIndexes[(*itChains).front()][i] + 1], trianglesWithinStartEndBothSides[triangleIndexes[(*itChains).front()][i] + 2], false, true);

                            // list is a startList
                            if (startFirstNonFacingIntersection.first)
                            {
                                Ogre::Real distance = startLastFacingIntersection.second;

                                // check if list is not at the back of concave geometry extending into the plane-cut area which means that the distance to the facing intersection point of the DownRay must be smaller than that to the non-facing one
                                if (distance < startFirstNonFacingIntersection.second)
                                {

//printf("startList #2\n");


                                    Ogre::Vector3 firstTriangleIntersection = startDownRay.getPoint(startFirstNonFacingIntersection.second);
                                    Ogre::Vector3 lastTriangleIntersection = startDownRay.getPoint(startLastFacingIntersection.second);

                                    // check if the intersection points of the DownRay already exist and add the data if not
                                    cutWithPlaneSurfLoyalCheckIntersectionsOfDownRay(firstTriangleIntersection, lastTriangleIntersection, i, j, itChains);


                                    // reverse chain-list so the elements are already in the correct order for later insertion into a final list
                                    (*itChains).reverse();


                                    // insert list at the right position
                                    bool inserted = false;
                                    std::list<Ogre::Real>::iterator itDists;
                                    std::list< std::list<unsigned> >::iterator itLists;
                                    for (itDists = startListsDistances.begin(), itLists = startLists.begin(); itDists != startListsDistances.end() && !inserted; ++itDists, ++itLists)
                                    {
                                        if ((*itDists) > distance)
                                        {
                                            startListsDistances.insert(itDists, distance);
                                            startLists.insert(itLists, (*itChains));
                                            inserted = true;
                                        }
                                    }
                                    if (!inserted)
                                    {
                                        startListsDistances.push_back(distance);
                                        startLists.push_back(*itChains);
                                    }
                                    chainAdded = true;
                                }
                            }
                        }

                    // the begin of the current chain-list is "on" the endDownRay
                    } else if (endLastFacingIntersection.first)
                    {
                        // go over all triangles of the first point in the current chain-list and check for an intersection with endDownRay (non-facing triangles)
                        for (unsigned i = 0; i < triangleIndexes[(*itChains).front()].size() && !chainAdded; i++)
                        {
                            std::pair<bool, Ogre::Real> endFirstNonFacingIntersection = Ogre::Math::intersects(endDownRay, trianglesWithinStartEndBothSides[triangleIndexes[(*itChains).front()][i]], trianglesWithinStartEndBothSides[triangleIndexes[(*itChains).front()][i] + 1], trianglesWithinStartEndBothSides[triangleIndexes[(*itChains).front()][i] + 2], false, true);

                            // list is an endList
                            if (endFirstNonFacingIntersection.first)
                            {
                                Ogre::Real distance = endLastFacingIntersection.second;

                                // check if list is not at the back of concave geometry extending into the plane-cut area which means that the distance to the facing intersection point of the DownRay must be smaller than that to the non-facing one
                                if (distance < endFirstNonFacingIntersection.second)
                                {

//printf("endList #2\n");


                                    Ogre::Vector3 firstTriangleIntersection = endDownRay.getPoint(endFirstNonFacingIntersection.second);
                                    Ogre::Vector3 lastTriangleIntersection = endDownRay.getPoint(endLastFacingIntersection.second);

                                    // check if the intersection points of the DownRay already exist and add the data if not
                                    cutWithPlaneSurfLoyalCheckIntersectionsOfDownRay(firstTriangleIntersection, lastTriangleIntersection, i, j, itChains);


                                    // insert list at the right position
                                    bool inserted = false;
                                    std::list<Ogre::Real>::iterator itDists;
                                    std::list< std::list<unsigned> >::iterator itLists;
                                    for (itDists = endListsDistances.begin(), itLists = endLists.begin(); itDists != endListsDistances.end() && !inserted; ++itDists, ++itLists)
                                    {
                                        if ((*itDists) > distance)
                                        {
                                            endListsDistances.insert(itDists, distance);
                                            endLists.insert(itLists, (*itChains));
                                            inserted = true;
                                        }
                                    }
                                    if (!inserted)
                                    {
                                        endListsDistances.push_back(distance);
                                        endLists.push_back(*itChains);
                                    }
                                    chainAdded = true;
                                }
                            }
                        }

                    }
                }

            }


        }



        // insert the sorted chains into one single sorted list

        std::list<unsigned> finalSortedIntersectionsIndexList;

        // insert startLists
        std::list< std::list<unsigned> >::iterator itStartLists;
        std::list<Ogre::Real>::iterator itStartDists;
        for (itStartLists = startLists.begin(), itStartDists = startListsDistances.begin(); itStartLists != startLists.end() && ((*itStartDists) < connectionListsDistancesStart.front()); ++itStartLists, ++itStartDists)
        {



//printf("startList inserted!!!!!\n");




            finalSortedIntersectionsIndexList.insert(finalSortedIntersectionsIndexList.end(), (*itStartLists).begin(), (*itStartLists).end());
        }

        // insert the highest connectionList
        finalSortedIntersectionsIndexList.insert(finalSortedIntersectionsIndexList.end(), (connectionLists.front()).begin(), (connectionLists.front()).end());

        // insert endLists
        // (create temporary list to append the endLists in the correct order and then add it to the end of the final list)
        std::list<unsigned> tempFinalList;
        std::list< std::list<unsigned> >::iterator itEndLists;
        std::list<Ogre::Real>::iterator itEndDists;
        for (itEndLists = endLists.begin(), itEndDists = endListsDistances.begin(); itEndLists != endLists.end() && ((*itEndDists) < connectionListsDistancesEnd.front()); ++itEndLists, ++itEndDists)
        {



//printf("endList inserted!!!!!\n");



            tempFinalList.insert(tempFinalList.begin(), (*itEndLists).begin(), (*itEndLists).end());
        }
        finalSortedIntersectionsIndexList.insert(finalSortedIntersectionsIndexList.end(), tempFinalList.begin(), tempFinalList.end());


        // remove (index of) first and last point which are equal to the raycast-points as they are inserted later anyway
        // (so in the output list there are exactly the two raycast-points which were originally passed and there will be no error due to floating point precision)
        if (finalSortedIntersectionsIndexList.size() >= 2)
        {
            finalSortedIntersectionsIndexList.pop_front();
            finalSortedIntersectionsIndexList.pop_back();
        }


        // fill output lists
        std::list<unsigned>::iterator itFinalSorted;
        for (itFinalSorted = finalSortedIntersectionsIndexList.begin(); itFinalSorted != finalSortedIntersectionsIndexList.end(); ++itFinalSorted)
        {
            intersectionsWithinStartEnd.push_back(intersectionsWithinStartEndBothSides[(*itFinalSorted)]);

            std::vector<Ogre::Vector3> triangles;
            for (unsigned i = 0; i < triangleIndexes[(*itFinalSorted)].size(); i++)
            {
                triangles.push_back(trianglesWithinStartEndBothSides[triangleIndexes[(*itFinalSorted)][i]]);
                triangles.push_back(trianglesWithinStartEndBothSides[triangleIndexes[(*itFinalSorted)][i] + 1]);
                triangles.push_back(trianglesWithinStartEndBothSides[triangleIndexes[(*itFinalSorted)][i] + 2]);
            }
            intersectedTriangles.push_back(triangles);
        }



    }

    // insert the two raycast-points and their intersected triangles into the output lists
    intersectionsWithinStartEnd.push_front(startRaycastPoint);
    intersectionsWithinStartEnd.push_back(endRaycastPoint);
    intersectedTriangles.push_front(startIntersectedTriangles);
    intersectedTriangles.push_back(endIntersectedTriangles);


    resultPositions = intersectionsWithinStartEnd;
    resultIntersectedTris = intersectedTriangles;
}

void PlaneCut::cutWithPlaneSurfLoyalGatherIntersectionData(const Ogre::Vector3& triangleIntersection)
{
    Ogre::Ray backRay(triangleIntersection, -projVec);
    std::pair<bool, Ogre::Real> lineIntersection = backRay.intersects(trajPlane);
    Ogre::Vector3 lineIntersectionPos = backRay.getPoint(lineIntersection.second);

    if ( (startEndTrajHalf.distance(lineIntersectionPos) < distanceStartEndTrajHalf) || (Ogre::Math::RealEqual(startEndTrajHalf.distance(lineIntersectionPos), distanceStartEndTrajHalf, epsilon)) )
    {

        bool intersectionExists = false;
        for (unsigned i = 0; i < intersectionsWithinStartEndBothSides.size() && !intersectionExists; i++)
        {
            if ( Ogre::Math::RealEqual(intersectionsWithinStartEndBothSides[i].x, triangleIntersection.x, epsilon) &&
            Ogre::Math::RealEqual(intersectionsWithinStartEndBothSides[i].y, triangleIntersection.y, epsilon) &&
            Ogre::Math::RealEqual(intersectionsWithinStartEndBothSides[i].z, triangleIntersection.z, epsilon) )
            {
                intersectionExists = true;

                if (!triangleAdded)
                {
                    triangleIndexes[i].push_back(trianglesWithinStartEndBothSides.size());
                    trianglesWithinStartEndBothSides.push_back(pointA);
                    trianglesWithinStartEndBothSides.push_back(pointB);
                    trianglesWithinStartEndBothSides.push_back(pointC);
                    std::vector<unsigned> intIndexes;
                    intIndexes.push_back(i);
                    intersectionIndexes.push_back(intIndexes);

                    trianglesUnchecked.push_back(true);
                    triangleAdded = true;
                } else {
                    triangleIndexes[i].push_back(trianglesWithinStartEndBothSides.size() - 3);
                    intersectionIndexes[intersectionIndexes.size() - 1].push_back(i);
                }
            }
        }

        if (!intersectionExists)
        {
            intersectionsWithinStartEndBothSides.push_back(triangleIntersection);
            if (!triangleAdded)
            {
                std::vector<unsigned> triIndexes;
                triIndexes.push_back(trianglesWithinStartEndBothSides.size());
                triangleIndexes.push_back(triIndexes);
                trianglesWithinStartEndBothSides.push_back(pointA);
                trianglesWithinStartEndBothSides.push_back(pointB);
                trianglesWithinStartEndBothSides.push_back(pointC);
                std::vector<unsigned> intIndexes;
                intIndexes.push_back(intersectionsWithinStartEndBothSides.size() - 1);
                intersectionIndexes.push_back(intIndexes);

                trianglesUnchecked.push_back(true);
                triangleAdded = true;
            } else {
                std::vector<unsigned> triIndexes;
                triIndexes.push_back(trianglesWithinStartEndBothSides.size() - 3);
                triangleIndexes.push_back(triIndexes);
                intersectionIndexes[intersectionIndexes.size() - 1].push_back(intersectionsWithinStartEndBothSides.size() - 1);
            }
        }

    }

}

void PlaneCut::cutWithPlaneSurfLoyalBuildIntersectionIndexChainWithoutCycleDetection(const unsigned& startIntersectionIndex, const unsigned& startTriangleIndex, const bool& direction, bool& cycleDetected)
{
    // set the correct retrieving function for the insertion iterator position of intersectionIndexChain for the given direction
    std::list<unsigned>::iterator (PlaneCut::*getInsertionPosition) ();
    if (direction)
    {
        getInsertionPosition = &PlaneCut::cutWithPlaneSurfLoyalBuildIntersectionIndexChainGetBeginPosition;
    } else
    {
        getInsertionPosition = &PlaneCut::cutWithPlaneSurfLoyalBuildIntersectionIndexChainGetEndPosition;
    }


    // set the variables to the start parameters and insert the start intersection into the output list
    unsigned currentIntersectionIndex = startIntersectionIndex;
    unsigned currentTriangleIndex = startTriangleIndex;
    bool neighbourIntersectionExists = true;
    intersectionIndexChain.insert(((*this).*getInsertionPosition)(), currentIntersectionIndex);

    // while there was a neighbouring intersection in the last iteration: search for another neighbouring intersection
    while (neighbourIntersectionExists)
    {
        neighbourIntersectionExists = false;

        unsigned normedTriIndex;
        // check all triangles of the current intersection for a triangle which is not the current triangle and which has two intersections in total
        for (unsigned i = 0; i < triangleIndexes[currentIntersectionIndex].size(); i++)
        {
            normedTriIndex = triangleIndexes[currentIntersectionIndex][i] / 3;

            // if the checked triangle is not the current triangle
            if (normedTriIndex != currentTriangleIndex)
            {
                // as we go over all triangles of each intersection of this chain we mark them one by one as checked
                trianglesUnchecked[normedTriIndex] = false;

                // if the checked triangle has two intersections in total
                if (intersectionIndexes[normedTriIndex].size() == 2)
                {
                    // get the right next intersection (which is not the current one)
                    if (intersectionIndexes[normedTriIndex][0] != currentIntersectionIndex)
                    {
                        currentIntersectionIndex = intersectionIndexes[normedTriIndex][0];
                    } else {
                        currentIntersectionIndex = intersectionIndexes[normedTriIndex][1];
                    }
                    currentTriangleIndex = normedTriIndex;
                    neighbourIntersectionExists = true;

                    // insert intersection into the chain-list
                    intersectionIndexChain.insert(((*this).*getInsertionPosition)(), currentIntersectionIndex);
                }
            }

        }
    }

}

void PlaneCut::cutWithPlaneSurfLoyalBuildIntersectionIndexChainWithCycleDetection(const unsigned& startIntersectionIndex, const unsigned& startTriangleIndex, const bool& direction, bool& cycleDetected)
{
    // set the correct retrieving function for the insertion iterator position of intersectionIndexChain for the given direction
    std::list<unsigned>::iterator (PlaneCut::*getInsertionPosition) ();
    if (direction)
    {
        getInsertionPosition = &PlaneCut::cutWithPlaneSurfLoyalBuildIntersectionIndexChainGetBeginPosition;
    } else
    {
        getInsertionPosition = &PlaneCut::cutWithPlaneSurfLoyalBuildIntersectionIndexChainGetEndPosition;
    }


    // set the variables to the start parameters and insert the start intersection into the output list
    unsigned currentIntersectionIndex = startIntersectionIndex;
    unsigned currentTriangleIndex = startTriangleIndex;
    bool neighbourIntersectionExists = true;
    intersectionIndexChain.insert(((*this).*getInsertionPosition)(), currentIntersectionIndex);

    // while there was a neighbouring intersection in the last iteration: search for another neighbouring intersection
    while (neighbourIntersectionExists)
    {
        neighbourIntersectionExists = false;

        unsigned normedTriIndex;
        // check all triangles of the current intersection for a triangle which is not the current triangle and which has two intersections in total
        for (unsigned i = 0; i < triangleIndexes[currentIntersectionIndex].size(); i++)
        {
            normedTriIndex = triangleIndexes[currentIntersectionIndex][i] / 3;

            // if the checked triangle is not the current triangle
            if (normedTriIndex != currentTriangleIndex)
            {
                // as we go over all triangles of each intersection of this chain we mark them one by one as checked
                trianglesUnchecked[normedTriIndex] = false;

                // if the checked triangle has two intersections in total
                if (intersectionIndexes[normedTriIndex].size() == 2)
                {
                    // get the right next intersection (which is not the current one)
                    if (intersectionIndexes[normedTriIndex][0] != currentIntersectionIndex)
                    {
                        currentIntersectionIndex = intersectionIndexes[normedTriIndex][0];
                    } else {
                        currentIntersectionIndex = intersectionIndexes[normedTriIndex][1];
                    }
                    currentTriangleIndex = normedTriIndex;
                    neighbourIntersectionExists = true;

                    // check for a cycle of intersections
                    std::list<unsigned>::iterator it;
                    for (it = intersectionIndexChain.begin(); it != intersectionIndexChain.end(); ++it)
                    {
                        if ((*it) == currentIntersectionIndex)
                        {
                            cycleDetected = true;
                            neighbourIntersectionExists = false;
                        }
                    }

                    // insert intersection into the chain-list
                    intersectionIndexChain.insert(((*this).*getInsertionPosition)(), currentIntersectionIndex);
                }
            }

        }
    }

}

std::list<unsigned>::iterator PlaneCut::cutWithPlaneSurfLoyalBuildIntersectionIndexChainGetBeginPosition()
{
    return intersectionIndexChain.begin();
}

std::list<unsigned>::iterator PlaneCut::cutWithPlaneSurfLoyalBuildIntersectionIndexChainGetEndPosition()
{
    return intersectionIndexChain.end();
}

void PlaneCut::cutWithPlaneSurfLoyalCheckIntersectionsOfDownRay(const Ogre::Vector3& firstTriangleIntersection, const Ogre::Vector3& lastTriangleIntersection, const unsigned& firstPointTriangleIndex, const unsigned& lastPointTriangleIndex, const std::list< std::list<unsigned> >::iterator& itChains)
{
    // if the two intersection points of the DownRay do not already exist:
    // add each intersection point and the intersected triangle index to the specific lists
    // and add the intersection point indexes at the begin/end of the current chain-list
    //
    // check the first point of the chain-list
    if ( !(Ogre::Math::RealEqual(firstTriangleIntersection.x, intersectionsWithinStartEndBothSides[(*itChains).front()].x, epsilon) &&
        Ogre::Math::RealEqual(firstTriangleIntersection.y, intersectionsWithinStartEndBothSides[(*itChains).front()].y, epsilon) &&
        Ogre::Math::RealEqual(firstTriangleIntersection.z, intersectionsWithinStartEndBothSides[(*itChains).front()].z, epsilon)) )
    {
        intersectionsWithinStartEndBothSides.push_back(firstTriangleIntersection);
        std::vector<unsigned> triIndexes;
        triIndexes.push_back(triangleIndexes[(*itChains).front()][firstPointTriangleIndex]);
        triangleIndexes.push_back(triIndexes);

        (*itChains).insert((*itChains).begin(), intersectionsWithinStartEndBothSides.size() - 1);
    }
    // check the last point of the chain-list
    if ( !(Ogre::Math::RealEqual(lastTriangleIntersection.x, intersectionsWithinStartEndBothSides[(*itChains).back()].x, epsilon) &&
        Ogre::Math::RealEqual(lastTriangleIntersection.y, intersectionsWithinStartEndBothSides[(*itChains).back()].y, epsilon) &&
        Ogre::Math::RealEqual(lastTriangleIntersection.z, intersectionsWithinStartEndBothSides[(*itChains).back()].z, epsilon)) )
    {
        intersectionsWithinStartEndBothSides.push_back(lastTriangleIntersection);
        std::vector<unsigned> triIndexes;
        triIndexes.push_back(triangleIndexes[(*itChains).back()][lastPointTriangleIndex]);
        triangleIndexes.push_back(triIndexes);

        (*itChains).insert((*itChains).end(), intersectionsWithinStartEndBothSides.size() - 1);
    }
}
