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

#include "SurfaceTrajectoryCalculation/SurfaceTrajectoryCalculation.h"

// epsilon value for checking equality of floating point numbers (Ogre::Real)
const Ogre::Real epsilon = std::numeric_limits< Ogre::Real >::epsilon() * 10;


SurfaceTrajectoryCalculation::SurfaceTrajectoryCalculation(Ogre::SceneManager*& sceneManager)
: raycasting(RaycastingToPolygonLevel(sceneManager))
{
}

SurfaceTrajectoryCalculation::~SurfaceTrajectoryCalculation()
{
    // free allocated resources
    raycasting.cleanUp();
}


void SurfaceTrajectoryCalculation::calcSurfaceTrajectoryProjOrSurfLoyal(std::vector<Ogre::Vector3>& trajectoryPoints, const Ogre::Plane& trajectoryPlane, const Ogre::Entity*& entity, const bool& detectCycles, const bool& projOrSurfLoyal, std::list<Ogre::Vector3>& resultPositions, std::list< std::vector<Ogre::Vector3> >& resultIntersectedTriangles, std::vector<unsigned>& resultRaycastIndex)
{
    raycasting.preloadMeshInformation(entity);
    Ogre::Vector3 raycastPoint;
    std::vector<Ogre::Vector3> raycastTris;
    std::vector<Ogre::Vector3> raycastPoints;
    std::vector< std::vector<Ogre::Vector3> > raycastIntersectedTris;
    for (unsigned i = 0; i < trajectoryPoints.size(); i++)
    {
        if ( raycasting.raycastFromPointPreloaded(trajectoryPoints[i], trajectoryPlane.normal, true, false, raycastPoint, raycastTris) )
        {
            raycastPoints.push_back(raycastPoint);
            raycastIntersectedTris.push_back(raycastTris);
        } else
        {
            printf("Warning: No mesh-intersection for trajectory point: (%f, %f, %f)! It will be ignored.\n", trajectoryPoints[i].x, trajectoryPoints[i].y, trajectoryPoints[i].z);
            trajectoryPoints.erase(trajectoryPoints.begin() + i);
            i--;
        }
    }


    std::list<Ogre::Vector3> surfacePositions;
    std::list< std::vector<Ogre::Vector3> > surfaceIntersectedTriangles;
    std::vector<unsigned> raycastIndex;

    PlaneCut planeCut(raycasting, trajectoryPlane);

    // if there is more than one raycast-point
    if (raycastPoints.size() >= 2)
    {
        std::list<Ogre::Vector3> positions;
        std::list< std::vector<Ogre::Vector3> > tris;

        // calculate the surface trajectory with projection or surface loyal algorithm
        if (projOrSurfLoyal)
        {
            // first plane-cut result will be inserted with both start- and end-entry into the output lists
            planeCut.cutWithPlaneProjLoyal(trajectoryPoints[0], trajectoryPoints[1], raycastPoints[0], raycastPoints[1], raycastIntersectedTris[0], raycastIntersectedTris[1], positions, tris);
            surfacePositions.insert(surfacePositions.end(), positions.begin(), positions.end());
            surfaceIntersectedTriangles.insert(surfaceIntersectedTriangles.end(), tris.begin(), tris.end());
            raycastIndex.push_back(0);
            raycastIndex.push_back(surfacePositions.size() - 1);

            // further plane-cut results will be appended to the output lists without the first entry to avoid duplication
            for (unsigned i = 1; i < (raycastPoints.size() - 1); i++)
            {
                planeCut.cutWithPlaneProjLoyal(trajectoryPoints[i], trajectoryPoints[i+1], raycastPoints[i], raycastPoints[i+1], raycastIntersectedTris[i], raycastIntersectedTris[i+1], positions, tris);
                surfacePositions.insert(surfacePositions.end(), ++(positions.begin()), positions.end());
                surfaceIntersectedTriangles.insert(surfaceIntersectedTriangles.end(), ++(tris.begin()), tris.end());
                raycastIndex.push_back(surfacePositions.size() - 1);
            }
        } else
        {
            // first plane-cut result will be inserted with both start- and end-entry into the output lists
            planeCut.cutWithPlaneSurfLoyal(trajectoryPoints[0], trajectoryPoints[1], raycastPoints[0], raycastPoints[1], raycastIntersectedTris[0], raycastIntersectedTris[1], detectCycles, positions, tris);
            surfacePositions.insert(surfacePositions.end(), positions.begin(), positions.end());
            surfaceIntersectedTriangles.insert(surfaceIntersectedTriangles.end(), tris.begin(), tris.end());
            raycastIndex.push_back(0);
            raycastIndex.push_back(surfacePositions.size() - 1);

            // further plane-cut results will be appended to the output lists without the first entry to avoid duplication
            for (unsigned i = 1; i < (raycastPoints.size() - 1); i++)
            {
                planeCut.cutWithPlaneSurfLoyal(trajectoryPoints[i], trajectoryPoints[i+1], raycastPoints[i], raycastPoints[i+1], raycastIntersectedTris[i], raycastIntersectedTris[i+1], detectCycles, positions, tris);
                surfacePositions.insert(surfacePositions.end(), ++(positions.begin()), positions.end());
                surfaceIntersectedTriangles.insert(surfaceIntersectedTriangles.end(), ++(tris.begin()), tris.end());
                raycastIndex.push_back(surfacePositions.size() - 1);
            }
        }

    // else if there is only one raycast-point
    } else if (raycastPoints.size() == 1)
    {
        surfacePositions.push_back(raycastPoints[0]);
        surfaceIntersectedTriangles.push_back(raycastIntersectedTris[0]);
        raycastIndex.push_back(0);
    }

    resultPositions = surfacePositions;
    resultIntersectedTriangles = surfaceIntersectedTriangles;
    resultRaycastIndex = raycastIndex;

    // free memory
    raycasting.deletePreloadedMeshInformation();
}

void SurfaceTrajectoryCalculation::calcOffsetTrajectoryProjOrSurfLoyal(std::vector<Ogre::Vector3>& trajectoryPoints, const Ogre::Plane& trajectoryPlane, const Ogre::Entity*& entity, const bool& detectCycles, const bool& projOrSurfLoyal, std::list<Ogre::Vector3>& resultPositions, std::vector<unsigned>& resultRaycastIndex, std::list< std::vector<Ogre::Vector3> >& resultNormals, std::list<Ogre::Radian>& resultAngles)
{
    // get surface trajectory data
    std::list<Ogre::Vector3> positions;
    std::list< std::vector<Ogre::Vector3> > intersectedTriangles;
    std::vector<unsigned> raycastIndex;
    calcSurfaceTrajectoryProjOrSurfLoyal(trajectoryPoints, trajectoryPlane, entity, detectCycles, projOrSurfLoyal, positions, intersectedTriangles, raycastIndex);

    std::list< std::vector<Ogre::Vector3> > offsetNormals;
    std::list<Ogre::Radian> angles;
    // if there are no surface trajectory points then return empty lists
    if (!positions.empty())
    {

        // calculate normals
        std::list< std::vector<Ogre::Vector3> > normals;
        std::list< std::vector<Ogre::Vector3> >::iterator itTris;
        for (itTris = intersectedTriangles.begin(); itTris != intersectedTriangles.end(); ++itTris)
        {
            std::vector<Ogre::Vector3> norms;
            for (unsigned i = 0; i < (*itTris).size(); i += 3)
            {
                norms.push_back( Ogre::Math::calculateBasicFaceNormal((*itTris)[i], (*itTris)[i+1], (*itTris)[i+2]) );
            }
            normals.push_back(norms);
        }

        // calculate offset trajectory data
        Ogre::Vector3 lastNormal = normals.front()[0];
        std::list< std::vector<Ogre::Vector3> >::iterator itNormsCurrent;
        std::list< std::vector<Ogre::Vector3> >::iterator itNormsNext;
        std::list< std::vector<Ogre::Vector3> >::iterator itNormsNextNext;
        std::list<Ogre::Vector3>::iterator itPos;
        unsigned pointsCounter = 0;
        unsigned indexCounter = 0;
        std::vector<unsigned> raycastIndexBias;
        raycastIndexBias.push_back(0);
        for (itNormsCurrent = normals.begin(), itNormsNext = ++(normals.begin()), itNormsNextNext = ++(normals.begin()), itPos = positions.begin(); itNormsCurrent != --(normals.end()); ++itNormsCurrent, ++itNormsNext, ++itNormsNextNext, ++itPos, pointsCounter++)
        {
            // if the current point is a raycast-point then add a new entry to the bias vector
            if (raycastIndex[indexCounter] == pointsCounter)
            {
                raycastIndexBias.push_back(0);
                indexCounter++;
            }

            // if the current point has only one normal then duplicate it so the calculation can continue regularly
            if ((*itNormsCurrent).size() == 1)
            {
                (*itNormsCurrent).push_back((*itNormsCurrent)[0]);
            }

            // check if there is a normal of the current point which equals a normal of the next point (meaning that this normal has to be the second normal in the offset description of the current point)
            bool normalsEqual = false;
            for (unsigned i = 0; i < (*itNormsCurrent).size() && !normalsEqual; i++)
            {
                for (unsigned j = 0; j < (*itNormsNext).size() && !normalsEqual; j++)
                {
                    if (Ogre::Math::RealEqual((*itNormsCurrent)[i].x, (*itNormsNext)[j].x, epsilon) &&
                        Ogre::Math::RealEqual((*itNormsCurrent)[i].y, (*itNormsNext)[j].y, epsilon) &&
                        Ogre::Math::RealEqual((*itNormsCurrent)[i].z, (*itNormsNext)[j].z, epsilon))
                    {
                        std::vector<Ogre::Vector3> norms;
                        norms.push_back(lastNormal);
                        norms.push_back((*itNormsCurrent)[i]);
                        offsetNormals.push_back(norms);

                        angles.push_back( offsetNormals.back()[0].angleBetween(offsetNormals.back()[1]) );

                        lastNormal = offsetNormals.back()[1];
                        normalsEqual = true;
                    }
                }
            }

            // if there is no equal normal of the current and the next point: create additional intermediate point
            if (!normalsEqual)
            {
                Ogre::Vector3 firstNormalOfAdditionalPoint;
                Ogre::Vector3 secondNormalOfAdditionalPoint;

                // get the two normals for the current point and set the first normal for the additional point (which is the second normal of the current point)

                std::vector<Ogre::Vector3> norms;
                norms.push_back(lastNormal);
                offsetNormals.push_back(norms);
                // choose as second normal for the current point the normal which does not equal the first normal (lastNormal) if possible
                if ( !(Ogre::Math::RealEqual((*itNormsCurrent)[0].x, lastNormal.x, epsilon) &&
                    Ogre::Math::RealEqual((*itNormsCurrent)[0].y, lastNormal.y, epsilon) &&
                    Ogre::Math::RealEqual((*itNormsCurrent)[0].z, lastNormal.z, epsilon)) )
                {
                    (offsetNormals.back()).push_back((*itNormsCurrent)[0]);
                    angles.push_back( offsetNormals.back()[0].angleBetween(offsetNormals.back()[1]) );
                    firstNormalOfAdditionalPoint = offsetNormals.back()[1];
                } else
                {
                    (offsetNormals.back()).push_back((*itNormsCurrent)[1]);
                    angles.push_back( offsetNormals.back()[0].angleBetween(offsetNormals.back()[1]) );
                    firstNormalOfAdditionalPoint = offsetNormals.back()[1];
                }

                // get the second normal of the additional point and set lastNormal to that normal (so the calculation after the additional point can just continue regularly)

                // if the next point is not the last point of the trajectory
                if (itNormsNext != --(normals.end()))
                {
                    // increment iterator so it actually points to the next element of the next element
                    ++itNormsNextNext;
                    bool normalsEqual = false;
                    // choose as second normal for the additional point a normal of the next point, that is not equal to a normal of the next next point
                    for (unsigned i = 0; i < (*itNormsNext).size() && !normalsEqual; i++)
                    {
                        for (unsigned j = 0; j < (*itNormsNextNext).size() && !normalsEqual; j++)
                        {
                            if (Ogre::Math::RealEqual((*itNormsNext)[i].x, (*itNormsNextNext)[j].x, epsilon) &&
                                Ogre::Math::RealEqual((*itNormsNext)[i].y, (*itNormsNextNext)[j].y, epsilon) &&
                                Ogre::Math::RealEqual((*itNormsNext)[i].z, (*itNormsNextNext)[j].z, epsilon))
                            {
                                if ( (i == 0) && ((*itNormsNext).size() > 1) )
                                {
                                    secondNormalOfAdditionalPoint = (*itNormsNext)[1];
                                    lastNormal = (*itNormsNext)[1];
                                } else
                                {
                                    secondNormalOfAdditionalPoint = (*itNormsNext)[0];
                                    lastNormal = (*itNormsNext)[0];
                                }
                                normalsEqual = true;
                            }
                        }
                    }
                    // if there was no equal normal between the next and next next point (which means the trajectory between them also runs through air) then pick the first normal of the next point as default because it does not matter
                    if (!normalsEqual)
                    {
                        secondNormalOfAdditionalPoint = (*itNormsNext)[0];
                        lastNormal = (*itNormsNext)[0];
                    }
                // decrement again to avoid out of bounds error in the for loop
                --itNormsNextNext;

                // if the next point is the last point of the trajectory then pick the first normal of the next point as default because it does not matter
                } else
                {
                    secondNormalOfAdditionalPoint = (*itNormsNext)[0];
                    lastNormal = (*itNormsNext)[0];
                }


                // insert data for the additional point

                Ogre::Vector3 firstPoint = *itPos;
                ++itPos;
                Ogre::Vector3 secondPoint = *itPos;
                Ogre::Vector3 additionalPoint = (firstPoint.operator+(secondPoint)) / 2.0f;
                positions.insert(itPos, additionalPoint);
                --itPos;

                std::vector<Ogre::Vector3> norms_;
                norms_.push_back(firstNormalOfAdditionalPoint);
                norms_.push_back(secondNormalOfAdditionalPoint);
                offsetNormals.push_back(norms_);

                angles.push_back( offsetNormals.back()[0].angleBetween(offsetNormals.back()[1]) );

                // increment the offset for the next raycast-points by one
                (raycastIndexBias.back())++;
            }

        }

        // insert data for the last point (raycast-point)
        std::vector<Ogre::Vector3> norms;
        norms.push_back(lastNormal);
        norms.push_back(lastNormal);
        offsetNormals.push_back(norms);

        angles.push_back( offsetNormals.back()[0].angleBetween(offsetNormals.back()[1]) );

        // set the first normal of the first point to its second normal, so both normals are equal like in the last point, and then also update angle
        offsetNormals.front()[0] = offsetNormals.front()[1];
        angles.front() = offsetNormals.front()[0].angleBetween(offsetNormals.front()[1]);


        // add the offset to the raycast indices so they are correct again
        for (unsigned i = 1; i < raycastIndex.size(); i++)
        {
            // accumulate the offset of the previous points until the current point
            raycastIndexBias[i] += raycastIndexBias[i - 1];
            // apply the accumulated offset to the current point
            raycastIndex[i] += raycastIndexBias[i];
        }

    }

    resultPositions = positions;
    resultRaycastIndex = raycastIndex;
    resultNormals = offsetNormals;
    resultAngles = angles;
}
