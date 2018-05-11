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

#ifndef SURFACETRAJECTORYCALCULATION_H_INCLUDED
#define SURFACETRAJECTORYCALCULATION_H_INCLUDED


#include "RaycastingToPolygonLevel.h"

#include "PlaneCut.h"

class SurfaceTrajectoryCalculation
{
    public:
        SurfaceTrajectoryCalculation(Ogre::SceneManager*& sceneManager);
        ~SurfaceTrajectoryCalculation();
        void calcSurfaceTrajectoryProjOrSurfLoyal(std::vector<Ogre::Vector3>& trajectoryPoints, const Ogre::Plane& trajectoryPlane, const Ogre::Entity*& entity, const bool& detectCycles, const bool& projOrSurfLoyal, std::list<Ogre::Vector3>& resultPositions, std::list< std::vector<Ogre::Vector3> >& resultIntersectedTriangles, std::vector<unsigned>& resultRaycastIndex);
        void calcOffsetTrajectoryProjOrSurfLoyal(std::vector<Ogre::Vector3>& trajectoryPoints, const Ogre::Plane& trajectoryPlane, const Ogre::Entity*& entity, const bool& detectCycles, const bool& projOrSurfLoyal, std::list<Ogre::Vector3>& resultPositions, std::vector<unsigned>& resultRaycastIndex, std::list< std::vector<Ogre::Vector3> >& resultNormals, std::list<Ogre::Radian>& resultAngles);
    private:
        RaycastingToPolygonLevel raycasting;
};

#endif // SURFACETRAJECTORYCALCULATION_H_INCLUDED
