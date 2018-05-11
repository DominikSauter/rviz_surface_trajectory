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

#ifndef SURFACETRAJECTORYTOOL_H_INCLUDED
#define SURFACETRAJECTORYTOOL_H_INCLUDED


#include <OgrePrerequisites.h>
#include <OgreVector3.h>
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreEntity.h>
#include <OgreSubEntity.h>
#include <OgreMaterialManager.h>
#include <OgreQuaternion.h>

#include <ros/console.h>

#include <rviz/tool.h>
#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>
#include <rviz/mesh_loader.h>
#include <rviz/geometry.h>
#include <rviz/properties/vector_property.h>

#include <time.h>

#include "SurfaceTrajectoryCalculation.h"

namespace Ogre
{
class SceneNode;
class Vector3;
}
namespace rviz
{
class VectorProperty;
class VisualizationManager;
class ViewportMouseEvent;
}
namespace rviz_surface_trajectory
{

class SurfaceTrajectoryTool: public rviz::Tool
{
Q_OBJECT
public:
    SurfaceTrajectoryTool();
    ~SurfaceTrajectoryTool();
    virtual void onInitialize();
    virtual void activate();
    virtual void deactivate();
    virtual int processMouseEvent(rviz::ViewportMouseEvent& event);
    virtual int processKeyEvent(QKeyEvent* event, rviz::RenderPanel* panel);
private:
    const timespec diff(const timespec& start, const timespec& end);
    void calcAndDisplaySurfTraj(const bool& detectCycles, const bool& projOrSurfLoyal);
    void calcAndDisplayOffsetTraj(const bool& detectCycles, const bool& projOrSurfLoyal);
    void placeObject(const Ogre::Vector3& position, Ogre::SceneNode*& object);
    void createProperty(Ogre::SceneNode*& object);
    void createStretchedLine(const Ogre::Vector3& start, const Ogre::Vector3& end, Ogre::SceneNode*& lineElement);
    void createPointLine(const Ogre::Vector3& start, const Ogre::Vector3& end, const Ogre::Real& stepWidth, Ogre::SceneNode*& object);
    Ogre::SceneNode* moving_node_;
    Ogre::SceneNode* moving_mesh_node_;
    Ogre::SceneNode* moving_traj_point_node_;
    Ogre::SceneNode* placed_mesh_node_;
    Ogre::SceneNode* ray_surf_traj_point_node_;
    Ogre::SceneNode* plane_surf_traj_point_node_;
    Ogre::SceneNode* traj_line_element_node_;
    Ogre::SceneNode* surf_line_element_node_;
    Ogre::SceneNode* traj_normal_element_node_;
    Ogre::SceneNode* surf_normal_element_node_;
    Ogre::Vector3 lastSurfTrajPoint;
    std::vector<Ogre::SceneNode*> traj_point_nodes_;
    std::vector<Ogre::SceneNode*> line_element_nodes_;
    std::vector<Ogre::SceneNode*> traj_line_element_nodes_;
    std::vector<Ogre::SceneNode*> surf_line_element_nodes_;
    std::vector<Ogre::SceneNode*> normal_element_nodes_;
    std::vector<Ogre::SceneNode*> traj_normal_element_nodes_;
    std::vector<Ogre::SceneNode*> surf_normal_element_nodes_;
    std::vector<Ogre::SceneNode*> surf_traj_point_nodes_;
    std::vector<rviz::Property*> surf_traj_point_props_;
    rviz::VectorProperty* current_property_;
    rviz::VectorProperty* current_mesh_property_;
    rviz::VectorProperty* current_traj_point_property_;
    int trajPointsCounter;
    int surfTrajPointsCounter;
    int raySurfTrajPointsCounter;
    int planeSurfTrajPointsCounter;
    bool showLines;
    bool showNormals;
};

} // end namespace rviz_surface_trajectory

#endif // SURFACETRAJECTORYTOOL_H_INCLUDED
