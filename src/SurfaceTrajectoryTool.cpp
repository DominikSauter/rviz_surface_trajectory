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

#include "SurfaceTrajectoryTool.h"

namespace rviz_surface_trajectory
{


    //////////////
    // SETTINGS //
    //////////////


// Key bindings
const char toolShortcut = 't';
const int qtKeyCalcSurfProj = 0x51;         // Qt::Key_Q
const int qtKeyCalcSurfSurfCyc = 0x57;      // Qt::Key_W
const int qtKeyCalcSurfSurfNoCyc = 0x45;    // Qt::Key_E
const int qtKeyCalcOffsetProj = 0x59;       // Qt::Key_Y
const int qtKeyCalcOffsetSurfCyc = 0x58;    // Qt::Key_X
const int qtKeyCalcOffsetSurfNoCyc = 0x43;  // Qt::Key_C
const int qtKeyTestCalc = 0x55;             // Qt::Key_U
const int qtKeyToggleLines = 0x4c;          // Qt::Key_L
const int qtKeyToggleNormals = 0x4e;        // Qt::Key_N
const int qtKeyDeleteTrajPoints = 0x44;     // Qt::Key_D

// Input mesh
const std::string mesh_resource_ = "package://rviz_surface_trajectory/Data/Meshes/InputMeshes/Testing/T-RexSkull_Skull.stl";
// Helper meshes
const std::string traj_point_resource_ = "package://rviz_surface_trajectory/Data/Meshes/HelperMeshes/PointCuboidSmall.stl";
const std::string raycast_traj_point_resource_ = "package://rviz_surface_trajectory/Data/Meshes/HelperMeshes/PointCuboid.stl";
const std::string line_element_resource_ = "package://rviz_surface_trajectory/Data/Meshes/HelperMeshes/LineCuboid.stl";

// Default distance between consecutive points when drawing a point line
const Ogre::Real defaultStepWidth = 0.05f;

// Default visibility settings for lines and normals
const bool defaultShowLines = true;
const bool defaultShowNormals = true;

// Enable loading of predefined objects
const bool loadPredefPosMesh = true;
const bool loadPredefPosTrajPoints = true;

// Positions of predefined objects
const Ogre::Vector3 predefPosMesh = Ogre::Vector3(71.566, 3.3139, 9.6868e-06);//complex scene: Ogre::Vector3(66.642, -2.1812, 0);
const int predefPosTrajPointsLength = 2;
const Ogre::Vector3 predefPosTrajPoints[] = {

    // 4 startLists, 1 endList (2 points)
    Ogre::Vector3(1.3337e-06, 5.0806, -7.9547),
    Ogre::Vector3(1.6239e-06, -19.359, 2.354)


    /*
    // complex scene with errors (3 points)
    Ogre::Vector3(0, 8.2712, -9.5263),
    Ogre::Vector3(0, -3.4214, -3.921),
    Ogre::Vector3(0, 8.6734, -0.62642)
    */

    /*
    // KIT (23 points)
    // K
    Ogre::Vector3(0, 6.3076, 22.7),
    Ogre::Vector3(0, 9.4, 26.532),
    Ogre::Vector3(0, 9.4, 22.7),
    Ogre::Vector3(0, 11.0, 22.7),
    Ogre::Vector3(0, 11.0, 31.0),
    Ogre::Vector3(0, 9.4, 31.0),
    Ogre::Vector3(0, 9.4, 27.987),
    Ogre::Vector3(0, 6.3076, 31.0),
    Ogre::Vector3(0, 4.3, 31.0),
    Ogre::Vector3(0, 7.8200, 27.284),
    Ogre::Vector3(0, 4.3, 22.7),
    // I
    Ogre::Vector3(0, 1.6, 22.7),
    Ogre::Vector3(0, 1.6, 31.0),
    Ogre::Vector3(0, -0.5, 31.0),
    Ogre::Vector3(0, -0.5, 22.7),
    // T
    Ogre::Vector3(0, -6.2, 22.7),
    Ogre::Vector3(0, -6.2, 29.3),
    Ogre::Vector3(0, -3.2, 29.3),
    Ogre::Vector3(0, -3.2, 31.0),
    Ogre::Vector3(0, -11.4, 31.0),
    Ogre::Vector3(0, -11.4, 29.3),
    Ogre::Vector3(0, -8.4, 29.3),
    Ogre::Vector3(0, -8.4, 22.7)
    */

    /*
    // S (12 points)
    Ogre::Vector3(0,0.7,-0.3),
    Ogre::Vector3(0,0.3,-0.5),
    Ogre::Vector3(0,-0.2,-0.5),
    Ogre::Vector3(0,-0.5,-0.3),
    Ogre::Vector3(0,-0.4,0.1),
    Ogre::Vector3(0,0,0.2),
    Ogre::Vector3(0,0.4,0.3),
    Ogre::Vector3(0,0.7,0.5),
    Ogre::Vector3(0,0.6,0.8),
    Ogre::Vector3(0,0.3,1.0),
    Ogre::Vector3(0,-0.1,1.0),
    Ogre::Vector3(0,-0.5,0.9)
    */
    };

// The plane on which the trajectory points are placed
const Ogre::Plane trajPlane(Ogre::Vector3(0,0,0),
                      Ogre::Vector3(0,1,0),
                      Ogre::Vector3(0,0,1)
                      );



    //////////
    // CODE //
    //////////


SurfaceTrajectoryTool::SurfaceTrajectoryTool()
    : moving_node_(NULL)
    , moving_mesh_node_(NULL)
    , moving_traj_point_node_(NULL)
    , placed_mesh_node_(NULL)
    , ray_surf_traj_point_node_(NULL)
    , plane_surf_traj_point_node_(NULL)
    , traj_line_element_node_(NULL)
    , surf_line_element_node_(NULL)
    , traj_normal_element_node_(NULL)
    , surf_normal_element_node_(NULL)
    , current_property_(NULL)
    , current_mesh_property_(NULL)
    , current_traj_point_property_(NULL)
{
    shortcut_key_ = toolShortcut;
}

SurfaceTrajectoryTool::~SurfaceTrajectoryTool()
{
    for( unsigned i = 0; i < traj_point_nodes_.size(); i++ )
    {
        scene_manager_->destroySceneNode( traj_point_nodes_[ i ]);
    }
    for( unsigned i = 0; i < surf_traj_point_nodes_.size(); i++ )
    {
        scene_manager_->destroySceneNode( surf_traj_point_nodes_[ i ]);
    }
    scene_manager_->destroySceneNode(moving_mesh_node_);
    scene_manager_->destroySceneNode(moving_traj_point_node_);
    if (placed_mesh_node_)
    {
        scene_manager_->destroySceneNode(placed_mesh_node_);
    }
}

void SurfaceTrajectoryTool::onInitialize()
{
    if( rviz::loadMeshFromResource( mesh_resource_ ).isNull() )
    {
        ROS_ERROR( "SurfaceTrajectoryTool: failed to load model resource '%s'.", mesh_resource_.c_str() );
        return;
    }
    if( rviz::loadMeshFromResource( traj_point_resource_ ).isNull() )
    {
        ROS_ERROR( "SurfaceTrajectoryTool: failed to load model resource '%s'.", traj_point_resource_.c_str() );
        return;
    }
    if( rviz::loadMeshFromResource( raycast_traj_point_resource_ ).isNull() )
    {
        ROS_ERROR( "SurfaceTrajectoryTool: failed to load model resource '%s'.", raycast_traj_point_resource_.c_str() );
        return;
    }
    if( rviz::loadMeshFromResource( line_element_resource_ ).isNull() )
    {
        ROS_ERROR( "SurfaceTrajectoryTool: failed to load model resource '%s'.", line_element_resource_.c_str() );
        return;
    }

    moving_mesh_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
    Ogre::Entity* entity_mesh = scene_manager_->createEntity( mesh_resource_ );
    Ogre::MaterialPtr material_mesh = entity_mesh->getSubEntity(0)->getMaterial();
    material_mesh = Ogre::MaterialManager::getSingleton().create("MeshMaterial", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    material_mesh->setAmbient(Ogre::ColourValue(0.4f, 0.3f, 0.3f, 1.0f));
    material_mesh->setDiffuse(Ogre::ColourValue(0.4f, 0.3f, 0.3f, 1.0f));
    entity_mesh->setMaterialName("MeshMaterial");
    //material->getTechnique(0)->getPass(0)->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    //material->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);
    moving_mesh_node_->attachObject( entity_mesh );
    moving_mesh_node_->setVisible( false );

    moving_traj_point_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
    Ogre::Entity* entity_traj_point = scene_manager_->createEntity( traj_point_resource_ );
    Ogre::MaterialPtr material_traj_point = entity_traj_point->getSubEntity(0)->getMaterial();
    material_traj_point = Ogre::MaterialManager::getSingleton().create("TrajectionPointMaterial", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    material_traj_point->setAmbient(Ogre::ColourValue(0.0f, 0.2f, 1.0f, 1.0f));
    material_traj_point->setDiffuse(Ogre::ColourValue(0.0f, 0.2f, 1.0f, 1.0f));
    entity_traj_point->setMaterialName("TrajectionPointMaterial");
    moving_traj_point_node_->attachObject( entity_traj_point );
    moving_traj_point_node_->setVisible( false );


    ray_surf_traj_point_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
    Ogre::Entity* entity_ray_surf_traj_point = scene_manager_->createEntity( raycast_traj_point_resource_ );
    Ogre::MaterialPtr material_ray_surf_traj_point = entity_ray_surf_traj_point->getSubEntity(0)->getMaterial();
    material_ray_surf_traj_point = Ogre::MaterialManager::getSingleton().create("RaycastSurfaceTrajectionPointMaterial", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    material_ray_surf_traj_point->setAmbient(Ogre::ColourValue(0.0f, 1.0f, 0.3f, 1.0f));
    material_ray_surf_traj_point->setDiffuse(Ogre::ColourValue(0.0f, 1.0f, 0.3f, 1.0f));
    entity_ray_surf_traj_point->setMaterialName("RaycastSurfaceTrajectionPointMaterial");
    ray_surf_traj_point_node_->attachObject( entity_ray_surf_traj_point );
    ray_surf_traj_point_node_->setVisible( false );

    plane_surf_traj_point_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
    Ogre::Entity* entity_plane_surf_traj_point = scene_manager_->createEntity( traj_point_resource_ );
    Ogre::MaterialPtr material_plane_surf_traj_point = entity_plane_surf_traj_point->getSubEntity(0)->getMaterial();
    material_plane_surf_traj_point = Ogre::MaterialManager::getSingleton().create("PlaneCutSurfaceTrajectionPointMaterial", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    material_plane_surf_traj_point->setAmbient(Ogre::ColourValue(0.3f, 1.0f, 0.0f, 1.0f));
    material_plane_surf_traj_point->setDiffuse(Ogre::ColourValue(0.3f, 1.0f, 0.0f, 1.0f));
    entity_plane_surf_traj_point->setMaterialName("PlaneCutSurfaceTrajectionPointMaterial");
    plane_surf_traj_point_node_->attachObject( entity_plane_surf_traj_point );
    plane_surf_traj_point_node_->setVisible( false );


    traj_line_element_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
    Ogre::Entity* entity_traj_line_element = scene_manager_->createEntity( line_element_resource_ );
    Ogre::MaterialPtr material_traj_line_element = entity_traj_line_element->getSubEntity(0)->getMaterial();
    material_traj_line_element = Ogre::MaterialManager::getSingleton().create("TrajectoryLineElementMaterial", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    material_traj_line_element->setAmbient(Ogre::ColourValue(1.0f, 1.0f, 0.0f, 1.0f));
    material_traj_line_element->setDiffuse(Ogre::ColourValue(1.0f, 1.0f, 0.0f, 1.0f));
    entity_traj_line_element->setMaterialName("TrajectoryLineElementMaterial");
    traj_line_element_node_->attachObject( entity_traj_line_element );
    traj_line_element_node_->setVisible( false );

    surf_line_element_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
    Ogre::Entity* entity_surf_line_element = scene_manager_->createEntity( line_element_resource_ );
    Ogre::MaterialPtr material_surf_line_element = entity_surf_line_element->getSubEntity(0)->getMaterial();
    material_surf_line_element = Ogre::MaterialManager::getSingleton().create("SurfaceLineElementMaterial", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    material_surf_line_element->setAmbient(Ogre::ColourValue(1.0f, 1.0f, 0.0f, 1.0f));
    material_surf_line_element->setDiffuse(Ogre::ColourValue(1.0f, 1.0f, 0.0f, 1.0f));
    entity_surf_line_element->setMaterialName("SurfaceLineElementMaterial");
    surf_line_element_node_->attachObject( entity_surf_line_element );
    surf_line_element_node_->setVisible( false );


    traj_normal_element_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
    Ogre::Entity* entity_traj_normal_element = scene_manager_->createEntity( line_element_resource_ );
    Ogre::MaterialPtr material_traj_normal_element = entity_traj_normal_element->getSubEntity(0)->getMaterial();
    material_traj_normal_element = Ogre::MaterialManager::getSingleton().create("TrajectoryNormalElementMaterial", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    material_traj_normal_element->setAmbient(Ogre::ColourValue(1.0f, 0.0f, 0.0f, 1.0f));
    material_traj_normal_element->setDiffuse(Ogre::ColourValue(1.0f, 0.0f, 0.0f, 1.0f));
    entity_traj_normal_element->setMaterialName("TrajectoryNormalElementMaterial");
    traj_normal_element_node_->attachObject( entity_traj_normal_element );
    traj_normal_element_node_->setVisible( false );

    surf_normal_element_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
    Ogre::Entity* entity_surf_normal_element = scene_manager_->createEntity( line_element_resource_ );
    Ogre::MaterialPtr material_surf_normal_element = entity_surf_normal_element->getSubEntity(0)->getMaterial();
    material_surf_normal_element = Ogre::MaterialManager::getSingleton().create("SurfaceNormalElementMaterial", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    material_surf_normal_element->setAmbient(Ogre::ColourValue(1.0f, 0.0f, 0.0f, 1.0f));
    material_surf_normal_element->setDiffuse(Ogre::ColourValue(1.0f, 0.0f, 0.0f, 1.0f));
    entity_surf_normal_element->setMaterialName("SurfaceNormalElementMaterial");
    surf_normal_element_node_->attachObject( entity_surf_normal_element );
    surf_normal_element_node_->setVisible( false );


    moving_node_ = moving_mesh_node_;
    trajPointsCounter = 0;
    surfTrajPointsCounter = 0;
    raySurfTrajPointsCounter = 0;
    planeSurfTrajPointsCounter = 0;
    showLines = defaultShowLines;
    showNormals = defaultShowNormals;

    if (loadPredefPosMesh)
    {
        createProperty(moving_mesh_node_);
        current_property_->setVector( predefPosMesh );
        placeObject(predefPosMesh, moving_mesh_node_);
    }
    if (loadPredefPosTrajPoints)
    {
        for( int i = 0; i < predefPosTrajPointsLength; i++ )
        {
            createProperty(moving_traj_point_node_);
            current_property_->setVector( predefPosTrajPoints[i] );
            placeObject(predefPosTrajPoints[i], moving_traj_point_node_);
        }
    }
}

void SurfaceTrajectoryTool::activate()
{
    if( moving_node_ )
    {
        moving_node_->setVisible( true );

        createProperty(moving_node_);
    }
}

void SurfaceTrajectoryTool::deactivate()
{
    if( moving_node_ )
    {
        moving_node_->setVisible( false );

        // only delete the property referenced by current_property_ if the mesh has not been placed yet (as there will be a mesh-property created on every activate() call)
        // OR if it is not the mesh-property (to which current_mesh_property_ is pointing as soon as the mesh was placed)
        if ((!placed_mesh_node_) || (current_property_ != current_mesh_property_))
        {
            delete current_property_;

        // if the mesh has been placed and the current_property_ points to the mesh-property then store the position of the placed mesh in the mesh-property before dereferencing it
        // (so the placed mesh position is displayed again while deactivated)
        } else
        {
            current_property_->setVector( placed_mesh_node_->getPosition() );
        }

        current_property_ = NULL;
    }
}

int SurfaceTrajectoryTool::processMouseEvent(rviz::ViewportMouseEvent& event)
{
    if( !moving_node_ )
    {
        return Render;
    }

    Ogre::Vector3 intersection;
    Ogre::Plane ground_plane;
    if (moving_node_ == moving_mesh_node_)
    {
        ground_plane = Ogre::Plane(Ogre::Vector3(1,0,0), Ogre::Vector3(0,1,0), Ogre::Vector3(0,0,0));

    } else if (moving_node_ == moving_traj_point_node_)
    {
        ground_plane = trajPlane;
    }
    if( rviz::getPointOnPlaneFromWindowXY( event.viewport, ground_plane, event.x, event.y, intersection ))
    {
        moving_node_->setVisible( true );
        moving_node_->setPosition( intersection );
        current_property_->setVector( intersection );
        if( event.leftDown() )
        {
            placeObject( intersection, moving_node_ );
            return Render | Finished;
        }
    }
    else
    {
        moving_node_->setVisible( false ); // If the mouse is not pointing at the ground plane, don't show the mesh.
    }

    if (event.rightDown())
    {
        if (moving_node_ == moving_mesh_node_)
        {
            moving_node_->setVisible( false );
            moving_node_ = moving_traj_point_node_;

            // Set the mesh-property of the moving mesh back to the position of the placed mesh if it exists. Else delete the current property.
            if (placed_mesh_node_)
            {
                current_property_->setVector( placed_mesh_node_->getPosition() );
            } else
            {
                delete current_property_;
                current_property_ = NULL;
            }

            createProperty(moving_traj_point_node_);

        } else if (moving_node_ == moving_traj_point_node_)
        {
            moving_node_->setVisible( false );
            moving_node_ = moving_mesh_node_;

            delete current_property_;
            current_property_ = NULL;

            createProperty(moving_mesh_node_);
        }
    }
    return Render;
}

int SurfaceTrajectoryTool::processKeyEvent(QKeyEvent* event, rviz::RenderPanel* panel)
{
    // calculate and display surface trajectory with projection loyalty
    if (event->key() == qtKeyCalcSurfProj)
    {
        calcAndDisplaySurfTraj(false, true);

    // calculate and display surface trajectory with surface loyalty and cycle detection
    } else if (event->key() == qtKeyCalcSurfSurfCyc)
    {
        calcAndDisplaySurfTraj(true, false);

    // calculate and display surface trajectory with surface loyalty and no cycle detection
    } else if (event->key() == qtKeyCalcSurfSurfNoCyc)
    {
        calcAndDisplaySurfTraj(false, false);

    // calculate and display offset trajectory with projection loyalty
    } else if (event->key() == qtKeyCalcOffsetProj)
    {
        calcAndDisplayOffsetTraj(false, true);

    // calculate and display offset trajectory with surface loyalty and cycle detection
    } else if (event->key() == qtKeyCalcOffsetSurfCyc)
    {
        calcAndDisplayOffsetTraj(true, false);

    // calculate and display offset trajectory with surface loyalty and no cycle detection
    } else if (event->key() == qtKeyCalcOffsetSurfNoCyc)
    {
        calcAndDisplayOffsetTraj(false, false);

    // run performance tests
    } else if (event->key() == qtKeyTestCalc)
    {
        ///////////////////
        // TEST SETTINGS //
        ///////////////////

        std::vector<unsigned> patternSizes;
        patternSizes.push_back(2);
        patternSizes.push_back(10);
        patternSizes.push_back(40);
        patternSizes.push_back(70);
        patternSizes.push_back(100);
        patternSizes.push_back(1000);
        std::vector<unsigned> patternRepetitions;
        patternRepetitions.push_back(100);
        patternRepetitions.push_back(100);
        patternRepetitions.push_back(10);
        patternRepetitions.push_back(10);
        patternRepetitions.push_back(10);
        patternRepetitions.push_back(3);
        std::vector<unsigned> calculationRepetitionsForAveraging;
        calculationRepetitionsForAveraging.push_back(1);
        calculationRepetitionsForAveraging.push_back(1);
        calculationRepetitionsForAveraging.push_back(1);
        calculationRepetitionsForAveraging.push_back(1);
        calculationRepetitionsForAveraging.push_back(1);
        calculationRepetitionsForAveraging.push_back(1);

        Ogre::Real patternWidth = 20.0f;
        Ogre::Real patternHeight = 20.0f;
        Ogre::Real planeDistanceFromMeshCenter = 200.0f;


        /////////////////////////
        // AUTOMATIC TEST RUNS //
        /////////////////////////

        // seconds multiplicator to get nanoseconds
        unsigned long long billion = 1000000000;

        // for all patterns
        for (unsigned j = 0; j < patternSizes.size(); j++)
        {
            // create random pattern scheme (random points in a rectangular region)
            std::vector<Ogre::Vector2> patternRelativePointCoords;
            for (unsigned i = 0; i < patternSizes[j]; i++)
            {
                Ogre::Vector2 relCoords(Ogre::Math::RangeRandom(-(patternWidth / 2.0f), patternWidth / 2.0f), Ogre::Math::RangeRandom(-(patternHeight / 2.0f), patternHeight / 2.0f));
                patternRelativePointCoords.push_back(relCoords);
            }

            // variables used to accumulate the averaged times from different directions
            unsigned long long surfProjTimeDiffAveragedAccumlated = 0;
            unsigned long long surfSurfCycTimeDiffAveragedAccumulated = 0;
            unsigned long long surfSurfNoCycTimeDiffAveragedAccumulated = 0;
            unsigned long long offsetProjTimeDiffAveragedAccumulated = 0;
            unsigned long long offsetSurfCycTimeDiffAveragedAccumulated = 0;
            unsigned long long offsetSurfNoCycTimeDiffAveragedAccumulated = 0;
            // variables used to accumulate the number of points from different directions
            unsigned long long surfProjPointsAccumulated = 0;
            unsigned long long surfSurfCycPointsAccumulated = 0;
            unsigned long long surfSurfNoCycPointsAccumulated = 0;
            unsigned long long offsetProjPointsAccumulated = 0;
            unsigned long long offsetSurfCycPointsAccumulated = 0;
            unsigned long long offsetSurfNoCycPointsAccumulated = 0;
            // variable used to accumulate the number of raycast-points from different directions
            unsigned long long raycastPointsNumberAccumulated = 0;

            // repeat the test from different projection directions
            for (unsigned k = 0; k < patternRepetitions[j]; k++)
            {
                // create the projection plane for a random projection direction
                Ogre::Vector3 randomVec(Ogre::Math::SymmetricRandom(), Ogre::Math::SymmetricRandom(), Ogre::Math::SymmetricRandom());
                randomVec.normalise();
                Ogre::Vector3 offsettedPoint = (placed_mesh_node_->getPosition()).operator+( randomVec.operator*(planeDistanceFromMeshCenter) );
                Ogre::Plane randomTrajPlane(-randomVec, offsettedPoint);

                // create actual 3D points out of the 2D pattern scheme
                Ogre::Vector3 xDirection = (randomTrajPlane.normal).perpendicular();
                xDirection.normalise();
                Ogre::Vector3 yDirection = xDirection.crossProduct(randomTrajPlane.normal);
                yDirection.normalise();
                std::vector<Ogre::Vector3> patternAbsolutePointCoords;
                for (unsigned i = 0; i < patternRelativePointCoords.size(); i++)
                {
                    Ogre::Vector3 absCoords( (offsettedPoint.operator+(xDirection * patternRelativePointCoords[i].x)).operator+(yDirection * patternRelativePointCoords[i].y) );
                    patternAbsolutePointCoords.push_back(absCoords);
//placeObject(absCoords, ray_surf_traj_point_node_);
                }

//placeObject(placed_mesh_node_->getPosition(), surf_normal_element_node_);
//createStretchedLine(placed_mesh_node_->getPosition(), offsettedPoint, surf_normal_element_nodes_[surf_normal_element_nodes_.size() - 1]);



                // calculate surface and offset trajectories and measure the time

                SurfaceTrajectoryCalculation calculation(scene_manager_);
                const Ogre::Entity* meshEntity = (Ogre::Entity*)placed_mesh_node_->getAttachedObject(0);
                // set up needed variables for the surface trajectory
                std::list<Ogre::Vector3> surfacePositions;
                std::list< std::vector<Ogre::Vector3> > intersectedTriangles;
                std::vector<unsigned> surfaceRaycastIndex;
                // set up needed variables for the offset trajectory
                std::list<Ogre::Vector3> offsetPositions;
                std::vector<unsigned> offsetRaycastIndex;
                std::list< std::vector<Ogre::Vector3> > normals;
                std::list<Ogre::Radian> angles;
                // repeat the calculation several times, measure the time and average it afterwards
                unsigned long long surfProjTimeDiffAccumulated = 0;
                unsigned long long surfSurfCycTimeDiffAccumulated = 0;
                unsigned long long surfSurfNoCycTimeDiffAccumulated = 0;
                unsigned long long offsetProjTimeDiffAccumulated = 0;
                unsigned long long offsetSurfCycTimeDiffAccumulated = 0;
                unsigned long long offsetSurfNoCycTimeDiffAccumulated = 0;
                // store the number of points
                unsigned long long surfProjPoints = 0;
                unsigned long long surfSurfCycPoints = 0;
                unsigned long long surfSurfNoCycPoints = 0;
                unsigned long long offsetProjPoints = 0;
                unsigned long long offsetSurfCycPoints = 0;
                unsigned long long offsetSurfNoCycPoints = 0;
                // store the number of raycast-points
                unsigned long long raycastPointsNumber = 0;

                timespec time1;
                timespec time2;
                for (unsigned l = 0; l < calculationRepetitionsForAveraging[j]; l++)
                {
                    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time1);
                    calculation.calcSurfaceTrajectoryProjOrSurfLoyal(patternAbsolutePointCoords, randomTrajPlane, meshEntity, false, true, surfacePositions, intersectedTriangles, surfaceRaycastIndex);
                    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time2);
                    surfProjTimeDiffAccumulated += (diff(time1,time2).tv_sec * billion) + diff(time1,time2).tv_nsec;
                    surfProjPoints = surfacePositions.size();
                    raycastPointsNumber = surfaceRaycastIndex.size();
printf("raycast-points: %llu\n", raycastPointsNumber);
printf("[pSize:%u][pRepetition:%u/%u][cRepetition:%u/%u] surfProj: %llu ns / %llu points\n", patternSizes[j], k, patternRepetitions[j] - 1, l, calculationRepetitionsForAveraging[j] - 1, (diff(time1,time2).tv_sec * billion) + diff(time1,time2).tv_nsec, surfProjPoints);

                    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time1);
                    calculation.calcSurfaceTrajectoryProjOrSurfLoyal(patternAbsolutePointCoords, randomTrajPlane, meshEntity, true, false, surfacePositions, intersectedTriangles, surfaceRaycastIndex);
                    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time2);
                    surfSurfCycTimeDiffAccumulated += (diff(time1,time2).tv_sec * billion) + diff(time1,time2).tv_nsec;
                    surfSurfCycPoints = surfacePositions.size();
printf("[pSize:%u][pRepetition:%u/%u][cRepetition:%u/%u] surfSurfCyc: %llu ns / %llu points\n", patternSizes[j], k, patternRepetitions[j] - 1, l, calculationRepetitionsForAveraging[j] - 1, (diff(time1,time2).tv_sec * billion) + diff(time1,time2).tv_nsec, surfSurfCycPoints);

/*
                    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time1);
                    calculation.calcSurfaceTrajectoryProjOrSurfLoyal(patternAbsolutePointCoords, randomTrajPlane, meshEntity, false, false, surfacePositions, intersectedTriangles, surfaceRaycastIndex);
                    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time2);
                    surfSurfNoCycTimeDiffAccumulated += (diff(time1,time2).tv_sec * billion) + diff(time1,time2).tv_nsec;
                    surfSurfNoCycPoints = surfacePositions.size();
printf("[pSize:%u][pRepetition:%u/%u][cRepetition:%u/%u] surfSurfNoCyc: %llu ns / %llu points\n", patternSizes[j], k, patternRepetitions[j] - 1, l, calculationRepetitionsForAveraging[j] - 1, (diff(time1,time2).tv_sec * billion) + diff(time1,time2).tv_nsec, surfSurfNoCycPoints);
*/

                    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time1);
                    calculation.calcOffsetTrajectoryProjOrSurfLoyal(patternAbsolutePointCoords, randomTrajPlane, meshEntity, false, true, offsetPositions, offsetRaycastIndex, normals, angles);
                    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time2);
                    offsetProjTimeDiffAccumulated += (diff(time1,time2).tv_sec * billion) + diff(time1,time2).tv_nsec;
                    offsetProjPoints = offsetPositions.size();
printf("[pSize:%u][pRepetition:%u/%u][cRepetition:%u/%u] offsetProj: %llu ns / %llu points\n", patternSizes[j], k, patternRepetitions[j] - 1, l, calculationRepetitionsForAveraging[j] - 1, (diff(time1,time2).tv_sec * billion) + diff(time1,time2).tv_nsec, offsetProjPoints);

                    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time1);
                    calculation.calcOffsetTrajectoryProjOrSurfLoyal(patternAbsolutePointCoords, randomTrajPlane, meshEntity, true, false, offsetPositions, offsetRaycastIndex, normals, angles);
                    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time2);
                    offsetSurfCycTimeDiffAccumulated += (diff(time1,time2).tv_sec * billion) + diff(time1,time2).tv_nsec;
                    offsetSurfCycPoints = offsetPositions.size();
printf("[pSize:%u][pRepetition:%u/%u][cRepetition:%u/%u] offsetSurfCyc: %llu ns / %llu points\n", patternSizes[j], k, patternRepetitions[j] - 1, l, calculationRepetitionsForAveraging[j] - 1, (diff(time1,time2).tv_sec * billion) + diff(time1,time2).tv_nsec, offsetSurfCycPoints);

/*
                    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time1);
                    calculation.calcOffsetTrajectoryProjOrSurfLoyal(patternAbsolutePointCoords, randomTrajPlane, meshEntity, false, false, offsetPositions, offsetRaycastIndex, normals, angles);
                    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time2);
                    offsetSurfNoCycTimeDiffAccumulated += (diff(time1,time2).tv_sec * billion) + diff(time1,time2).tv_nsec;
                    offsetSurfNoCycPoints = offsetPositions.size();
printf("[pSize:%u][pRepetition:%u/%u][cRepetition:%u/%u] offsetSurfNoCyc: %llu ns / %llu points\n", patternSizes[j], k, patternRepetitions[j] - 1, l, calculationRepetitionsForAveraging[j] - 1, (diff(time1,time2).tv_sec * billion) + diff(time1,time2).tv_nsec, offsetSurfNoCycPoints);
*/
                }

                // accumulate averaged times
                surfProjTimeDiffAveragedAccumlated += surfProjTimeDiffAccumulated / calculationRepetitionsForAveraging[j];
                surfSurfCycTimeDiffAveragedAccumulated += surfSurfCycTimeDiffAccumulated / calculationRepetitionsForAveraging[j];
                //surfSurfNoCycTimeDiffAveragedAccumulated += surfSurfNoCycTimeDiffAccumulated / calculationRepetitionsForAveraging[j];
                offsetProjTimeDiffAveragedAccumulated += offsetProjTimeDiffAccumulated / calculationRepetitionsForAveraging[j];
                offsetSurfCycTimeDiffAveragedAccumulated += offsetSurfCycTimeDiffAccumulated / calculationRepetitionsForAveraging[j];
                //offsetSurfNoCycTimeDiffAveragedAccumulated += offsetSurfNoCycTimeDiffAccumulated / calculationRepetitionsForAveraging[j];

                // accumulate number of points
                surfProjPointsAccumulated += surfProjPoints;
                surfSurfCycPointsAccumulated += surfSurfCycPoints;
                //surfSurfNoCycPointsAccumulated += surfSurfNoCycPoints;
                offsetProjPointsAccumulated += offsetProjPoints;
                offsetSurfCycPointsAccumulated += offsetSurfCycPoints;
                //offsetSurfNoCycPointsAccumulated += offsetSurfNoCycPoints;

                // accumulate number of raycast-points
                raycastPointsNumberAccumulated += raycastPointsNumber;



/*
// calculate averaged normals
std::list<Ogre::Vector3> normals;
Ogre::Vector3 normal(0,0,0);
std::list< std::vector<Ogre::Vector3> >::iterator it;
for (it=intersectedTriangles.begin(); it!=intersectedTriangles.end(); ++it)
{
    normal = Ogre::Vector3(0,0,0);
    for (unsigned j = 0; j < (*it).size(); j += 3)
    {
        normal = normal.operator+( Ogre::Math::calculateBasicFaceNormalWithoutNormalize((*it)[j], (*it)[j+1], (*it)[j+2]) );
    }
    normal.normalise();
    normals.push_back(normal);
}

// visualize the surface trajectory data
std::list<Ogre::Vector3>::iterator itPos;
std::list<Ogre::Vector3>::iterator itNorm;
unsigned pointsCounter = 0;
unsigned indexCounter = 0;
for (itPos=surfacePositions.begin(), itNorm=normals.begin(); itPos!=surfacePositions.end(); ++itPos, ++itNorm, pointsCounter++)
{
    if ( !(surfaceRaycastIndex[indexCounter] == pointsCounter) )
    {
        plane_surf_traj_point_node_->setPosition(*itPos);
        createProperty(plane_surf_traj_point_node_);
        placeObject(*itPos, plane_surf_traj_point_node_);

        // create normal
        placeObject(*itPos, surf_normal_element_node_);
        createStretchedLine(*itPos, (*itPos).operator+(*itNorm), surf_normal_element_nodes_[surf_normal_element_nodes_.size() - 1]);
    } else
    {
        ray_surf_traj_point_node_->setPosition(*itPos);
        createProperty(ray_surf_traj_point_node_);
        placeObject(*itPos, ray_surf_traj_point_node_);

        // create normal
        placeObject(*itPos, surf_normal_element_node_);
        createStretchedLine(*itPos, (*itPos).operator+(*itNorm), surf_normal_element_nodes_[surf_normal_element_nodes_.size() - 1]);
        indexCounter++;
    }
}
*/

/*
// visualize the offset trajectory data
std::list<Ogre::Vector3>::iterator itPos;
std::list< std::vector<Ogre::Vector3> >::iterator itNorm;
unsigned pointsCounter = 0;
unsigned indexCounter = 0;
for (itPos=offsetPositions.begin(), itNorm=normals.begin(); itPos!=offsetPositions.end(); ++itPos, ++itNorm, pointsCounter++)
{
    if ( !(offsetRaycastIndex[indexCounter] == pointsCounter) )
    {
        plane_surf_traj_point_node_->setPosition(*itPos);
        createProperty(plane_surf_traj_point_node_);
        placeObject(*itPos, plane_surf_traj_point_node_);

        // create normals
        for (unsigned i = 0; i < (*itNorm).size(); i++)
        {
            placeObject(*itPos, surf_normal_element_node_);
            createStretchedLine(*itPos, (*itPos).operator+((*itNorm)[i]), surf_normal_element_nodes_[surf_normal_element_nodes_.size() - 1]);
        }
    } else
    {
        ray_surf_traj_point_node_->setPosition(*itPos);
        createProperty(ray_surf_traj_point_node_);
        placeObject(*itPos, ray_surf_traj_point_node_);

        // create normals
        for (unsigned i = 0; i < (*itNorm).size(); i++)
        {
            placeObject(*itPos, surf_normal_element_node_);
            createStretchedLine(*itPos, (*itPos).operator+((*itNorm)[i]), surf_normal_element_nodes_[surf_normal_element_nodes_.size() - 1]);
        }
        indexCounter++;
    }
}
*/


            }

            // get the average time of all directions
            unsigned long long surfProjTimeDiffAveragedAccumlatedAveraged = surfProjTimeDiffAveragedAccumlated / patternRepetitions[j];
            unsigned long long surfSurfCycTimeDiffAveragedAccumulatedAveraged = surfSurfCycTimeDiffAveragedAccumulated / patternRepetitions[j];
            //unsigned long long surfSurfNoCycTimeDiffAveragedAccumulatedAveraged = surfSurfNoCycTimeDiffAveragedAccumulated / patternRepetitions[j];
            unsigned long long offsetProjTimeDiffAveragedAccumulatedAveraged = offsetProjTimeDiffAveragedAccumulated / patternRepetitions[j];
            unsigned long long offsetSurfCycTimeDiffAveragedAccumulatedAveraged = offsetSurfCycTimeDiffAveragedAccumulated / patternRepetitions[j];
            //unsigned long long offsetSurfNoCycTimeDiffAveragedAccumulatedAveraged = offsetSurfNoCycTimeDiffAveragedAccumulated / patternRepetitions[j];


            // get the average number of points of all directions
            float surfProjPointsAccumulatedAveraged = surfProjPointsAccumulated / (float)patternRepetitions[j];
            float surfSurfCycPointsAccumulatedAveraged = surfSurfCycPointsAccumulated / (float)patternRepetitions[j];
            //float surfSurfNoCycPointsAccumulatedAveraged = surfSurfNoCycPointsAccumulated / (float)patternRepetitions[j];
            float offsetProjPointsAccumulatedAveraged = offsetProjPointsAccumulated / (float)patternRepetitions[j];
            float offsetSurfCycPointsAccumulatedAveraged = offsetSurfCycPointsAccumulated / (float)patternRepetitions[j];
            //float offsetSurfNoCycPointsAccumulatedAveraged = offsetSurfNoCycPointsAccumulated / (float)patternRepetitions[j];


            // get the average number of raycast-points of all directions
            float raycastPointsNumberAccumulatedAveraged = raycastPointsNumberAccumulated / (float)patternRepetitions[j];


printf("-------------------------------------------------------------------------------------\n");
printf("raycastPointsNumberAccumulatedAveraged: %f raycast-points\n", raycastPointsNumberAccumulatedAveraged);
printf("surfProjTimeDiffAveragedAccumlatedAveraged: %llu ns / %f points\n", surfProjTimeDiffAveragedAccumlatedAveraged, surfProjPointsAccumulatedAveraged);
printf("surfSurfCycTimeDiffAveragedAccumulatedAveraged: %llu ns / %f points\n", surfSurfCycTimeDiffAveragedAccumulatedAveraged, surfSurfCycPointsAccumulatedAveraged);
//printf("surfSurfNoCycTimeDiffAveragedAccumulatedAveraged : %llu ns / %f points\n", surfSurfNoCycTimeDiffAveragedAccumulatedAveraged, surfSurfNoCycPointsAccumulatedAveraged);
printf("offsetProjTimeDiffAveragedAccumulatedAveraged: %llu ns / %f points\n", offsetProjTimeDiffAveragedAccumulatedAveraged, offsetProjPointsAccumulatedAveraged);
printf("offsetSurfCycTimeDiffAveragedAccumulatedAveraged: %llu ns / %f points\n", offsetSurfCycTimeDiffAveragedAccumulatedAveraged, offsetSurfCycPointsAccumulatedAveraged);
//printf("offsetSurfNoCycTimeDiffAveragedAccumulatedAveraged: %llu ns / %f points\n", offsetSurfNoCycTimeDiffAveragedAccumulatedAveraged, offsetSurfNoCycPointsAccumulatedAveraged);
printf("-------------------------------------------------------------------------------------\n");


            // write the data to the output file in a new line
            std::ofstream testsFile("PerformanceTests.txt", std::ios::out | std::ios::app);
            if (testsFile.is_open())
            {
                testsFile
                << (surfProjTimeDiffAveragedAccumlatedAveraged / (float)billion) << " " << surfProjPointsAccumulatedAveraged << " "
                << (surfSurfCycTimeDiffAveragedAccumulatedAveraged / (float)billion) << " " << surfSurfCycPointsAccumulatedAveraged << " "
                << (offsetProjTimeDiffAveragedAccumulatedAveraged / (float)billion) << " " << offsetProjPointsAccumulatedAveraged << " "
                << (offsetSurfCycTimeDiffAveragedAccumulatedAveraged / (float)billion) << " " << offsetSurfCycPointsAccumulatedAveraged << " "
                << patternSizes[j] << " " << patternRepetitions[j] << " " << calculationRepetitionsForAveraging[j] << " " << raycastPointsNumberAccumulatedAveraged << std::endl;
                testsFile.close();
            } else
            {
                std::cout << "Unable to open file";
            }


        }


    // toggle lines
    } else if (event->key() == qtKeyToggleLines)
    {
        for (unsigned i = 0; i < traj_line_element_nodes_.size(); i++)
        {
            traj_line_element_nodes_[i]->setVisible( !showLines );
        }
        for (unsigned i = 0; i < surf_line_element_nodes_.size(); i++)
        {
            surf_line_element_nodes_[i]->setVisible( !showLines );
        }
        showLines = !showLines;

    // toggle normals
    } else if (event->key() == qtKeyToggleNormals)
    {
        for (unsigned i = 0; i < traj_normal_element_nodes_.size(); i++)
        {
            traj_normal_element_nodes_[i]->setVisible( !showNormals );
        }
        for (unsigned i = 0; i < surf_normal_element_nodes_.size(); i++)
        {
            surf_normal_element_nodes_[i]->setVisible( !showNormals );
        }
        showNormals = !showNormals;

    // delete trajectory points
    } else if (event->key() == qtKeyDeleteTrajPoints)
    {
        for (unsigned i = 0; i < traj_point_nodes_.size(); i++)
        {
            scene_manager_->destroySceneNode(traj_point_nodes_[i]);
        }
        traj_point_nodes_.clear();
        for (unsigned i = 0; i < traj_line_element_nodes_.size(); i++)
        {
            scene_manager_->destroySceneNode(traj_line_element_nodes_[i]);
        }
        traj_line_element_nodes_.clear();
        for (unsigned i = 0; i < traj_normal_element_nodes_.size(); i++)
        {
            scene_manager_->destroySceneNode(traj_normal_element_nodes_[i]);
        }
        traj_normal_element_nodes_.clear();
        for (unsigned i = 0; i < trajPointsCounter; i++)
        {
            getPropertyContainer()->takeChild( getPropertyContainer()->subProp("Trajectory point " + QString::number(i)) );
        }
        trajPointsCounter = 0;
        if (current_property_ == current_traj_point_property_)
        {
            getPropertyContainer()->takeChild(current_property_);
            createProperty(moving_traj_point_node_);
        }
    }
}

const timespec SurfaceTrajectoryTool::diff(const timespec& start, const timespec& end)
{
	timespec temp;
	if ((end.tv_nsec-start.tv_nsec)<0) {
		temp.tv_sec = end.tv_sec-start.tv_sec-1;
		temp.tv_nsec = 1000000000+end.tv_nsec-start.tv_nsec;
	} else {
		temp.tv_sec = end.tv_sec-start.tv_sec;
		temp.tv_nsec = end.tv_nsec-start.tv_nsec;
	}
	return temp;
}

void SurfaceTrajectoryTool::calcAndDisplaySurfTraj(const bool& detectCycles, const bool& projOrSurfLoyal)
{
    // delete old data
    for( unsigned i = 0; i < surf_traj_point_nodes_.size(); i++ )
    {
        scene_manager_->destroySceneNode(surf_traj_point_nodes_[ i ]);
    }
    surf_traj_point_nodes_.clear();
    for (unsigned i = 0; i < surf_line_element_nodes_.size(); i++)
    {
        scene_manager_->destroySceneNode(surf_line_element_nodes_[i]);
    }
    surf_line_element_nodes_.clear();
    for (unsigned i = 0; i < surf_normal_element_nodes_.size(); i++)
    {
        scene_manager_->destroySceneNode(surf_normal_element_nodes_[i]);
    }
    surf_normal_element_nodes_.clear();
    for (unsigned i = 0; i < surf_traj_point_props_.size(); i++)
    {
        getPropertyContainer()->takeChild(surf_traj_point_props_[i]);
    }
    surf_traj_point_props_.clear();
    surfTrajPointsCounter = 0;
    raySurfTrajPointsCounter = 0;
    planeSurfTrajPointsCounter = 0;



    // set up needed variables and calculate surface trajectory
    const Ogre::Entity* meshEntity = (Ogre::Entity*)placed_mesh_node_->getAttachedObject(0);
    std::vector<Ogre::Vector3> trajPoints;
    std::list<Ogre::Vector3> positions;
    std::list< std::vector<Ogre::Vector3> > intersectedTriangles;
    std::vector<unsigned> raycastIndex;
    for (int i = 0; i < traj_point_nodes_.size(); i++)
    {
        trajPoints.push_back(traj_point_nodes_[i]->getPosition());
    }
    SurfaceTrajectoryCalculation calculation(scene_manager_);
    calculation.calcSurfaceTrajectoryProjOrSurfLoyal(trajPoints, trajPlane, meshEntity, detectCycles, projOrSurfLoyal, positions, intersectedTriangles, raycastIndex);


    // calculate averaged normals
    std::list<Ogre::Vector3> normals;
    Ogre::Vector3 normal(0,0,0);
    std::list< std::vector<Ogre::Vector3> >::iterator it;
    for (it=intersectedTriangles.begin(); it!=intersectedTriangles.end(); ++it)
    {
        normal = Ogre::Vector3(0,0,0);
        for (unsigned j = 0; j < (*it).size(); j += 3)
        {
            normal = normal.operator+( Ogre::Math::calculateBasicFaceNormalWithoutNormalize((*it)[j], (*it)[j+1], (*it)[j+2]) );
        }
        normal.normalise();
        normals.push_back(normal);
    }

    // visualize the surface trajectory data
    std::list<Ogre::Vector3>::iterator itPos;
    std::list<Ogre::Vector3>::iterator itNorm;
    unsigned pointsCounter = 0;
    unsigned indexCounter = 0;
    for (itPos=positions.begin(), itNorm=normals.begin(); itPos!=positions.end(); ++itPos, ++itNorm, pointsCounter++)
    {
        if ( !(raycastIndex[indexCounter] == pointsCounter) )
        {
            plane_surf_traj_point_node_->setPosition(*itPos);
            createProperty(plane_surf_traj_point_node_);
            placeObject(*itPos, plane_surf_traj_point_node_);

            // create normal
            placeObject(*itPos, surf_normal_element_node_);
            createStretchedLine(*itPos, (*itPos).operator+(*itNorm), surf_normal_element_nodes_[surf_normal_element_nodes_.size() - 1]);
        } else
        {
            ray_surf_traj_point_node_->setPosition(*itPos);
            createProperty(ray_surf_traj_point_node_);
            placeObject(*itPos, ray_surf_traj_point_node_);

            // create normal
            placeObject(*itPos, surf_normal_element_node_);
            createStretchedLine(*itPos, (*itPos).operator+(*itNorm), surf_normal_element_nodes_[surf_normal_element_nodes_.size() - 1]);
            indexCounter++;
        }
    }
}

void SurfaceTrajectoryTool::calcAndDisplayOffsetTraj(const bool& detectCycles, const bool& projOrSurfLoyal)
{
    // delete old data
    for( unsigned i = 0; i < surf_traj_point_nodes_.size(); i++ )
    {
        scene_manager_->destroySceneNode(surf_traj_point_nodes_[ i ]);
    }
    surf_traj_point_nodes_.clear();
    for (unsigned i = 0; i < surf_line_element_nodes_.size(); i++)
    {
        scene_manager_->destroySceneNode(surf_line_element_nodes_[i]);
    }
    surf_line_element_nodes_.clear();
    for (unsigned i = 0; i < surf_normal_element_nodes_.size(); i++)
    {
        scene_manager_->destroySceneNode(surf_normal_element_nodes_[i]);
    }
    surf_normal_element_nodes_.clear();
    for (unsigned i = 0; i < surf_traj_point_props_.size(); i++)
    {
        getPropertyContainer()->takeChild(surf_traj_point_props_[i]);
    }
    surf_traj_point_props_.clear();
    surfTrajPointsCounter = 0;
    raySurfTrajPointsCounter = 0;
    planeSurfTrajPointsCounter = 0;



    // set up needed variables and calculate offset trajectory
    const Ogre::Entity* meshEntity = (Ogre::Entity*)placed_mesh_node_->getAttachedObject(0);
    std::vector<Ogre::Vector3> trajPoints;
    std::list<Ogre::Vector3> positions;
    std::vector<unsigned> raycastIndex;
    for (int i = 0; i < traj_point_nodes_.size(); i++)
    {
        trajPoints.push_back(traj_point_nodes_[i]->getPosition());
    }
    SurfaceTrajectoryCalculation calculation(scene_manager_);

    std::list< std::vector<Ogre::Vector3> > normals;
    std::list<Ogre::Radian> angles;
    calculation.calcOffsetTrajectoryProjOrSurfLoyal(trajPoints, trajPlane, meshEntity, detectCycles, projOrSurfLoyal, positions, raycastIndex, normals, angles);



std::list<Ogre::Radian>::iterator itAngles;
for (itAngles = angles.begin(); itAngles != angles.end(); ++itAngles)
{
printf("Angle: %f\n", (*itAngles).valueDegrees());
}


    // visualize the offset trajectory data
    std::list<Ogre::Vector3>::iterator itPos;
    std::list< std::vector<Ogre::Vector3> >::iterator itNorm;
    unsigned pointsCounter = 0;
    unsigned indexCounter = 0;
    for (itPos=positions.begin(), itNorm=normals.begin(); itPos!=positions.end(); ++itPos, ++itNorm, pointsCounter++)
    {
        if ( !(raycastIndex[indexCounter] == pointsCounter) )
        {
            plane_surf_traj_point_node_->setPosition(*itPos);
            createProperty(plane_surf_traj_point_node_);
            placeObject(*itPos, plane_surf_traj_point_node_);

            // create normals
            for (unsigned i = 0; i < (*itNorm).size(); i++)
            {
                placeObject(*itPos, surf_normal_element_node_);
                createStretchedLine(*itPos, (*itPos).operator+((*itNorm)[i]), surf_normal_element_nodes_[surf_normal_element_nodes_.size() - 1]);
            }
        } else
        {
            ray_surf_traj_point_node_->setPosition(*itPos);
            createProperty(ray_surf_traj_point_node_);
            placeObject(*itPos, ray_surf_traj_point_node_);

            // create normals
            for (unsigned i = 0; i < (*itNorm).size(); i++)
            {
                placeObject(*itPos, surf_normal_element_node_);
                createStretchedLine(*itPos, (*itPos).operator+((*itNorm)[i]), surf_normal_element_nodes_[surf_normal_element_nodes_.size() - 1]);
            }
            indexCounter++;
        }
    }
}

// This is a helper function to create a new object in the Ogre scene and save its scene node in a list.
void SurfaceTrajectoryTool::placeObject(const Ogre::Vector3& position, Ogre::SceneNode*& object)
{
    Ogre::SceneNode* node;
    Ogre::Entity* entity;
    if (object == moving_mesh_node_)
    {
        if (placed_mesh_node_)
        {
            placed_mesh_node_->setPosition( position );
        }
        else
        {
            placed_mesh_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode("PlacedMesh");
            entity = scene_manager_->createEntity( mesh_resource_ );
            entity->setMaterialName("MeshMaterial");
            placed_mesh_node_->attachObject( entity );
            placed_mesh_node_->setVisible( true );
            placed_mesh_node_->setPosition( position );
        }
        current_property_ = NULL; // Drop the reference so that deactivate() won't remove it.
    }
    else if (object == moving_traj_point_node_)
    {
        node = scene_manager_->getRootSceneNode()->createChildSceneNode("TrajectoryPoint" + Ogre::StringConverter::toString(trajPointsCounter));
        entity = scene_manager_->createEntity( traj_point_resource_ );
        entity->setMaterialName("TrajectionPointMaterial");
        node->attachObject( entity );
        node->setVisible( true );
        node->setPosition( position );
        traj_point_nodes_.push_back( node );

        // create normal
        placeObject(position, traj_normal_element_node_);
        createStretchedLine(position, position.operator+(trajPlane.normal), traj_normal_element_nodes_[traj_normal_element_nodes_.size() - 1]);

        // create connection line
        if (trajPointsCounter > 0)
        {
            placeObject(position, traj_line_element_node_);
            createStretchedLine(position,
                                scene_manager_->getRootSceneNode()->getChild("TrajectoryPoint" + Ogre::StringConverter::toString(trajPointsCounter-1))->getPosition(),
                                traj_line_element_nodes_[traj_line_element_nodes_.size() - 1]);
        }
        trajPointsCounter++;

        current_property_ = NULL; // Drop the reference so that deactivate() won't remove it.
    }
    else if (object == ray_surf_traj_point_node_)
    {
        node = scene_manager_->getRootSceneNode()->createChildSceneNode("RaycastSurfaceTrajectoryPoint" + Ogre::StringConverter::toString(raySurfTrajPointsCounter));
        entity = scene_manager_->createEntity( raycast_traj_point_resource_ );
        entity->setMaterialName("RaycastSurfaceTrajectionPointMaterial");
        node->attachObject( entity );
        node->setVisible( true );
        node->setPosition( position );
        surf_traj_point_nodes_.push_back( node );

        // create connection line
        if (surfTrajPointsCounter > 0)
        {
            placeObject(position, surf_line_element_node_);
            createStretchedLine(position, lastSurfTrajPoint, surf_line_element_nodes_[surf_line_element_nodes_.size() - 1]);
        }
        raySurfTrajPointsCounter++;
        surfTrajPointsCounter++;
        lastSurfTrajPoint = position;
    }
    else if (object == plane_surf_traj_point_node_)
    {
        node = scene_manager_->getRootSceneNode()->createChildSceneNode("PlaneCutSurfaceTrajectoryPoint" + Ogre::StringConverter::toString(planeSurfTrajPointsCounter));
        entity = scene_manager_->createEntity( traj_point_resource_ );
        entity->setMaterialName("PlaneCutSurfaceTrajectionPointMaterial");
        node->attachObject( entity );
        node->setVisible( true );
        node->setPosition( position );
        surf_traj_point_nodes_.push_back( node );

        // create connection line
        if (surfTrajPointsCounter > 0)
        {
            placeObject(position, surf_line_element_node_);
            createStretchedLine(position, lastSurfTrajPoint, surf_line_element_nodes_[surf_line_element_nodes_.size() - 1]);
        }
        planeSurfTrajPointsCounter++;
        surfTrajPointsCounter++;
        lastSurfTrajPoint = position;
    }
    else if (object == traj_line_element_node_)
    {
        node = scene_manager_->getRootSceneNode()->createChildSceneNode();
        entity = scene_manager_->createEntity( line_element_resource_ );
        entity->setMaterialName("TrajectoryLineElementMaterial");
        node->attachObject( entity );
        node->setVisible( true );
        node->setPosition( position );
        line_element_nodes_.push_back(node);
        traj_line_element_nodes_.push_back( node );
    }
    else if (object == surf_line_element_node_)
    {
        node = scene_manager_->getRootSceneNode()->createChildSceneNode();
        entity = scene_manager_->createEntity( line_element_resource_ );
        entity->setMaterialName("SurfaceLineElementMaterial");
        node->attachObject( entity );
        node->setVisible( true );
        node->setPosition( position );
        line_element_nodes_.push_back(node);
        surf_line_element_nodes_.push_back( node );
    }
    else if (object == traj_normal_element_node_)
    {
        node = scene_manager_->getRootSceneNode()->createChildSceneNode();
        entity = scene_manager_->createEntity( line_element_resource_ );
        entity->setMaterialName("TrajectoryNormalElementMaterial");
        node->attachObject( entity );
        node->setVisible( true );
        node->setPosition( position );
        normal_element_nodes_.push_back(node);
        traj_normal_element_nodes_.push_back( node );
    }
    else if (object == surf_normal_element_node_)
    {
        node = scene_manager_->getRootSceneNode()->createChildSceneNode();
        entity = scene_manager_->createEntity( line_element_resource_ );
        entity->setMaterialName("SurfaceNormalElementMaterial");
        node->attachObject( entity );
        node->setVisible( true );
        node->setPosition( position );
        normal_element_nodes_.push_back(node);
        surf_normal_element_nodes_.push_back( node );
    }
}

// Creates a new property for the current object, if it has not already been created, and adds it as child to the PropertyContainer.
void SurfaceTrajectoryTool::createProperty(Ogre::SceneNode*& object)
{
    if (object == moving_mesh_node_)
    {
        if (!placed_mesh_node_)
        {
            current_mesh_property_ = new rviz::VectorProperty( "Mesh" );
            current_mesh_property_->setReadOnly( true );
            getPropertyContainer()->addChild( current_mesh_property_ );
        }
        current_property_ = current_mesh_property_;

    } else if (object == moving_traj_point_node_)
    {
        current_traj_point_property_ = new rviz::VectorProperty( "Trajectory point " + QString::number( traj_point_nodes_.size() ) );
        current_traj_point_property_->setReadOnly( true );
        getPropertyContainer()->addChild( current_traj_point_property_ );

        current_property_ = current_traj_point_property_;
    } else if (object == ray_surf_traj_point_node_)
    {
        rviz::VectorProperty* property = new rviz::VectorProperty( "Surf. traj. point " + QString::number( surfTrajPointsCounter )
                                                                  + " (Raycast " + QString::number(raySurfTrajPointsCounter) + ")" );
        property->setVector( ray_surf_traj_point_node_->getPosition() );
        property->setReadOnly( true );
        getPropertyContainer()->addChild( property );
        surf_traj_point_props_.push_back(property);
    } else if (object == plane_surf_traj_point_node_)
    {
        rviz::VectorProperty* property = new rviz::VectorProperty( "Surf. traj. point " + QString::number( surfTrajPointsCounter )
                                                                  + " (Plane-cut " + QString::number(planeSurfTrajPointsCounter) + ")" );
        property->setVector( plane_surf_traj_point_node_->getPosition() );
        property->setReadOnly( true );
        getPropertyContainer()->addChild( property );
        surf_traj_point_props_.push_back(property);
    }
}

void SurfaceTrajectoryTool::createStretchedLine(const Ogre::Vector3& start, const Ogre::Vector3& end, Ogre::SceneNode*& lineElement)
{
    Ogre::Real meshScaleFactorToOne = 28.0f;

    Ogre::Real distance = start.distance(end);
    lineElement->setScale(distance * meshScaleFactorToOne,1.0f,1.0f);

    Ogre::Vector3 startEndHalf = (start.operator+(end)) / 2.0f;
    lineElement->setPosition(startEndHalf);

    Ogre::Vector3 src = lineElement->getOrientation() * Ogre::Vector3::UNIT_X;
    Ogre::Vector3 direction = end.operator-(start);
    direction.normalise();
    Ogre::Quaternion quat = src.getRotationTo(direction);
    lineElement->rotate(quat);
}

// Creates the visualisation of a line as small consecutive points.
void SurfaceTrajectoryTool::createPointLine(const Ogre::Vector3& start, const Ogre::Vector3& end, const Ogre::Real& stepWidth, Ogre::SceneNode*& object)
{
    Ogre::Real distance = start.distance(end);
    Ogre::Vector3 direction = end.operator-(start);
    direction.normalise();
    direction *= stepWidth;
    Ogre::Real steps = distance / stepWidth;
    if (object == traj_line_element_node_)
    {
        for (Ogre::Real i = 0; i < steps; i++)
        {
            placeObject( start.operator+(direction * i), traj_line_element_node_ );
        }
    } else if (object == surf_line_element_node_)
    {
        for (Ogre::Real i = 0; i < steps; i++)
        {
            placeObject( start.operator+(direction * i), surf_line_element_node_ );
        }
    } else if (object == traj_normal_element_node_)
    {
        for (Ogre::Real i = 0; i < steps; i++)
        {
            placeObject( start.operator+(direction * i), traj_normal_element_node_ );
        }
    } else if (object == surf_normal_element_node_)
    {
        for (Ogre::Real i = 0; i < steps; i++)
        {
            placeObject( start.operator+(direction * i), surf_normal_element_node_ );
        }
    }
}

} // end namespace rviz_surface_trajectory

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_surface_trajectory::SurfaceTrajectoryTool,rviz::Tool)
