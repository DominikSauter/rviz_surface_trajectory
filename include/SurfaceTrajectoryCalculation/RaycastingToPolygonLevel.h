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

#ifndef RAYCASTINGTOPOLYGONLEVEL_H_INCLUDED
#define RAYCASTINGTOPOLYGONLEVEL_H_INCLUDED


#include <OgreSceneManager.h>
#include <OgreEntity.h>
#include <OgreManualObject.h>
#include <OgreSubMesh.h>
#include <OgreSubEntity.h>

/***************************************************************************//*!
* @brief Ray Collision Detection
* Usage: (Example with FPS view & cursor style (minecraft, cs, ...))
* @code
* //...
* Ogre::SceneManager* sceneMgr = ...
* //...
* RaycastingToPolygonLevel ray(sceneMgr);
* //...
*
*   Ogre::Vector3 resultPos;
*   std::vector<Ogre::Vector3> resultTris;
* 	if( ray.raycastFromPoint(m_camera->getPosition(), Ogre::Vector3 direction = m_camera->getDirection(), resultPos, resultTris) ){
*		printf("Your mouse is over the position %f,%f,%f (witch is textured)\n", result.x, result.y, result.z);
*	}else{
*		printf("No mouse collision\n Are you looking the sky ?\n");
* 	}
* @endcode
*/
class RaycastingToPolygonLevel
{
	public:
		RaycastingToPolygonLevel(Ogre::SceneManager*& sceneMgr);
		void cleanUp();
		void preloadMeshInformation(const Ogre::Entity*& entity);
		void deletePreloadedMeshInformation();
		void getPreloadedMeshInformation(size_t& vertex_count, const Ogre::Vector3*& vertices, size_t& index_count, const unsigned long*& indices, const Ogre::Entity*& entity);
		const bool raycastFromPointPreloaded(const Ogre::Vector3& point, const Ogre::Vector3& normal, const bool& positiveSide, const bool& negativeSide, Ogre::Vector3& resultPosition, std::vector<Ogre::Vector3>& resultTriangles);
		const bool raycastFromPoint(const Ogre::Vector3& point, const Ogre::Vector3& normal, const std::vector<Ogre::Entity*>& whiteList, Ogre::Vector3& resultPosition, std::vector<Ogre::Vector3>& resultTriangles);
        const bool raycastFromPoint(const Ogre::Vector3& point, const Ogre::Vector3& normal, const std::vector<Ogre::ManualObject*>& whiteList, Ogre::Vector3& resultPosition, std::vector<Ogre::Vector3>& resultTriangles);
		const bool raycastFromPoint(const Ogre::Vector3& point, const Ogre::Vector3& normal, Ogre::Vector3& resultPosition, std::vector<Ogre::Vector3>& resultTriangles);
	private:
        static void getMeshInformation(const Ogre::Entity*& entity, size_t& vertex_count, Ogre::Vector3*& vertices, size_t& index_count, unsigned long*& indices, const Ogre::Vector3& position, const Ogre::Quaternion& orient, const Ogre::Vector3& scale);
		static void getMeshInformation(const Ogre::ManualObject*& manual, size_t& vertex_count, Ogre::Vector3*& vertices, size_t& index_count, unsigned long*& indices, const Ogre::Vector3& position, const Ogre::Quaternion& orient, const Ogre::Vector3& scale);
        static void getMeshInformation(const Ogre::MeshPtr& mesh, size_t &vertex_count, Ogre::Vector3*& vertices,  size_t& index_count, unsigned long*& indices, const Ogre::Vector3& position, const Ogre::Quaternion& orient, const Ogre::Vector3& scale);

        Ogre::SceneManager* sceneManager;       // used for destroying the ray scene query
        Ogre::RaySceneQuery* m_raySceneQuery;   //!< Ray query

		// information used for the preloaded raycast version
        size_t vertex_count_;
        size_t index_count_;
        Ogre::Vector3* vertices_;
        unsigned long* indices_;
        const Ogre::Entity* entity_;
};

#endif // RAYCASTINGTOPOLYGONLEVEL_H_INCLUDED
