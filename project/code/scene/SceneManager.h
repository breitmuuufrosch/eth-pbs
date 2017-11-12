#pragma once

#include <osg/Group>
#include <osgViewer/Viewer>

#include "SpaceObject.h"

namespace pbs17
{
	/**
	 * \brief Load the objects for the scene and keep track of them.
	 */
	class SceneManager {
	public:
		/**
		 * \brief Constructor of the scene-manager.
		 */
		SceneManager();


		/**
		 * \brief Destructor of the scene-manager.
		 */
		virtual ~SceneManager();


		/**
		 * \brief Load the scene
		 * IMPORTANT: Add the "model" to the _spaceObjects for simulation-calculations and
		 *            and the "model->getModel()" to the scene for rendering.
		 *
		 * IMPORTANT: In a later step, we can define some file or parameters for creating the scene!
		 *            Just let me know if you have some ideas :-)
		 *
		 * \return Root-node of OSG for rendering.
		 */
		osg::ref_ptr<osg::Node> loadScene();


		/**
		 * \brief Init the viewer for the rendering window and set up the camera.
		 * IMPORTANT: Camera can be animated in a later step! Or it can follow also some objects (not yet tested)
		 *
		 * \param scene
		 *      Root-node of OSG which contains the objects.
		 *
		 * \return OSG-viewer which handles the rendering
		 */
		osg::ref_ptr<osgViewer::Viewer> initViewer(osg::ref_ptr<osg::Node> scene) const;


		/**
		 * \brief Get the loaded space-objects.
		 *
		 * \return All space-objects in the scene.
		 */
		std::vector<SpaceObject*> getSpaceObjects() const {
			return _spaceObjects;
		}

	private:

		//! Root-node of OSG which contains the whole scene (used for rendering)
		osg::ref_ptr<osg::Group> _scene;

		//! All space-objects in the scene (used for calcualtions)
		std::vector<SpaceObject*> _spaceObjects;
	};
}
