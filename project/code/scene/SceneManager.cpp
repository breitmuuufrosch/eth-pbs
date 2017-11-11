#include "SceneManager.h"

#include <osgGA/TrackballManipulator>
#include <osgViewer/Viewer>

using namespace pbs17;

/**
 * \brief Constructor of the scene-manager.
 */
SceneManager::SceneManager() {
}


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
osg::ref_ptr<osg::Node> SceneManager::loadScene() {
	osg::ref_ptr<osg::Group> scene = new osg::Group();
	SpaceObject* model = new SpaceObject("A2.obj", osg::Vec3(10.0, 0.0, 0.0), osg::Vec3(-10.0, 0.0, 0.0), 1.0);
	_spaceObjects.push_back(model);
	scene->addChild(model->getModel());

	SpaceObject* model2 = new SpaceObject("asteroid OBJ.obj", osg::Vec3(-10.0, 0.0, 0.0), osg::Vec3(10.0, 0.0, 0.0), 0.1);
	_spaceObjects.push_back(model2);
	scene->addChild(model2->getModel());

	return scene;
}


/**
 * \brief Init the viewer for the rendering window and set up the camera.
 * IMPORTANT: Camera can be animated in a later step! Or it can follow also some objects (not yet tested)
 * 
 * \param scene
 *      Root-node of OSG which contains the objects.
 * 
 * \return OSG-viewer which handles the rendering
 */
osg::ref_ptr<osgViewer::Viewer> SceneManager::initViewer(osg::ref_ptr<osg::Node> scene) const {
	osg::ref_ptr<osgViewer::Viewer> viewer = new osgViewer::Viewer;
	viewer->setUpViewOnSingleScreen(0);
	//viewer->setUpViewInWindow(80, 80, 1000, 600);

	viewer->setSceneData(scene);

	osg::ref_ptr<osgGA::TrackballManipulator> manipulator = new osgGA::TrackballManipulator;
	viewer->setCameraManipulator(manipulator);

	osg::Matrix rotation = osg::Matrix::rotate(-osg::PI / .6, osg::X_AXIS);
	osg::Matrix translation = osg::Matrix::translate(0.0f, 0.0f, 55.0f);

	manipulator->setByMatrix(translation * rotation);

	return viewer;
}
