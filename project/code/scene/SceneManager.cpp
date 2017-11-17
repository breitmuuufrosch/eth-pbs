#include "SceneManager.h"

#include <math.h>
#include <iostream>
#include <osg/TexGen>
#include <osg/ShapeDrawable>
#include <osgGA/TrackballManipulator>
#include <osgViewer/Viewer>
#include <osgUtil/Optimizer>

#include "Asteroid.h"
#include "Planet.h"
#include "../osg/SkyBox.h"
#include "../config.h"

using namespace pbs17;

/**
 * \brief Constructor of the scene-manager.
 */
SceneManager::SceneManager() {
}


/**
 * \brief Destructor of the scene-manager.
 */
SceneManager::~SceneManager() {
	if (_spaceObjects.size() > 0) {
		for (std::vector<SpaceObject*>::iterator it = _spaceObjects.begin(); it != _spaceObjects.end(); ++it) {
			delete *it;
		}

		_spaceObjects.resize(0);
	}

	if (_scene) {
		_scene = nullptr;
	}
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
	_scene = new osg::Group();

	// -------------------------------------------------------------------------
	// Sky-box
	// -------------------------------------------------------------------------

	osg::ref_ptr<osg::Drawable> skyDrawable = new osg::ShapeDrawable;
	skyDrawable->setShape(new osg::Sphere(osg::Vec3(), 10));

	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	geode->addDrawable(skyDrawable);

	osg::ref_ptr<SkyBox> skybox = new SkyBox;
	skybox->getOrCreateStateSet()->setTextureAttributeAndModes(0, new osg::TexGen);
	skybox->init(0, DATA_PATH + "/skyBox/2");
	skybox->addChild(geode);

	osg::ref_ptr<osg::MatrixTransform> skyTransform = new osg::MatrixTransform;
	skyTransform->setMatrix(osg::Matrix::rotate(osg::PI / 2.0, osg::Z_AXIS) * osg::Matrix::rotate(- osg::PI / 5.0, osg::X_AXIS));
	skyTransform->addChild(skybox);

	_scene->addChild(skyTransform);


	// -------------------------------------------------------------------------
	// Planets
	// -------------------------------------------------------------------------

	osg::ref_ptr<osg::Group> planets = new osg::Group;
	_scene->addChild(planets);

	// SUN
	SpaceObject* planetSun = new Planet(5.0, "sunmap.jpg");
	planetSun->initOsg(Eigen::Vector3d(0, 0, 0.0), 1.0, 1.0);
	planetSun->initPhysics(10, Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0));
	_spaceObjects.push_back(planetSun);
	planets->addChild(planetSun->getModel());


	// EARTH
	SpaceObject* planetEarth = new Planet(2.0, "earthmap1k.jpg");
	planetEarth->initOsg(Eigen::Vector3d(10.0, 0.0, 6.5), 1.0, 1.0);
	planetEarth->initPhysics(1, Eigen::Vector3d(-4.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0));
	_spaceObjects.push_back(planetEarth);
	planets->addChild(planetEarth->getModel());

	// MARS
	/*SpaceObject* planet3 = new Planet(2.0, Eigen::Vector3d(10.0, 0.0, 0.0), "mars_1k_color.jpg");
	planet3->initOsg(Eigen::Vector3d(10.0, 10.0, 0.0), 1.0, 1.0);
	planet3->initPhysics(63.9, Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0));
	_spaceObjects.push_back(planet3);
	planets->addChild(planet3->getModel());*/

    int numObjects = 5;
	double rad = 2.0 * osg::PI / numObjects;
	
	for (int i = 0; i < numObjects; ++i) {
        SpaceObject* asteroid1 = new Asteroid("A2.obj");
		asteroid1->initOsg(Eigen::Vector3d(-10.0 * sin(i * rad), -10.0 * cos(i * rad), 0.0), 1.0, (i+1.0) / (numObjects));
		asteroid1->initPhysics(5.972, Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 1.0), Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0));
		_spaceObjects.push_back(asteroid1);
		planets->addChild(asteroid1->getModel());

		//SpaceObject* asteroid2 = new Asteroid("asteroid OBJ.obj");
		//asteroid2->initOsg(Eigen::Vector3d(-10.0 * sin(i * rad) + 10.0, -10.0 * cos(i * rad), 0.0), 1.0, 0.1);
		//_spaceObjects.push_back(asteroid2);
		//planets->addChild(asteroid2->getModel());

		//SpaceObject* planet1 = new Planet(2.0);
		//planet1->initOsg(Eigen::Vector3d(-10.0 * sin(i * rad), -10.0 * cos(i * rad), 0.0), 1.0, 1.0);
		//planet1->initPhysics(5.972, Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 1.0), Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0));
		//_spaceObjects.push_back(planet1);
		//planets->addChild(planet1->getModel());

		SpaceObject* planet2 = new Planet(2.0);
		planet2->initOsg(Eigen::Vector3d(-15.0 * sin((i+0.5) * rad), -10.0 * cos((i+0.5) * rad), 0.0), 1.0, 1.0);
		planet2->initPhysics(5.972, Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 1.0), Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0));
		_spaceObjects.push_back(planet2);
		planets->addChild(planet2->getModel());
	}

	//osgUtil::Optimizer optimizer;
	//optimizer.optimize(_scene.get());

	return _scene;
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
    //viewer->setUpViewOnSingleScreen(0);
    viewer->setUpViewInWindow(80, 80, 1000, 600, 1);

	viewer->setSceneData(scene);

	osg::ref_ptr<osgGA::TrackballManipulator> manipulator = new osgGA::TrackballManipulator;
	viewer->setCameraManipulator(manipulator);

	osg::Matrix rotation = osg::Matrix::rotate(-osg::PI / .6, osg::X_AXIS);
	osg::Matrix translation = osg::Matrix::translate(0.0f, 0.0f, 75.0f);

	//manipulator->setByMatrix(translation * rotation);
	manipulator->setByMatrix(translation);

	return viewer;
}
