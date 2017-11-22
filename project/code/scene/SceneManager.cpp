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

    // add the skybox
    addSkybox();


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
        SpaceObject* asteroid1 = new Asteroid();
		asteroid1->initOsg(Eigen::Vector3d(-10.0 * sin(i * rad), -10.0 * cos(i * rad), 0.0), 1.0, (i+1.0) / (numObjects));
		asteroid1->initPhysics(5.972, Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 1.0), Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0));
		_spaceObjects.push_back(asteroid1);
        planets->addChild(asteroid1->getModel());

		SpaceObject* planet2 = new Planet(2.0);
		planet2->initOsg(Eigen::Vector3d(-15.0 * sin((i+0.5) * rad), -10.0 * cos((i+0.5) * rad), 0.0), 1.0, 1.0);
		planet2->initPhysics(5.972, Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 1.0), Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0));
		_spaceObjects.push_back(planet2);
		planets->addChild(planet2->getModel());
	}

	return _scene;
}


osg::ref_ptr<osg::Node> SceneManager::loadScene(variables_map vm) {
    _scene = new osg::Group();
    // add the skybox
    addSkybox();

    // check values and with emitter
    bool useSphereEmitter = vm["emitter"].as<std::string>() == "sphere";
    bool random = vm["rand"].as<bool>();
    int spheres = vm["spheres"].as<int>();
    int asteroids = vm["asteroids"].as<int>();

    if(useSphereEmitter) {
        sphereEmitter(spheres, asteroids, random);
    } else {
        cubeEmitter(spheres, asteroids, random);
    }


    return _scene;
}

osg::ref_ptr<osg::Node> SceneManager::loadScene(json j) {
    _scene = new osg::Group();
    osg::ref_ptr<osg::Group> planets = new osg::Group;
    _scene->addChild(planets);
    // add the skybox
    addSkybox();

    std::cout << "Loading scene " + j["name"].get<std::string>() << " ";

    std::vector<json> objects = j["objects"].get<std::vector<json>>();
    std::cout << "with " << objects.size() << " objects."  << std::endl;
    for(json d: objects) {
        SpaceObject* so;
        if(d["type"].get<std::string>() == "planet") {
            so = new Planet(d);
        } else if(d["type"].get<std::string>() == "asteroid") {
            so = new Asteroid(d);
        } else {
            std::cout << "Type (" + d["type"].get<std::string>() + ") not supported!" << std::endl;
        }
        _spaceObjects.push_back(so);
        planets->addChild(so->getModel());
    }

    return _scene;
}

void SceneManager::sphereEmitter(int spheres, int asteroids, bool random) {
    double rad = 2.0 * osg::PI / spheres;
    osg::ref_ptr<osg::Group> planets = new osg::Group;
    _scene->addChild(planets);

    for (int i = 0; i < spheres; ++i) {
        SpaceObject* planet2 = new Planet(2.0);
        if(random) {
            float random = ((float) rand()) / (float) RAND_MAX;
            random *= spheres;
            planet2->initOsg(Eigen::Vector3d(-15.0 * sin(random * rad), -10.0 * cos(random * rad), 0.0), 1.0, 1.0);
        } else {
            planet2->initOsg(Eigen::Vector3d(-15.0 * sin(i * rad), -10.0 * cos(i * rad), 0.0), 1.0, 1.0);
        }
        planet2->initPhysics(DEFAULT_MASS,
            Eigen::Vector3d(0.0, 0.0, 0.0),
            Eigen::Vector3d(0.0, 0.0, 1.0),
            Eigen::Vector3d(0.0, 0.0, 0.0),
            Eigen::Vector3d(0.0, 0.0, 0.0));
        _spaceObjects.push_back(planet2);
        planets->addChild(planet2->getModel());
    }

    // ASTEROIDS
    rad = 2.0 * osg::PI / asteroids;
    for (int i = 0; i < asteroids; ++i) {
        SpaceObject* asteroid1 = new Asteroid();

        if(random) {
            float random = ((float) rand()) / (float) RAND_MAX;
            random *= asteroids;
            asteroid1->initOsg(Eigen::Vector3d(-15.0 * sin(random * rad), -10.0 * cos(random * rad), 0.0), 1.0, 1.0);
        } else {
            asteroid1->initOsg(Eigen::Vector3d(-15.0 * sin(i * rad), -10.0 * cos(i * rad), 0.0), 1.0, 1.0);
        }
        asteroid1->initPhysics(DEFAULT_MASS,
            Eigen::Vector3d(0.0, 0.0, 0.0),
            Eigen::Vector3d(0.0, 0.0, 1.0),
            Eigen::Vector3d(0.0, 0.0, 0.0),
            Eigen::Vector3d(0.0, 0.0, 0.0));
        _spaceObjects.push_back(asteroid1);
        planets->addChild(asteroid1->getModel());
    }
}

void SceneManager::cubeEmitter(int spheres, int asteroids, bool random) {
    osg::ref_ptr<osg::Group> planets = new osg::Group;
    _scene->addChild(planets);

    int size = spheres / 2;
    int j = 0;

    for (int i = 0; i < spheres; ++i) {
        SpaceObject* planet2 = new Planet(2.0);
        if(random) {
            float random = ((float) rand()) / (float) RAND_MAX;
            planet2->initOsg(Eigen::Vector3d(15.0 * random, 10.0 * random , 0.0), 1.0, 1.0);
        } else {
            planet2->initOsg(Eigen::Vector3d(15.0 * (i % size), 10.0 * (j % size), 0.0), 1.0, 1.0);
            if(i == size) {
                j++;
            }
        }
        planet2->initPhysics(DEFAULT_MASS,
            Eigen::Vector3d(0.0, 0.0, 0.0),
            Eigen::Vector3d(0.0, 0.0, 1.0),
            Eigen::Vector3d(0.0, 0.0, 0.0),
            Eigen::Vector3d(0.0, 0.0, 0.0));
        _spaceObjects.push_back(planet2);
        planets->addChild(planet2->getModel());
    }

    // ASTEROIDS
    size = asteroids / 2;
    j = 0;

    for (int i = 0; i < asteroids; ++i) {
        SpaceObject* asteroid1 = new Asteroid("A2.obj");
        if(random) {
            float random = ((float) rand()) / (float) RAND_MAX;
            asteroid1->initOsg(Eigen::Vector3d(15.0 * random, 10.0 * random , 0.0), 1.0, 1.0);
        } else {
            asteroid1->initOsg(Eigen::Vector3d(15.0 * (i % size), 10.0 * (j % size), 0.0), 1.0, 1.0);
            if(i == size) {
                j++;
            }
        }
        asteroid1->initPhysics(DEFAULT_MASS,
            Eigen::Vector3d(0.0, 0.0, 0.0),
            Eigen::Vector3d(0.0, 0.0, 1.0),
            Eigen::Vector3d(0.0, 0.0, 0.0),
            Eigen::Vector3d(0.0, 0.0, 0.0));
        _spaceObjects.push_back(asteroid1);
        planets->addChild(asteroid1->getModel());
    }
}


void SceneManager::addSkybox() {
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
    viewer->setUpViewInWindow(80, 80, 1000, 600, 0);

	viewer->setSceneData(scene);

	osg::ref_ptr<osgGA::TrackballManipulator> manipulator = new osgGA::TrackballManipulator;
	viewer->setCameraManipulator(manipulator);

	osg::Matrix rotation = osg::Matrix::rotate(-osg::PI / .6, osg::X_AXIS);
	osg::Matrix translation = osg::Matrix::translate(0.0f, 0.0f, 75.0f);

	//manipulator->setByMatrix(translation * rotation);
	manipulator->setByMatrix(translation);

	return viewer;
}
