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
#include "../osg/KeyboardHandler.h"

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

    int numObjects = 1;
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

std::vector<Eigen::Vector2d> SceneManager::getSamples(int pointCount, bool random) {
    std::vector<Eigen::Vector2d> res;
    int sqrtVal = (int) (std::sqrt((float) pointCount) + 0.5f);
    double invSqrtVal = 1.f / sqrtVal;
    pointCount = sqrtVal*sqrtVal;

    for (int i = 0; i < pointCount; ++i) {
        if(random) {
            double x = ((double) rand()) / (double) RAND_MAX;
            double y = ((double) rand()) / (double) RAND_MAX;
            res.push_back(Eigen::Vector2d(x, y));
        } else {
            int y = i / sqrtVal, x = i % sqrtVal;
            res.push_back(Eigen::Vector2d((x + 0.5) * invSqrtVal, (y + 0.5) * invSqrtVal));
        }
    }
    return res;
}

std::vector<Eigen::Vector3d> SceneManager::getSamples3(int pointCount, bool random) {
    std::vector<Eigen::Vector3d> res;
    int sqrtVal = (int) (std::cbrt((float) pointCount) + 0.5f);
    double invSqrtVal = 1.f / sqrtVal;
    pointCount = sqrtVal*sqrtVal*sqrtVal;
    if(random) {
        for (int i = 0; i < pointCount; ++i) {
            double x = ((double) rand()) / (double) RAND_MAX;
            double y = ((double) rand()) / (double) RAND_MAX;
            double z = ((double) rand()) / (double) RAND_MAX;
            res.push_back(Eigen::Vector3d(x, y, z));
        }
    } else {
        for (int x = 0; x < sqrtVal; ++x) {
            for (int y = 0; y < sqrtVal; ++y) {
                for (int z = 0; z < sqrtVal; ++z) {
                     res.push_back(Eigen::Vector3d((x + 0.5) * invSqrtVal, (y + 0.5) * invSqrtVal, (z + 0.5) * invSqrtVal));
                }
            }
        }
    }
    return res;
}


void SceneManager::sphereEmitter(int spheres, int asteroids, bool random) {
    double radius = 15.0;
    osg::ref_ptr<osg::Group> planets = new osg::Group;
    _scene->addChild(planets);

    std::vector<Eigen::Vector2d> samples = getSamples(spheres, random);
    // for spheres
    for (int i = 0; i < spheres; ++i) {
        SpaceObject* planet2 = new Planet(2.0);
        //  Point2f(2.0f * hightStop * sample(1) + hightStart, 2.0F * M_PI  * sample(0));
        Eigen::Vector2d cylSample = Eigen::Vector2d(2.0 * samples[i].y() - 1.0, 2.0 * osg::PI * samples[i].x());
        double omegaZ = cylSample(0);
        double r = sqrt(1.0 -  omegaZ * omegaZ);
        double phi = cylSample(1);
        double omegaX = r * cos(phi);
        double omegaY = r * sin(phi);

        planet2->initOsg(Eigen::Vector3d(radius * omegaX, radius * omegaY, radius * omegaZ), 1.0, 1.0);

        planet2->initPhysics(DEFAULT_MASS,
            Eigen::Vector3d(0.0, 0.0, 0.0),
            Eigen::Vector3d(0.0, 0.0, 1.0),
            Eigen::Vector3d(0.0, 0.0, 0.0),
            Eigen::Vector3d(0.0, 0.0, 0.0));
        _spaceObjects.push_back(planet2);
        planets->addChild(planet2->getModel());
    }

    // ASTEROIDS
    samples = getSamples(asteroids, random);

    for (int i = 0; i < asteroids; ++i) {
        SpaceObject* asteroid1 = new Asteroid();

        Eigen::Vector2d cylSample = Eigen::Vector2d(2.0 * samples[i].y() - 1.0, 2.0 * osg::PI * samples[i].x());
        double omegaZ = cylSample(0);
        double r = sqrt(1.0 -  omegaZ * omegaZ);
        double phi = cylSample(1);
        double omegaX = r * cos(phi);
        double omegaY = r * sin(phi);

        asteroid1->initOsg(Eigen::Vector3d(radius * omegaX, radius * omegaY, radius * omegaZ), 1.0, 1.0);

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

    double sideLength = 15.0;

    std::vector<Eigen::Vector3d> samples = getSamples3(spheres, random);


    for (int i = 0; i < spheres; ++i) {
        SpaceObject* planet2 = new Planet(2.0);

        planet2->initOsg(samples[i] * sideLength, 1.0, 1.0);
        planet2->initPhysics(DEFAULT_MASS,
            Eigen::Vector3d(0.0, 0.0, 0.0),
            Eigen::Vector3d(0.0, 0.0, 1.0),
            Eigen::Vector3d(0.0, 0.0, 0.0),
            Eigen::Vector3d(0.0, 0.0, 0.0));
        _spaceObjects.push_back(planet2);
        planets->addChild(planet2->getModel());
    }

    // ASTEROIDS
    samples = getSamples3(asteroids, random);

    for (int i = 0; i < asteroids; ++i) {
        SpaceObject* asteroid1 = new Asteroid("A2.obj");
        asteroid1->initOsg(samples[i] * sideLength, 1.0, 1.0);

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
	viewer->addEventHandler(new KeyHandler(_spaceObjects));

	osg::ref_ptr<osgGA::TrackballManipulator> manipulator = new osgGA::TrackballManipulator;
	viewer->setCameraManipulator(manipulator);

	osg::Matrix rotation = osg::Matrix::rotate(-osg::PI / .6, osg::X_AXIS);
	osg::Matrix translation = osg::Matrix::translate(0.0f, 0.0f, 15.0f);

	//manipulator->setByMatrix(translation * rotation);
	manipulator->setByMatrix(translation);

	return viewer;
}
