/**
 * \brief Starting point for the program.
 *
 * \Author: Alexander Lelidis (14-907-562), Andreas Emch (08-631-384), Uroš Tešić (17-950-346)
 * \Date:   2017-11-11
 */

#include <osg/Node>
#include <osgDB/ReadFile>
#include <osgViewer/Viewer>
#include <boost/program_options.hpp>
#include <string>

#include <iomanip>
#include "scene/SceneManager.h"
#include "physics/SimulationManager.h"
#include <json.hpp>
#include "config.h"

using namespace boost::program_options;
// for convenience
using json = nlohmann::json;

osg::Light *createLight(osg::Vec4 color)
{
	osg::Light *light = new osg::Light();

	// each light must have a unique number
	light->setLightNum(0);

	// we set the light's position via a PositionAttitudeTransform object
	light->setPosition(osg::Vec4(0.0, 0.0, 0.0, 1.0));
	light->setDiffuse(color);
	light->setSpecular(osg::Vec4(1.0, 1.0, 1.0, 1.0));
	light->setAmbient(osg::Vec4(0.0, 0.0, 0.0, 1.0));

	return light;
}

int main(int argc, const char *argv[]) {
    pbs17::SceneManager* sceneManager = new pbs17::SceneManager;
    osg::ref_ptr<osg::Node> scene = nullptr;

	std::cout << std::fixed << std::setprecision(6);
    variables_map vm;

	try {
		options_description desc{ "Options" };
		desc.add_options()
			("help,h", "Help screen")
			("spheres,s", value<int>()->default_value(10), "Spheres")
			("asteroids,a", value<int>()->default_value(0), "Asteroids")
			("emitter,e", value<std::string>()->default_value("sphere"), "Emitter")
			("rand,r", value<bool>()->default_value(true), "Random")
			("sceneJson,j", value<std::string>(), "Json file containing the scene");


		store(parse_command_line(argc, argv, desc), vm);
		notify(vm);

		if (vm.count("help")) {
			std::cout << desc << '\n';
			return 0;
		} else if (vm.count("sceneJson")) {
			const std::string jsonFilePath = vm["sceneJson"].as<std::string>();

			// check if file exists
			std::ifstream stream(jsonFilePath);
			if (!stream) {
				stream = std::ifstream(SCENES_PATH + "/" + jsonFilePath);

				if (!stream) {
					std::cout << "File " + jsonFilePath + " doesnt exists!" << std::endl;
					return 0;
				}
			}

			// read json files
			json j;
			stream >> j;

			// load scene with json file
			std::cout << "load scene with json: " << jsonFilePath << '\n';
			scene = sceneManager->loadScene(j);
		} else {
			// load scene with parameters
			std::cout << "load scene with parameters" << '\n';
			scene = sceneManager->loadScene(vm);
		}
	}
	catch (const error &ex) {
		std::cerr << ex.what() << '\n';
	}






    //osg::ref_ptr<osg::Node> scene = sceneManager->loadScene();
	osg::ref_ptr<osgViewer::Viewer> viewer = sceneManager->initViewer(scene);

	osg::StateSet *lightStateSet = scene->getOrCreateStateSet();
	osg::LightSource *lightSource = new osg::LightSource();
	lightSource->setLight(createLight(osg::Vec4(1.0, 0.0, 0.0, 1.0)));
	lightSource->setLocalStateSetModes(osg::StateAttribute::ON);
	lightSource->setStateSetModes(*lightStateSet, osg::StateAttribute::ON);
	scene->asGroup()->addChild(lightSource);

	pbs17::SimulationManager* simulationManager = new pbs17::SimulationManager(sceneManager->getSpaceObjects());

	double startTime = 0.0;

	while (!viewer->done()) {
		viewer->frame();

		long frameNumber = viewer->getFrameStamp()->getFrameNumber();
		double currentTime = viewer->elapsedTime();
		double dt = currentTime - startTime;
		//std::cout << "Frame: " << frameNumber << "\tdt: " << dt << "\tfps: " << 1.0 / dt << std::endl;

		dt = pbs17::SimulationManager::getSimulationDt();
		simulationManager->simulate(dt);

		startTime = currentTime;
	}

	delete sceneManager;
	delete simulationManager;

	return 0;
}
