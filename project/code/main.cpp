/**
 * \brief Starting point for the program.
 * 
 * \Author: Alexander Lelidis (14-907-562), Andreas Emch (08-631-384), Uroš Tešić (17-950-346)
 * \Date:   2017-11-11
 */

#include <osg/Node>
#include <osgDB/ReadFile>
#include <osgViewer/Viewer>

#include <iomanip>
#include "scene/SceneManager.h"
#include "physics/SimulationManager.h"

int main(int, char **) {
	std::cout << std::fixed << std::setprecision(6);

	pbs17::SceneManager* sceneManager = new pbs17::SceneManager;
	osg::ref_ptr<osg::Node> scene = sceneManager->loadScene();
	osg::ref_ptr<osgViewer::Viewer> viewer = sceneManager->initViewer(scene);

	pbs17::SimulationManager* simulationManager = new pbs17::SimulationManager(sceneManager->getSpaceObjects());
	
	double startTime = 0.0;

	while (!viewer->done()) {
		viewer->frame();

		long frameNumber = viewer->getFrameStamp()->getFrameNumber();
		double currentTime = viewer->elapsedTime();
		double dt = currentTime - startTime;
		 //std::cout << "Frame: " << frameNumber << "\tdt: " << dt << "\tfps: " << 1.0 / dt << std::endl;

		 dt = 0.01;
		simulationManager->simulate(dt);

		startTime = currentTime;
	}

	delete sceneManager;
	delete simulationManager;

	return 0;
}
