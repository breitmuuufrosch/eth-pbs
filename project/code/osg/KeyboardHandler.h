#pragma once

#include "osgGA/GUIEventHandler"

#include "../scene/SpaceObject.h"
#include "../physics/SimulationManager.h"

namespace pbs17 {
	class KeyboardHandler : public osgGA::GUIEventHandler {
	public:
		KeyboardHandler(const std::vector<SpaceObject*> &objects)
			: _sceneManager(objects), _showBoundingBox(true), _showConvexHull(true), _isPaused(false) {}


		bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter&) override;

	private:

		std::vector<SpaceObject*> _sceneManager;

		bool _showBoundingBox;
		bool _showConvexHull;
		bool _isPaused;
	};
}