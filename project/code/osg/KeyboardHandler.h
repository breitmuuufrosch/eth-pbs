#pragma once

#include "osgGA/GUIEventHandler"

#include "../scene/SpaceObject.h"
#include "../physics/SimulationManager.h"

namespace pbs17 {
	class KeyHandler : public osgGA::GUIEventHandler {
	public:
		KeyHandler(const std::vector<SpaceObject*> &objects)
			: _sceneManager(objects), _showBoundingBox(true), _showConvexHull(true), _isPaused(false) {}


		bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter&) override {
			switch (ea.getEventType()) {
			case osgGA::GUIEventAdapter::KEYDOWN:
			{
				switch (ea.getKey()) {
				case osgGA::GUIEventAdapter::KEY_B:
				{
					_showBoundingBox = !_showBoundingBox;

					for (auto it = _sceneManager.begin(); it != _sceneManager.end(); ++it) {
						(*it)->getModel()->setValue(1, _showBoundingBox);
					}

					return true;

				}
				case osgGA::GUIEventAdapter::KEY_C:
				{
					_showConvexHull = !_showConvexHull;

					for (auto it = _sceneManager.begin(); it != _sceneManager.end(); ++it) {
						(*it)->getConvexSwitch()->setValue(0, !_showConvexHull);
						(*it)->getConvexSwitch()->setValue(1, _showConvexHull);
					}

					return true;
				}
				case osgGA::GUIEventAdapter::KEY_P:
				{
					_isPaused = !_isPaused;

					SimulationManager::setIsPaused(_isPaused);
				}
				}
			}

			default:
				return false;
			}
		}

	private:

		std::vector<SpaceObject*> _sceneManager;

		bool _showBoundingBox;
		bool _showConvexHull;
		bool _isPaused;
	};
}