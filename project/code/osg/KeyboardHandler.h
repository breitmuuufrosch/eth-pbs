#pragma once

#include "osgGA/GUIEventHandler"

#include "../scene/SpaceObject.h"

namespace pbs17 {
	class KeyHandler : public osgGA::GUIEventHandler {
	public:
		KeyHandler(const std::vector<SpaceObject*> &objects)
			: _sceneManager(objects), _showBoundingBox(true), _showConvexHull(true) {}


		bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter&) override {
			switch (ea.getEventType()) {
			case(osgGA::GUIEventAdapter::KEYDOWN):
			{
				if (ea.getKey() == osgGA::GUIEventAdapter::KEY_B) {
					// Place your code here...
					_showBoundingBox = !_showBoundingBox;

					for (auto it = _sceneManager.begin(); it != _sceneManager.end(); ++it) {
						(*it)->getModel()->setValue(1, _showBoundingBox);
					}

					return true;
				}

				if (ea.getKey() == osgGA::GUIEventAdapter::KEY_C) {
					// Place your code here...
					_showConvexHull = !_showConvexHull;

					for (auto it = _sceneManager.begin(); it != _sceneManager.end(); ++it) {
						(*it)->getConvexSwitch()->setValue(0, !_showConvexHull);
						(*it)->getConvexSwitch()->setValue(1, _showConvexHull);
					}

					return true;
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
	};
}