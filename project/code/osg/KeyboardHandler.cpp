#pragma once

#include "KeyboardHandler.h"

using namespace pbs17;
	
bool KeyboardHandler::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter&) {
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

			return true;
		}
		case osgGA::GUIEventAdapter::KEY_Plus:
		case osgGA::GUIEventAdapter::KEY_KP_Add:
		{
			SimulationManager::increaseSimulationDt(0.01);

			return true;
		}
		case osgGA::GUIEventAdapter::KEY_Minus:
		case osgGA::GUIEventAdapter::KEY_KP_Subtract:
		{
			SimulationManager::decreaseSimulationDt(0.01);

			return true;
		}
		default:
			return false;
		}
	}

	default:
		return false;
	}
}