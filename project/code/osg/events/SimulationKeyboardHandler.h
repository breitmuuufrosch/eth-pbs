#pragma once

#include "KeyboardHandler.h"


namespace pbs17 {
	class SimulationKeyboardHandler : public KeyboardHandler {
	public:
		SimulationKeyboardHandler(const std::vector<SpaceObject*> &objects)
			: KeyboardHandler(objects, true, false, false) {}

		bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter&) override;

	};
}
