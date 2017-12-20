/**
 * \brief Implementation for the keyboard-interaction in the simulation-mode.
 *
 * \Author: Alexander Lelidis (14-907-562), Andreas Emch (08-631-384), Uroš Tešić (17-950-346)
 * \Date:   2017-12-05
 */

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
