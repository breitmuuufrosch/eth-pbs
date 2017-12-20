/**
 * \brief Implementation for the particle system to simulate smoke.
 *
 * \Author: Alexander Lelidis (14-907-562), Andreas Emch (08-631-384), Uroš Tešić (17-950-346)
 * \Date:   2017-12-12
 */

#pragma once

#include <osg/Group>
#include <osgParticle/PointPlacer>

namespace pbs17 {
	class SmokeParticleSystem : public osg::Group {
	public:
		SmokeParticleSystem(osg::Group* scene, osg::MatrixTransform* mt);

	private:
		osgParticle::PointPlacer* _placer;
	};
}
