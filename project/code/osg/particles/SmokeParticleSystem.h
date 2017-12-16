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
