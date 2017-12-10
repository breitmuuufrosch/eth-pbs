#pragma once

#include <osg/NodeCallback>
#include <osg/MatrixTransform>
#include <osg/Group>
#include <osgParticle/PointPlacer>
#include <osgParticle/Particle>
#include <osgParticle/ParticleSystem>
#include <osgParticle/ParticleSystemUpdater>
#include <osgParticle/ModularEmitter>
#include <osgParticle/RandomRateCounter>
#include <osgParticle/ModularProgram>
#include <osgParticle/FluidFrictionOperator>
#include <osgParticle/AccelOperator>

namespace pbs17 {
	class EmitterUpdateCallback : public osg::NodeCallback {

	private:

		osgParticle::PointPlacer* placer;
		osg::MatrixTransform* trans;

	public:

		EmitterUpdateCallback(osgParticle::PointPlacer* p, osg::MatrixTransform* mt) {
			placer = p;
			trans = mt;
		}

		void operator() (osg::Node* node, osg::NodeVisitor* nv) {
			placer->setCenter(trans->getMatrix().getTrans());
			traverse(node, nv);
		}
	};
}