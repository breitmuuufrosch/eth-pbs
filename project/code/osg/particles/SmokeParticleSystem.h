#pragma once

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

#include "../../config.h"
#include "EmitterUpdateCallback .h"

namespace pbs17 {
	class SmokeParticleSystem : public osg::Group {
	public:
		SmokeParticleSystem(osg::Group* scene, osg::MatrixTransform* mt) {
			/* Build the particle system in here ... */
			osgParticle::ParticleSystem* dustParticleSystem = new osgParticle::ParticleSystem;
			osg::Geode *geode = new osg::Geode();
			geode->addDrawable(dustParticleSystem);
			this->addChild(geode);

			// Set particle system state
			dustParticleSystem->setDefaultAttributes(DATA_PATH + "/texture/dust.png", true, false);

			// Define the Particle template
			osgParticle::Particle smokeParticle;
			smokeParticle.setSizeRange(osgParticle::rangef(0.1f, 0.5f)); // meters
			smokeParticle.setLifeTime(15); // seconds
			smokeParticle.setMass(0.4f);
			smokeParticle.setColorRange(osgParticle::rangev4(
				osg::Vec4(1.0, 1.0, 1.0, 1.0),
				osg::Vec4(0, 0, 0, 0.3f)));
			dustParticleSystem->setDefaultParticleTemplate(smokeParticle);

			// Add a ParticleSystemUpdater to the ParticleSystem
			osgParticle::ParticleSystemUpdater *dustSystemUpdater = new osgParticle::ParticleSystemUpdater;
			dustSystemUpdater->addParticleSystem(dustParticleSystem);
			this->addChild(dustSystemUpdater);

			// Add a ModularEmitter
			osgParticle::ModularEmitter *emitter = new osgParticle::ModularEmitter;
			emitter->setParticleSystem(dustParticleSystem);
			scene->addChild(emitter);

			osgParticle::RandomRateCounter *dustRate =
				static_cast<osgParticle::RandomRateCounter *>(emitter->getCounter());
			dustRate->setRateRange(40, 100);

			//osgParticle::PointPlacer* placer =
			//	static_cast<osgParticle::PointPlacer *>(emitter->getPlacer());
			//this->setUpdateCallback(new EmitterUpdateCallback(placer, mt));

			osgParticle::RadialShooter* shooter =
				static_cast<osgParticle::RadialShooter *>(emitter->getShooter());
			shooter->setThetaRange(0, 3.15149 / 4.0f); // radians, relative to Z axis.
			shooter->setInitialSpeedRange(0.4, 0.6); // meters/second
			emitter->setShooter(shooter);
		}

	private:
		osgParticle::PointPlacer* placer;
	};
}
