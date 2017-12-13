#pragma once

#include <osg/Group>
#include <osg/NodeCallback>
#include <osg/MatrixTransform>
#include <osgParticle/PointPlacer>

namespace pbs17 {
	class EmitterUpdateCallback : public osg::NodeCallback {

	public:

		EmitterUpdateCallback(osgParticle::PointPlacer* p, osg::MatrixTransform* mt) {
			_placer = p;
			_trans = mt;
		}

		void operator() (osg::Node* node, osg::NodeVisitor* nv) {
			//_placer->setCenter(_trans->getMatrix().getTrans());
			traverse(node, nv);
		}

	private:

		osgParticle::PointPlacer* _placer;
		osg::MatrixTransform* _trans;
	};
}