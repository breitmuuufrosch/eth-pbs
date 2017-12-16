#pragma once

#include <osg/NodeCallback>
#include <osg/MatrixTransform>
#include <osgParticle/PointPlacer>

namespace pbs17 {
	class EmitterUpdateCallback : public osg::NodeCallback {

	public:

		EmitterUpdateCallback(osgParticle::PointPlacer* p, osg::MatrixTransform* mt);

		void operator() (osg::Node* node, osg::NodeVisitor* nv);

	private:

		osgParticle::PointPlacer* _placer;
		osg::MatrixTransform* _trans;
	};
}