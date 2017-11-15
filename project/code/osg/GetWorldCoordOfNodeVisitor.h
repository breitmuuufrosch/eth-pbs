#pragma once

#include <osg/Matrix>

/// Example for me,not yet used
class GetWorldCoordOfNodeVisitor : public osg::NodeVisitor {
public:
	GetWorldCoordOfNodeVisitor() :
		osg::NodeVisitor(NodeVisitor::TRAVERSE_PARENTS), done(false) {
		wcMatrix = new osg::Matrixd();
	}

	void apply(osg::Node &node) override {
		if (!done) {
			if (0 == node.getNumParents()) {
				wcMatrix->set(osg::computeLocalToWorld(this->getNodePath()));
				done = true;
			}

			traverse(node);
		}
	}

	osg::Matrixd* giveUpDaMat() const {
		return wcMatrix;
	}

private:
	bool done;
	osg::Matrix* wcMatrix;
};
