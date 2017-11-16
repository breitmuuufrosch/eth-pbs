#pragma once

#include "../scene/SpaceObject.h"


namespace pbs17 {
	class Collision {
	private:
		//! Pointer to the first colliding object
		SpaceObject* _firstObject;
		//! Pointer to the second colliding object
		SpaceObject* _secondObject;
		//! Unit vector of the collision normal
		Eigen::Vector3d _unitNormal;
		//! Point of contact of the first object (in world coordinates)
		Eigen::Vector3d _firstPOC;
		//! Point of contact of the second object (in world coordinates)
		Eigen::Vector3d _secondPOC;
		//! Represents the displacement vector needed to annul collision
		Eigen::Vector3d _intersectionVector;
	public:
		SpaceObject* getFirstObject() const {
			return _firstObject;
		}

		void setFirstObject(SpaceObject * so) {
			_firstObject = so;
		}

		SpaceObject* getSecondObject() const {
			return _secondObject;
		}

		void setSecondObject(SpaceObject * so) {
			_secondObject = so;
		}

		Eigen::Vector3d getUnitNormal() const {
			return _unitNormal;
		}

		void setUnitNormal(Eigen::Vector3d un) {
			_unitNormal = un;
		}

		Eigen::Vector3d getFirstPOC() const {
			return _firstPOC;
		}

		void setFirstPOC(Eigen::Vector3d poc) {
			_firstPOC = poc;
		}

		Eigen::Vector3d getSecondPOC() const {
			return _secondPOC;
		}

		void setSecondPOC(Eigen::Vector3d poc) {
			_secondPOC = poc;
		}

		Eigen::Vector3d getIntersectionVector() const {
			return _intersectionVector;
		}

		void setIntersectionVector(Eigen::Vector3d iv) {
			_intersectionVector = iv;
		}
	};

	struct CollisionCompareLess {
		bool operator() (Collision & lhs, Collision & rhs) {
			return lhs.getIntersectionVector().norm() < rhs.getIntersectionVector().norm();
		}
	};
}

