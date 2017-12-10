#include "NBodyManager.h"

#include <math.h>

using namespace pbs17;
using namespace Eigen;
using namespace std;


NBodyManager::NBodyManager() { }


void NBodyManager::simulateStep(double dt, std::vector<SpaceObject *> &spaceObjects) {

    //CONST
	double G = 1.0; // 6.67408 * pow(10.0, -4.0);
    double EPS = 0.000000001;


    int cntSpaceObj = spaceObjects.size();


    // initialize the forces
    vector<Vector3d> forces(cntSpaceObj);
    for (int i = 0; i < cntSpaceObj; ++i) {
        forces[i] = Vector3d(0.0, 0.0, 0.0);
    }


    for (int i = 0; i < cntSpaceObj; ++i) {
        SpaceObject* curObject = spaceObjects[i];
        // Calculate a rotation around the rotation-center of the object
        Vector3d curCenter = curObject->getPosition();
        double m = curObject->getMass();

        // acceleration vector

        for (int j = 0; j < cntSpaceObj; ++j) {
            if (i == j) continue; // do not compare the object with it self

            // compare the objects based on the center of mass
            SpaceObject* compareObject = spaceObjects[j];
            Vector3d compareCenter = compareObject->getPosition();

            // get the distance
            Vector3d d = compareCenter - curCenter;

            // compute the square distance
            double r = (compareCenter - curCenter).norm();
            r *= r;

            // F is the force between the masses
            double f = (G * m * compareObject->getMass()) / (r + EPS);
            forces[i] += f * d.normalized();
        }
    }

    // update positions
    for (int i = 0; i < cntSpaceObj; ++i) {
        SpaceObject* spaceObject = spaceObjects[i];
        Vector3d a = forces[i] / spaceObject->getMass();
        Vector3d v = spaceObject->getLinearVelocity() + (dt * a);
        spaceObject->setLinearVelocity(v);

        Vector3d dtv = dt * v;
        Vector3d p = spaceObject->getPosition() + dtv;

		Vector3d dto = dt * spaceObject->getAngularVelocity();
		Vector3d o = spaceObject->getOrientation() + dto;

		spaceObject->updatePositionOrientation(p, o);
    }
}
