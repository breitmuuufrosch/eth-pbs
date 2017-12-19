/**
 * \brief Implementation of the n-body simulation and interaction.
 *
 * \Author: Alexander Lelidis (14-907-562), Andreas Emch (08-631-384), Uroš Tešić (17-950-346)
 * \Date:   2017-11-11
 */

#pragma once

#include <Eigen/Core>
#include <vector>
#include <map>

// Forward declarations
namespace pbs17 {
	class SpaceObject;
}

namespace pbs17 {

	/**
	 * \brief The manager to handle the collision (within all needed calculations) to simulate one frame.
	 */
	class NBodyManager {
	public:
		/**
		 * \brief Constructor of the CollisionManager.
		 *
		 */
		NBodyManager();


		/**
		 * \brief Simulate the scene.
		 *
		 * \param dt
		 *      Time difference since between the last frames.
		 * \param spaceObjects
		 *      All space-objects in the scene.
		 */
		void simulateStep(double dt, std::vector<SpaceObject*> &spaceObjects);


		void initSpatialGrid(std::vector<SpaceObject*> &spaceObjects, Eigen::Vector3i gridResolution);

		
	private:
		//CONST
		const double G = 1.0; // 6.67408 * pow(10.0, -4.0);
		const double EPS = 0.000000001;

		bool _useSpatialGrid = false;

		Eigen::Vector3d _spatialBbMin;
		Eigen::Vector3d _spatialBbMax;

		Eigen::Vector3i _resolutionSize;
		Eigen::Vector3d _resolution;;

		double _treshold = 0.1;

		std::vector<std::vector<SpaceObject*>> _spatialGrid;
		std::vector<std::map<long, SpaceObject*>> _influencer;
		Eigen::MatrixXi _spatialPosition;
		Eigen::MatrixXi _influenceRadius;
		
		/**
		 * \brief Get the position of the point in the spatial-index-grid for a given point.
		 *
		 * \param position Position of the point which one want to know the index in the spatial-grid
		 *
		 * \return Position in the spatial-index
		 */
		Eigen::Vector3i binSpatialInformation(Eigen::Vector3d position);

		/**
		 * \brief Get the position of the point in the spatial-index-grid for a given point.
		 *
		 * \param position Position of the point which one want to know the index in the spatial-grid
		 *
		 * \return Position in the spatial-index
		 */
		unsigned int getIndex(Eigen::Vector3i position);

		void addInfluencer(unsigned int i, SpaceObject* spaceObject, Eigen::Vector3i position);
		void removeInfluencer(unsigned int i, SpaceObject* spaceObject, Eigen::Vector3i position);

	};

}
