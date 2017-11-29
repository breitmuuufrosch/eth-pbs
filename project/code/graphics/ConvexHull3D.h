#pragma once

#include <Eigen/Core>
#include <osg/Geometry>

#include "CGAL.h"

namespace pbs17 {

	/**
	 * \brief Class which represents the convex-hull and manages the structures between the different frameworks.
	 */
	class ConvexHull3D {
	public:

		/**
		 * \brief Constructor which initializes the convex-hull based on the given vertices.
		 * 
		 * \param vertices
		 *      All vertices for which the bounding-box has to be calculated.
		 */
		ConvexHull3D(osg::Vec3Array* vertices);


		/**
		* \brief Initialize the convex-hull based on the given vertices.
		*
		* \param vertices
		*      All vertices for which the bounding-box has to be calculated.
		*/
		void init(osg::Vec3Array* vertices);


		/**
		 * \brief Get the vertices in a (#V x 3)-matrix.
		 * 
		 * \return (#V x 3)-matrix of vertices.
		 */
		const Eigen::MatrixXd& getVertices() const {
			return _vertices;
		}


		/**
		 * \brief Get the faces in a (#F x 3)-matrix.
		 * 
		 * \return (#F x 3)-matrix of faces.
		 */
		const Eigen::MatrixXi& getFaces() const {
			return _faces;
		}


		/**
		 * \brief Get the osg-model which can be added to the scene-graph.
		 * 
		 * \return Geometry which represents the convex-hull.
		 */
		osg::ref_ptr<osg::Geometry> getOsgModel() const {
			return _osgModel;
		}


		/**
		 * \brief Get the cgal-model which is calculated as the convex-hull from cgal.
		 * 
		 * \return Polyhedron which represents the convex-hull.
		 */
		Polyhedron_3 getCgalModel() const {
			return _cgalModel;
		}


		/**
		* \brief Get the cgal-model which is calculated as the convex-hull from cgal.
		*
		* \return Polyhedron which represents the convex-hull.
		*/
		Nef_Polyhedron_3 getCgalNefModel() const {
			return _cgalNefModel;
		}


		/**
		 * \brief Apply an affine transformation to the model.
		 * 
		 * \param transformation
		 *	    Affine transformation to apply.
		 */
		void transform(Aff_transformation_3 transformation) {
			clock_t begin = clock();
			_cgalNefModel.transform(transformation);
			clock_t end = clock();
			double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
			std::cout << "Transform: " << elapsed_secs << std::endl;
		}


	private:

		//! Vertices which belongs on the convex-hull => (#V x 3)-matrix.
		Eigen::MatrixXd _vertices;
		//! Faced which belongs on the convex-hull => (#F x 3)-matrix. (based on _vertices)
		Eigen::MatrixXi _faces;

		//! Generated geometry which represents the convex-hull in OSG.
		osg::ref_ptr<osg::Geometry> _osgModel;
		//! Generated geometry which represents the convex-hull in CGAL.
		Polyhedron_3 _cgalModel;
		Nef_Polyhedron_3 _cgalNefModel;

	};
}
