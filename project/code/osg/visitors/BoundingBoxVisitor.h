/**
* \brief Functionality for managing loaded models to prevent loading multiple times the same model.
* The code is copied from http://www.vis-sim.com/osg/code/osgcode_bbox1.htm and adapted to our use.
*
* \Author: Alexander Lelidis (14-907-562), Andreas Emch (08-631-384), Uroš Tešić (17-950-346)
* \Date:   2017-11-11
*/

#pragma once

#include <osg/NodeVisitor>
#include <osg/BoundingBox>
#include <osg/Billboard>

namespace pbs17 {

	/**
	 * \brief Calculate the bounding-box of anode
	 */
	class  CalculateBoundingBox : public osg::NodeVisitor {

	public:

		/**
		 * \brief Constructor. Initialize the visitor to traverse all children.
		 */
		CalculateBoundingBox() : NodeVisitor(TRAVERSE_ALL_CHILDREN) {
			m_transformMatrix.makeIdentity();
		}


		/**
		 * \brief Destructor.
		 */
		virtual ~CalculateBoundingBox() {}


		/**
		 * \brief Calculate the bounding-box for the type osg::Geode.
		 *
		 * \param geode
		 *      Current geode-child.
		 */
		// apply(osg::Geode &geode) override;


		/**
		 * \brief Calculate the bounding-box for the type osg::MatrixTransform.
		 *
		 * \param node
		 *      Current matrix-transform-child.
		 */
		//void apply(osg::MatrixTransform &node) override;


		/**
		 * \brief Calculate the bounding-box for the type osg::Billboard.
		 * important to handle billboard so that its size will not affect the geode size continue traversing the graph
		 *
		 * \param node
		 *      Current billboard-child.
		 */
		//void apply(osg::Billboard &node) override;


		/**
		 * \brief Return the calculated bounding-box.
		 *
		 * \return Calculated bounding-box.
		 */
		osg::BoundingBox &getBoundBox() {
			return m_boundingBox;
		}

	protected:

		//! The overall resultant bounding box.
		osg::BoundingBox m_boundingBox;

		//! The current transform matrix.
		osg::Matrix m_transformMatrix;
	};
}