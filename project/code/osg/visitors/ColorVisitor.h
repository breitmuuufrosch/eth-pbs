/**
 * \brief Color visitor to colorize a node.
 *
 * \Author: Alexander Lelidis (14-907-562), Andreas Emch (08-631-384), Uroš Tešić (17-950-346)
 * \Date:   2017-11-16
 */

#pragma once

#include <osg/NodeVisitor>

namespace osg {
	class Array;
	class Node;
	class Geometry;
}


namespace pbs17 {
	/**
	 * \brief Change the color to a given node and subchilds.
	 */
	class ColorVisitor : public osg::NodeVisitor {
	public:

		/**
		 * \brief Constructor with the color to apply to the node.
		 * 
		 * \param color
		 *      The color which this visitor applies.
		 */
		ColorVisitor(const osg::Vec4 &color);


		/**
		 * \brief Destructor.
		 */
		virtual ~ColorVisitor();


		/**
		 * \brief Handle traversal of osg::Node node types
		 * 
		 * \param node
		 *      The node to traverse in the graph in order to apply the color.
		 */
		void apply(osg::Node &node) override;


		/**
		 * \brief Handle traversal of osg::Group node types
		 * 
		 * \param group
		 *      The group to traverse in the graph in order to apply the color.
		 */
		void apply(osg::Group &group) override;


		/**
		 * \brief Handle traversal of osg::Geode node types
		 * 
		 * \param geode
		 *      The geode to traverse in the graph in order to apply the color.
		 */
		void apply(osg::Geode &geode) override;


		void apply(osg::MatrixTransform &matrixTransform) override;


	private:

		//! The color to apply to the node and children.
		osg::Vec4 _color;

		//! The colors-array which is used to apply the color to the geometry.
		osg::ref_ptr<osg::Vec4Array> _colorArrays;
	};
}
