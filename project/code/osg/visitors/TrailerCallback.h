/**
* \brief Functionality for managing loaded models to prevent loading multiple times the same model.
* The code is copied from http://www.vis-sim.com/osg/code/osgcode_bbox1.htm and adapted to our use.
*
* \Author: Alexander Lelidis (14-907-562), Andreas Emch (08-631-384), Uroš Tešić (17-950-346)
* \Date:   2017-11-11
*/

#pragma once

#include <osg/Geometry>
#include <osg/NodeCallback>

#include "../FollowingRibbon.h"

namespace pbs17 {

	/**
	 * \brief Calculate the bounding-box of anode
	 */
	class  TrailerCallback : public osg::NodeCallback {

	public:

		/**
		 * \brief Constructor. Initialize the visitor to traverse all children.
		 */
		TrailerCallback(FollowingRibbon* ribbon, osg::Geometry* geometry)
			: _ribbon(ribbon), _geometry(geometry) {}


		/**
		 * \brief Destructor.
		 */
		virtual ~TrailerCallback() {}


		/**
		 * \brief Calculate the bounding-box for the type osg::Geode.
		 *
		 * \param node
		 *      Current geode-child.
		 * \param nv
		 *	    Visitor of the tree
		 */
		virtual void operator()(osg::Node* node, osg::NodeVisitor* nv);


	protected:

		FollowingRibbon* _ribbon;
		
		//! Target vertex list
		osg::observer_ptr<osg::Geometry> _geometry;

	};
}
