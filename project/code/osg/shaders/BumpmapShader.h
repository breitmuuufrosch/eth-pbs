/*!
\file
* \brief Header file for the class BumpmapShader.
*
* $Author: emcha1/pfafd1 $
* $Date: 2013-01-12 $
* $Revision: 1 $
*/

#pragma once

#include "Shader.h"
#include <osg/Texture2D>

namespace pbs17 {
	//! The BumpmapShader is used to simulate a simple cartoon-effect.
	class BumpmapShader : public Shader {

	public:

		//! Constructor
		/*!
		Initializes the shader-programms.
		\param color The color for which the effect should be applied.
		*/
		BumpmapShader(osg::ref_ptr<osg::Texture2D> texture, osg::ref_ptr<osg::Texture2D> normals);


		//! Destructor.
		virtual ~BumpmapShader();


		//! Apply the shader to the given node.
		/*!
		\param node The node to which the shader should be applied.
		*/
		void apply(osg::Node* node) override;


	private:

		osg::ref_ptr<osg::Texture2D> _texture;
		osg::ref_ptr<osg::Texture2D> _normals;
	};
}

