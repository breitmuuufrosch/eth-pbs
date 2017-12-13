/*!
\file
* \brief Header file for the class CartoonShader.
*
* $Author: emcha1/pfafd1 $
* $Date: 2013-01-12 $
* $Revision: 1 $
*/

#pragma once

#include "Shader.h"

namespace pbs17 {
	//! The CartoonShader is used to simulate a simple cartoon-effect.
	class CartoonShader : public Shader {

	public:

		//! Constructor
		/*!
		Initializes the shader-programms.
		\param color The color for which the effect should be applied.
		*/
		CartoonShader(osg::Vec4d color);


		//! Destructor.
		virtual ~CartoonShader();


		//! Apply the shader to the given node.
		/*!
		\param node The node to which the shader should be applied.
		*/
		void apply(osg::Node* node) override;


	private:

		//! The color for which the effect should be applied. (With this color the 4 different lightened colors will be defined in order to make the different parts of illumination)
		osg::Vec4d _color;

	};
}

